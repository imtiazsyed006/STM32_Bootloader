#!/usr/bin/env python3
import os
import socket
import struct
import threading
import time
import queue
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

# ---------------------- Network defaults ----------------------
DEFAULT_IP      = "192.168.0.25"
PORT_CMD_TX     = 2001   # host -> device (send ETX packets)
PORT_REPLY_RX   = 2000   # device -> host (ETX RESPONSE + any text)

# ---------------------- Protocol constants ----------------------
ETX_OTA_SOF  = 0xAA
ETX_OTA_EOF  = 0xBB
ETX_OTA_ACK  = 0x00
ETX_OTA_NACK = 0x01

ETX_OTA_PACKET_TYPE_CMD      = 0
ETX_OTA_PACKET_TYPE_DATA     = 1
ETX_OTA_PACKET_TYPE_HEADER   = 2
ETX_OTA_PACKET_TYPE_RESPONSE = 3

ETX_OTA_CMD_START = 0
ETX_OTA_CMD_END   = 1
ETX_OTA_CMD_ABORT = 2

ETX_OTA_DATA_MAX_SIZE = 1024

# ---------------------- CRC32 (STM32 hardware polynomial) ----------------------
CRC32_POLY = 0x04C11DB7
CRC32_INIT = 0xFFFFFFFF

def crc32_stm32(data: bytes) -> int:
    crc = CRC32_INIT
    for b in data:
        crc ^= (b << 24) & 0xFFFFFFFF
        for _ in range(8):
            if (crc & 0x80000000) != 0:
                crc = ((crc << 1) ^ CRC32_POLY) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF
    return crc & 0xFFFFFFFF

# ---------------------- Packet builders ----------------------
def pack_u16_le(v: int) -> bytes:
    return struct.pack('<H', v)

def pack_u32_le(v: int) -> bytes:
    return struct.pack('<I', v)

def make_cmd_packet(cmd: int) -> bytes:
    payload = bytes([cmd])
    crc = crc32_stm32(payload)
    return bytes([ETX_OTA_SOF, ETX_OTA_PACKET_TYPE_CMD]) + pack_u16_le(1) + payload + pack_u32_le(crc) + bytes([ETX_OTA_EOF])

def make_header_packet(total_size: int, total_crc: int) -> bytes:
    meta = pack_u32_le(total_size) + pack_u32_le(total_crc) + pack_u32_le(0) + pack_u32_le(0)
    crc = crc32_stm32(meta)
    return bytes([ETX_OTA_SOF, ETX_OTA_PACKET_TYPE_HEADER]) + pack_u16_le(16) + meta + pack_u32_le(crc) + bytes([ETX_OTA_EOF])

def make_data_packet(chunk: bytes) -> bytes:
    crc = crc32_stm32(chunk)
    return bytes([ETX_OTA_SOF, ETX_OTA_PACKET_TYPE_DATA]) + pack_u16_le(len(chunk)) + chunk + pack_u32_le(crc) + bytes([ETX_OTA_EOF])

# ---------------------- TCP uploader with background reader ----------------------
class STM32TcpUploader:
    def __init__(self, ip: str, timeout: float = 1.0, retries: int = 3, inter_packet_delay: float = 0.002):
        self.ip = ip
        self.timeout = timeout
        self.retries = retries
        self.inter_packet_delay = inter_packet_delay

        self.cmd_sock: socket.socket | None = None
        self.reply_sock: socket.socket | None = None

        # reader thread state (for port 2000)
        self._reader = None
        self._stop_reader = threading.Event()
        self._resp_queue: queue.Queue[int] = queue.Queue()
        self._rx_buf = bytearray()
        self._log_cb = lambda s: None
        self._lock = threading.Lock()  # protects _rx_buf

    def set_log_cb(self, cb):
        self._log_cb = cb or (lambda s: None)

    def open(self):
        # Command socket (send ETX frames)
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.cmd_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.cmd_sock.settimeout(self.timeout)
        self.cmd_sock.connect((self.ip, PORT_CMD_TX))

        # Reply socket (read ETX RESPONSE frames; might also carry text)
        self.reply_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.reply_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.reply_sock.settimeout(self.timeout)
        self.reply_sock.connect((self.ip, PORT_REPLY_RX))

        # Start background reader on reply_sock
        self._stop_reader.clear()
        self._reader = threading.Thread(target=self._reply_reader_loop, daemon=True)
        self._reader.start()

        time.sleep(0.05)

    def close(self):
        self._stop_reader.set()
        try:
            if self.reply_sock:
                try:
                    self.reply_sock.shutdown(socket.SHUT_RDWR)
                except:
                    pass
                self.reply_sock.close()
        finally:
            self.reply_sock = None

        try:
            if self.cmd_sock:
                try:
                    self.cmd_sock.shutdown(socket.SHUT_RDWR)
                except:
                    pass
                self.cmd_sock.close()
        finally:
            self.cmd_sock = None

    # ---------------------- Reader thread on port 2000 ----------------------
    def _reply_reader_loop(self):
        sock = self.reply_sock
        if not sock:
            self._log_cb("[RX] reply socket not set")
            return

        sock.settimeout(0.2)
        while not self._stop_reader.is_set():
            try:
                data = sock.recv(4096)
                if not data:
                    self._log_cb("[RX] connection closed by device")
                    break
                with self._lock:
                    self._rx_buf.extend(data)
                    self._drain_rx_buffer()
            except socket.timeout:
                continue
            except Exception as e:
                self._log_cb(f"[RX] error: {e}")
                break
        self._log_cb("[RX] reader thread exiting")

    def _drain_rx_buffer(self):
        """Parse and log everything currently in _rx_buf (port 2000 only):
           - Any bytes before SOF are treated as text and logged.
           - RESPONSE frames parsed strictly like the UART path:
             [SOF][TYPE=3][LEN=1][STATUS][CRC(4)][EOF]
           - Non-RESPONSE frames do not bulk-consume buffer; drop 1 byte to resync.
        """
        buf = self._rx_buf

        while True:
            # 1) flush leading non-SOF as text (printable/viewable)
            if buf and buf[0] != ETX_OTA_SOF:
                try:
                    nxt = buf.index(ETX_OTA_SOF)
                    chunk = bytes(buf[:nxt]); del buf[:nxt]
                except ValueError:
                    chunk = bytes(buf); buf.clear()
                if chunk:
                    try:
                        text = chunk.decode('utf-8', errors='replace').replace('\r', '')
                        for line in text.split('\n'):
                            if line:
                                self._log_cb(f"[DEV] {line}")
                    except Exception:
                        hexstr = ' '.join(f"{b:02X}" for b in chunk)
                        #self._log_cb(f"[DEV] {hexstr}")
                if not buf:
                    return

            # Need at least SOF + TYPE + LEN
            if len(buf) < 4:
                return
            if buf[0] != ETX_OTA_SOF:
                del buf[0]
                continue

            typ = buf[1]

            # Only trust LEN if this is a RESPONSE frame
            if typ != ETX_OTA_PACKET_TYPE_RESPONSE:
                del buf[0]  # drop SOF and keep hunting
                continue

            # RESPONSE must have LEN==1
            data_len = buf[2] | (buf[3] << 8)
            if data_len != 1:
                del buf[0]
                continue

            total = 1 + 1 + 2 + 1 + 4 + 1  # SOF+TYPE+LEN+STATUS+CRC+EOF
            if len(buf) < total:
                return  # wait for more bytes

            # Extract exact frame
            frame = bytes(buf[:total]); del buf[:total]

            # EOF check
            if frame[-1] != ETX_OTA_EOF:
                continue

            status = frame[4]
            rec_crc = struct.unpack('<I', frame[5:9])[0]
            cal_crc = crc32_stm32(bytes([status]))

            # Tolerate CRC mismatch for RESPONSE only (common HAL word-mode mismatch)
            if cal_crc != rec_crc:
                self._log_cb(f"[DEV] Response CRC mismatch (got=0x{rec_crc:08X}, exp=0x{cal_crc:08X}) — TOLERATING")

            try:
                self._resp_queue.put_nowait(status)
            except queue.Full:
                self._log_cb("[DEV] Response queue full; dropping status")

    # ---------------------- Send + await ACK via queue ----------------------
    def _send_and_expect_ack(self, packet: bytes, expect_timeout_s: float = 5.0):
        if not self.cmd_sock or not self.reply_sock:
            raise RuntimeError("Sockets not open")

        last_exc = None
        for attempt in range(1, self.retries + 1):
            try:
                # Purge stale statuses so we only accept responses for THIS packet
                try:
                    while True:
                        self._resp_queue.get_nowait()
                except queue.Empty:
                    pass

                # Send packet on command channel
                self.cmd_sock.sendall(packet)

                # Wait for a status from the reader thread (port 2000)
                status = self._resp_queue.get(timeout=expect_timeout_s)
                if status == ETX_OTA_ACK:
                    return
                last_exc = RuntimeError("Device responded NACK")
            except queue.Empty:
                last_exc = TimeoutError("Timeout waiting for device RESPONSE")
            except Exception as e:
                last_exc = e

            if attempt < self.retries:
                time.sleep(0.05)
            else:
                if last_exc:
                    raise last_exc

    # ---------------------- Helpers for manual/step sending ----------------------
    def build_frames_from_file(self, fw_path: str):
        """Return list of dicts: [{'name':str,'packet':bytes,'datasz':int}], plus totals."""
        if not os.path.isfile(fw_path):
            raise RuntimeError("Firmware file not found")
        with open(fw_path, 'rb') as f:
            data = f.read()

        total_size = len(data)
        total_crc = crc32_stm32(data)

        frames = []
        # START
        frames.append({'name': 'START', 'packet': make_cmd_packet(ETX_OTA_CMD_START), 'datasz': 0})
        # HEADER
        frames.append({'name': 'HEADER', 'packet': make_header_packet(total_size, total_crc), 'datasz': 0})
        # DATA chunks
        idx = 0
        while idx < total_size:
            chunk = data[idx: idx + ETX_OTA_DATA_MAX_SIZE]
            pkt = make_data_packet(chunk)
            frames.append({'name': f'DATA[{idx}:{idx+len(chunk)}]', 'packet': pkt, 'datasz': len(chunk)})
            idx += len(chunk)
        # END
        frames.append({'name': 'END', 'packet': make_cmd_packet(ETX_OTA_CMD_END), 'datasz': 0})
        return frames, total_size, total_crc

    # ---------------------- Public API (auto upload) ----------------------
    def upload(self, fw_path: str, progress_cb=lambda a, b: None, log_cb=lambda s: None):
        if not self.cmd_sock or not self.reply_sock:
            raise RuntimeError("TCP not connected")
        if not os.path.isfile(fw_path):
            raise RuntimeError("Firmware file not found")

        with open(fw_path, 'rb') as f:
            data = f.read()

        total_size = len(data)
        total_crc = crc32_stm32(data)
        log_cb(f"Firmware size: {total_size} bytes | CRC32(STM32): 0x{total_crc:08X}")

        log_cb(f"Sending START to {self.ip}:{PORT_CMD_TX} ...")
        self._send_and_expect_ack(make_cmd_packet(ETX_OTA_CMD_START))

        log_cb("Sending HEADER...")
        self._send_and_expect_ack(make_header_packet(total_size, total_crc))

        sent = 0
        idx = 0
        while idx < total_size:
            chunk = data[idx: idx + ETX_OTA_DATA_MAX_SIZE]
            pkt = make_data_packet(chunk)
            self._send_and_expect_ack(pkt, expect_timeout_s=max(self.timeout, 5.0))
            idx += len(chunk)
            sent = idx
            progress_cb(sent, total_size)
            if self.inter_packet_delay > 0:
                time.sleep(self.inter_packet_delay)

        # ... after sending last DATA
        log_cb("Sending END...")
        try:
            self._send_and_expect_ack(make_cmd_packet(ETX_OTA_CMD_END), expect_timeout_s=2.0)
        except Exception as e:
            # Device may have already finalized and/or is rebooting -> benign
            if isinstance(e, OSError) and getattr(e, "winerror", None) == 10054:
                self._log_cb("[RX] device closed connection (finalizing/rebooting)")
            else:
                self._log_cb(f"[Info] END not acknowledged: {e}. Proceeding.")

# ---------------------- GUI ----------------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("STM32 TCP OTA Uploader")
        self.geometry("740x600")

        self.ip_var = tk.StringVar(value=DEFAULT_IP)
        self.delay_var = tk.StringVar(value="0.002")
        self.timeout_var = tk.StringVar(value="1.0")
        self.file_var = tk.StringVar()

        self.tcp: STM32TcpUploader | None = None

        # Step-send state
        self.frames = None            # list of dicts
        self.total_size = 0
        self.total_crc = 0
        self.frame_idx = 0
        self.sent_bytes = 0
        self._step_lock = threading.Lock()  # prevent overlapping sends

        self._build_ui()

    def _build_ui(self):
        frm = ttk.Frame(self, padding=12)
        frm.pack(fill=tk.BOTH, expand=True)

        row = ttk.Frame(frm); row.pack(fill=tk.X, pady=4)
        ttk.Label(row, text="Device IP:").pack(side=tk.LEFT)
        ttk.Entry(row, textvariable=self.ip_var, width=20).pack(side=tk.LEFT, padx=6)
        ttk.Label(row, text=f"(TX:{PORT_CMD_TX} / RX:{PORT_REPLY_RX})").pack(side=tk.LEFT)

        row = ttk.Frame(frm); row.pack(fill=tk.X, pady=4)
        ttk.Label(row, text="Timeout (s):").pack(side=tk.LEFT)
        ttk.Entry(row, textvariable=self.timeout_var, width=8).pack(side=tk.LEFT, padx=6)
        ttk.Label(row, text="Inter-packet delay (s):").pack(side=tk.LEFT, padx=(16,0))
        ttk.Entry(row, textvariable=self.delay_var, width=8).pack(side=tk.LEFT, padx=6)
        ttk.Label(row, text="(0–0.010; 0.002 is safe)").pack(side=tk.LEFT)

        row = ttk.Frame(frm); row.pack(fill=tk.X, pady=4)
        ttk.Label(row, text="Firmware (.bin):").pack(side=tk.LEFT)
        ttk.Entry(row, textvariable=self.file_var, width=60).pack(side=tk.LEFT, padx=6)
        ttk.Button(row, text="Browse...", command=self._browse_file).pack(side=tk.LEFT)

        row = ttk.Frame(frm); row.pack(fill=tk.X, pady=10)
        ttk.Button(row, text="Connect", command=self._on_connect).pack(side=tk.LEFT, padx=6)
        self.start_btn = ttk.Button(row, text="Upload (Auto)", command=self._on_upload, state=tk.DISABLED)
        self.start_btn.pack(side=tk.LEFT, padx=(0, 10))

        # New: Send button (manual/step)
        self.send_btn = ttk.Button(row, text="Send (Step)", command=self._on_send_step, state=tk.DISABLED)
        self.send_btn.pack(side=tk.LEFT)

        ttk.Button(row, text="Quit", command=self.destroy).pack(side=tk.RIGHT)

        self.progress = ttk.Progressbar(frm, mode='determinate', maximum=100)
        self.progress.pack(fill=tk.X, pady=6)

        self.log_text = tk.Text(frm, height=20, wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True, pady=6)
        self._log("Enter device IP (matching your lwIP config), choose your .bin, click Connect. Use 'Send (Step)' to send one frame at a time.")

    def _browse_file(self):
        path = filedialog.askopenfilename(title="Select firmware (.bin)",
                                          filetypes=[("Binary", "*.bin"), ("All files", "*.*")])
        if path:
            self.file_var.set(path)
            # Reset step state when changing file
            self.frames = None
            self.frame_idx = 0
            self.sent_bytes = 0
            self.progress['value'] = 0

    def _log(self, s: str):
        self.log_text.insert(tk.END, s + "\n")
        self.log_text.see(tk.END)
        self.update_idletasks()

    def _on_connect(self):
        ip = self.ip_var.get().strip()
        try:
            timeout = float(self.timeout_var.get().strip())
            delay = float(self.delay_var.get().strip())
        except:
            messagebox.showerror("Error", "Timeout and delay must be numbers.")
            return

        self._log(f"Connecting to {ip} (cmd:{PORT_CMD_TX}, reply:{PORT_REPLY_RX}) ...")
        try:
            self.tcp = STM32TcpUploader(ip, timeout=timeout, inter_packet_delay=delay)
            self.tcp.set_log_cb(self._log)  # <-- give it our logger
            self.tcp.open()
            self._log("Connected.")
            self.start_btn['state'] = tk.NORMAL
            self.send_btn['state'] = tk.NORMAL
        except Exception as e:
            self._log(f"ERROR: {e}")
            messagebox.showerror("Connection failed", str(e))
            try:
                if self.tcp: self.tcp.close()
            except:
                pass
            self.tcp = None
            self.start_btn['state'] = tk.DISABLED
            self.send_btn['state'] = tk.DISABLED

    def _on_upload(self):
        ip = self.ip_var.get().strip()
        fw = self.file_var.get().strip()

        if not ip:
            messagebox.showerror("Error", "Please enter device IP.")
            return
        if not fw or not os.path.isfile(fw):
            messagebox.showerror("Error", "Please choose a valid .bin file.")
            return
        if not self.tcp:
            messagebox.showerror("Error", "Not connected.")
            return

        self.progress['value'] = 0
        self._log(f"Uploading to {ip} ...")

        def worker():
            try:
                def prog(sent, total):
                    pct = int((sent / total) * 100)
                    self.progress['value'] = pct
                    self._log(f"Sent {sent}/{total} bytes ({pct}%)")

                self.tcp.upload(fw, progress_cb=prog, log_cb=self._log)
                self._log("Upload done.")
            except Exception as e:
                self._log(f"ERROR: {e}")
                messagebox.showerror("Upload failed", str(e))

        threading.Thread(target=worker, daemon=True).start()

    # ---------------------- Manual / Step sending ----------------------
    def _ensure_frames_prepared(self):
        """Build frames once per selected file."""
        if self.frames is not None:
            return
        fw = self.file_var.get().strip()
        if not fw or not os.path.isfile(fw):
            raise RuntimeError("Please choose a valid .bin file first.")
        if not self.tcp:
            raise RuntimeError("Not connected.")

        self.frames, self.total_size, self.total_crc = self.tcp.build_frames_from_file(fw)
        self.frame_idx = 0
        self.sent_bytes = 0
        self.progress['value'] = 0
        self._log(f"Prepared {len(self.frames)} frames. Total={self.total_size} bytes, CRC32=0x{crc32_stm32(open(fw,'rb').read()):08X}")

    def _on_send_step(self):
        # Run one frame send in a short-lived thread so UI doesn't block
        def worker():
            with self._step_lock:
                try:
                    self.send_btn['state'] = tk.DISABLED
                    self._ensure_frames_prepared()

                    if self.frame_idx >= len(self.frames):
                        self._log("All frames already sent. Reset or choose another file.")
                        return

                    frame = self.frames[self.frame_idx]
                    name = frame['name']
                    pkt  = frame['packet']
                    dsz  = frame['datasz']

                    self._log(f"[STEP] Sending {name} ...")
                    try:
                        # END frame may NACK if device auto-finalized — tolerate
                        if name == 'END':
                            try:
                                self.tcp._send_and_expect_ack(pkt, expect_timeout_s=2.0)
                            except Exception as e:
                                self._log(f"[Info] END not acknowledged: {e}. Proceeding.")
                        else:
                            self.tcp._send_and_expect_ack(pkt, expect_timeout_s=max(self.tcp.timeout, 5.0))
                    except Exception as e:
                        self._log(f"[STEP ERROR] {e}")
                        messagebox.showerror("Send failed", str(e))
                        return

                    # Update counters/progress for DATA frames
                    if dsz > 0:
                        self.sent_bytes += dsz
                        pct = int((self.sent_bytes / self.total_size) * 100) if self.total_size else 0
                        self.progress['value'] = pct
                        self._log(f"Sent {self.sent_bytes}/{self.total_size} bytes ({pct}%)")

                    self.frame_idx += 1

                    if self.frame_idx >= len(self.frames):
                        self._log("[STEP] All frames sent.")
                except Exception as e:
                    self._log(f"[STEP ERROR] {e}")
                    messagebox.showerror("Step send error", str(e))
                finally:
                    # Re-enable unless we've finished (still okay to click again; it will just say done)
                    self.send_btn['state'] = tk.NORMAL

        threading.Thread(target=worker, daemon=True).start()

    def destroy(self):
        try:
            if self.tcp:
                self.tcp.close()
        except:
            pass
        super().destroy()

if __name__ == "__main__":
    try:
        App().mainloop()
    except KeyboardInterrupt:
        pass
