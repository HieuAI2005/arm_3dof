# import serial
# import time

# def send_message(msg, port='/dev/ttyACM0'):
#     try:
#         ser = serial.Serial(port, 9600, timeout=1)
#         time.sleep(2)
#         msg = ','.join(map(str, msg))
#         ser.write(msg.encode())
#         print("Done:", msg)
#         ser.close()
#     except Exception as e:
#         print(e)
import time, serial, threading, re

DEFAULT_PORT = '/dev/ttyACM0'
BAUD = 9600

class SerialMgr:
    def __init__(self, port=DEFAULT_PORT, baud=BAUD):
        self.port = port
        self.baud = baud
        self._ser = None
        self._lock = threading.Lock()

    def open(self):
        if self._ser and self._ser.is_open:
            return self._ser
        ser = serial.Serial(
            self.port, self.baud,
            timeout=1, write_timeout=1,
            rtscts=False, dsrdtr=False
        )
        # cố gắng vô hiệu reset do DTR/RTS
        try:
            ser.setDTR(False)
            ser.setRTS(False)
        except Exception:
            pass
        # chờ ổn định serial + board
        time.sleep(2.2)
        # dọn input rác
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            pass
        self._ser = ser
        return self._ser

    def write_line(self, line):
        with self._lock:
            ser = self.open()
            data = (line.strip() + '\n').encode()
            ser.write(data)
            ser.flush()

    def read_lines_for(self, duration_s=1.0):
        """Đọc các dòng trong khoảng thời gian duration_s."""
        out = []
        end = time.time() + max(0.05, duration_s)
        with self._lock:
            ser = self.open()
            while time.time() < end:
                try:
                    line = ser.readline().decode(errors='ignore')
                except Exception:
                    break
                if line:
                    line = line.strip('\r\n')
                    if line:
                        out.append(line)
                else:
                    time.sleep(0.01)
        return out

    def read_until(self, patterns, timeout_s=6.0):
        """Đọc tới khi thấy một trong các pattern (regex hoặc chuỗi con) hoặc hết timeout."""
        buf = []
        end = time.time() + timeout_s
        patt = [re.compile(p) if isinstance(p, str) else p for p in patterns]
        with self._lock:
            ser = self.open()
            while time.time() < end:
                try:
                    line = ser.readline().decode(errors='ignore')
                except Exception:
                    break
                if not line:
                    time.sleep(0.01)
                    continue
                line = line.strip('\r\n')
                if line:
                    buf.append(line)
                    txt = line
                    for rgx in patt:
                        if rgx.search(txt):
                            return buf
        return buf

_mgr = SerialMgr()

def init_serial(port=DEFAULT_PORT, baud=BAUD):
    global _mgr
    _mgr = SerialMgr(port, baud)
    _mgr.open()
    # đọc chào mừng ban đầu
    boot = _mgr.read_lines_for(2.0)
    if boot:
        print("[ARDUINO_BOOT]", *boot, sep="\n[RX] ")

def send_pick(angles):
    """Gửi 'P b,s,e' và đọc log tới khi thấy OK:PICK_MS hoặc hết timeout."""
    t1, t2, t3 = [int(round(a)) for a in angles]
    cmd = f"P {t1},{t2},{t3}"
    print(f"[TX] {cmd}")
    _mgr.write_line(cmd)
    # đợi các sự kiện PICK và OK:PICK_MS
    lines = _mgr.read_until([r"OK:PICK_MS=\d+"], timeout_s=12.0)
    for ln in lines:
        print("[RX]", ln)
    return lines

def send_place(idx):
    """Gửi 'D idx' và đọc log tới khi thấy OK:PLACE_MS hoặc ít nhất vài dòng hồi đáp."""
    cmd = f"D {int(idx)}"
    print(f"[TX] {cmd}")
    _mgr.write_line(cmd)
    # chờ log place khoảng 8s (tuỳ thời gian place)
    lines = _mgr.read_until([r"OK:PLACE_MS=\d+"], timeout_s=10.0)
    for ln in lines:
        print("[RX]", ln)
    return lines

def send_cancel():
    """Tuỳ chọn: nếu bạn có lệnh Hủy trên Arduino (ví dụ 'C'), thêm vào firmware để thoát HOLDING."""
    # _mgr.write_line("C")
    # print("[TX] C")
    pass