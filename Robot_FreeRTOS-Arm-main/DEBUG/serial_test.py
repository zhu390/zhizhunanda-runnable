import serial, time, threading, sys, datetime

PORT = "COM7"
BAUD = 115200
LOG = r"e:\Renesas\RA6M5-Robot\Robot_FreeRTOS - Arm\DEBUG\debug.txt"

stop = threading.Event()
logf = open(LOG, "w", encoding="utf-8", newline="")
lock = threading.Lock()

def ts():
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

def writeln(s):
    with lock:
        logf.write(s)
        logf.flush()
        sys.stdout.write(s)
        sys.stdout.flush()

def reader(ser):
    buf = b""
    while not stop.is_set():
        try:
            data = ser.read(256)
        except Exception as e:
            writeln(f"[reader error] {e}\n")
            break
        if data:
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                txt = line.decode("utf-8", errors="replace").rstrip("\r")
                writeln(f"[{ts()}] {txt}\n")

def send(ser, cmd):
    writeln(f"[{ts()}] >>> SEND: {cmd}\n")
    ser.write((cmd + "\r\n").encode())
    ser.flush()

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
    except Exception as e:
        writeln(f"[FATAL] cannot open {PORT}: {e}\n")
        return
    writeln(f"[{ts()}] ===== opened {PORT} @ {BAUD} =====\n")
    t = threading.Thread(target=reader, args=(ser,), daemon=True)
    t.start()

    time.sleep(1.0)

    # 验证静态缓冲区修复：以前 auto 20/30 必崩，修复后应全部通过
    plan = [
        ("soft_reset",  8),
        ("auto 20 0 0", 10),   # path_size=21, 之前在 point14 崩
        ("soft_reset",  8),
        ("auto 30 0 0", 12),   # path_size=31, 之前在 point9  崩
        ("soft_reset",  8),
        ("auto 20 -20 0", 12), # 之前在 point10 崩（带 Y 分量）
    ]

    for cmd, wait in plan:
        send(ser, cmd)
        time.sleep(wait)

    writeln(f"[{ts()}] ===== test sequence complete =====\n")
    time.sleep(1.0)
    stop.set()
    time.sleep(0.3)
    ser.close()
    logf.close()

if __name__ == "__main__":
    main()
