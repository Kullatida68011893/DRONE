
import time
import csv
import random
from dataclasses import dataclass
from typing import Optional, Iterable

# If using UART:
# pip install pyserial
try:
    import serial
except ImportError:
    serial = None


# ----------------------------
# 1) Data structure ("contract")
# ----------------------------
@dataclass
class UwbMeas:
    t_ms: int
    x: float
    y: float
    z: float
    valid: bool
    raw: str = ""


# ----------------------------
# 2) Robust parser
# ----------------------------
class UwbParser:
    """
    Expects lines like:
      POS,1.23,4.56,0.89
    You can adjust this once you know your real DWM1001 format.
    """

    def parse_line(self, line: str) -> UwbMeas:
        t_ms = int(time.time() * 1000)
        s = line.strip()
        m = UwbMeas(t_ms=t_ms, x=0.0, y=0.0, z=0.0, valid=False, raw=s)

        # Basic validation
        if not s:
            return m
        if not s.startswith("POS,"):
            return m

        parts = s.split(",")
        if len(parts) < 4:
            return m

        try:
            x = float(parts[1].strip())
            y = float(parts[2].strip())
            z = float(parts[3].strip())
        except ValueError:
            return m

        m.x, m.y, m.z = x, y, z
        m.valid = True
        return m


# ----------------------------
# 3) Logger (CSV)
# ----------------------------
class CsvLogger:
    def __init__(self, filename: str):
        self.filename = filename
        self.f = None
        self.writer = None

    def open(self):
        self.f = open(self.filename, "w", newline="")
        self.writer = csv.writer(self.f)
        self.writer.writerow(["t_ms", "x", "y", "z", "valid", "raw"])
        self.f.flush()

    def log(self, m: UwbMeas):
        if self.writer is None:
            raise RuntimeError("Logger not opened. Call logger.open() first.")
        self.writer.writerow([m.t_ms, f"{m.x:.6f}", f"{m.y:.6f}", f"{m.z:.6f}", int(m.valid), m.raw])
        # flush often for safety (you can reduce flush rate later)
        self.f.flush()

    def close(self):
        if self.f:
            self.f.close()
            self.f = None
            self.writer = None


# ----------------------------
# 4) Input sources (Simulator or UART)
#    Both yield "lines"
# ----------------------------
class UwbSimulator:
    """
    Generates POS,x,y,z lines with noise + occasional spikes/bad lines
    """
    def __init__(self, rate_hz: float = 20.0):
        self.dt = 1.0 / rate_hz
        self.t0 = time.time()

    def lines(self) -> Iterable[str]:
        while True:
            t = time.time() - self.t0
            # Base motion
            x = 1.0 + 0.2 * t
            y = 2.0 + 0.3 * (random.random() - 0.5)
            z = 0.8

            # Noise
            x += random.gauss(0, 0.05)
            y += random.gauss(0, 0.05)
            z += random.gauss(0, 0.02)

            # Occasional spike (multipath-like)
            if int(t) % 7 == 0 and (t - int(t)) < 0.05:
                x += 1.5

            # Occasional dropout / corrupt line
            if int(t) % 11 == 0 and (t - int(t)) < 0.05:
                yield "BADLINE"
            else:
                yield f"POS,{x:.4f},{y:.4f},{z:.4f}"

            time.sleep(self.dt)


class UartReader:
    """
    Reads newline-delimited lines from a serial port.
    """
    def __init__(self, port: str, baud: int = 115200, timeout: float = 1.0):
        if serial is None:
            raise RuntimeError("pyserial not installed. Run: pip install pyserial")
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)

    def lines(self) -> Iterable[str]:
        if self.ser is None:
            raise RuntimeError("UART not opened. Call reader.open() first.")

        while True:
            raw = self.ser.readline()  # reads until \n or timeout
            if not raw:
                continue
            try:
                line = raw.decode("utf-8", errors="ignore").strip()
            except Exception:
                continue
            if line:
                yield line

    def close(self):
        if self.ser:
            self.ser.close()
            self.ser = None


# ----------------------------
# 5) Main pipeline
# ----------------------------
def run_pipeline(use_sim: bool = True, csv_name: str = "uwb_log.csv", uart_port: str = "/dev/ttyUSB0"):
    parser = UwbParser()
    logger = CsvLogger(csv_name)
    logger.open()

    if use_sim:
        src = UwbSimulator(rate_hz=20.0)
        line_stream = src.lines()
        print("[INFO] Running in SIM mode.")
    else:
        reader = UartReader(port=uart_port, baud=115200)
        reader.open()
        line_stream = reader.lines()
        print(f"[INFO] Running in UART mode on {uart_port} @115200.")

    try:
        for line in line_stream:
            meas = parser.parse_line(line)
            logger.log(meas)

            # Optional: live print only valid measurements
            if meas.valid:
                print(f"{meas.t_ms}, x={meas.x:.3f}, y={meas.y:.3f}, z={meas.z:.3f}")
            else:
                print(f"{meas.t_ms}, INVALID: {meas.raw}")

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    finally:
        logger.close()
        if not use_sim:
            reader.close()


if __name__ == "__main__":
    # ✅ Change this flag to swap input source
    USE_SIM = True

    # If USE_SIM = False, set your serial port:
    # Windows: "COM5"
    # macOS: "/dev/tty.usbserial-XXXX" or "/dev/tty.SLAB_USBtoUART"
    # Linux: "/dev/ttyUSB0"
    UART_PORT = "/dev/ttyUSB0"

    run_pipeline(use_sim=USE_SIM, csv_name="uwb_log.csv", uart_port=UART_PORT)
    