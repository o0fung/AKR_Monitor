import struct
from dataclasses import dataclass
from typing import Optional

import serial
from rich import print

SERIAL_PORT = '/dev/cu.usbserial-1130'
BAUDRATE = 115200
DATA_SIZE = 66          # Total bytes including checksum
PAYLOAD_SIZE = DATA_SIZE - 1
HEADER = (255, 255)     # Last two bytes of 3-byte rolling header
LENGTH_BYTE_INDEX = 0   # First byte in rolling window carries length (DATA_SIZE)
FLOAT_COUNT = 14
FORMAT_PAYLOAD = '<' + 'f' * FLOAT_COUNT + '9B'  # 14 floats + 9 single bytes
EXPECTED_PAYLOAD_LEN = struct.calcsize(FORMAT_PAYLOAD)  # 65


if EXPECTED_PAYLOAD_LEN != PAYLOAD_SIZE:
    raise ValueError("Format size mismatch")

@dataclass
class Packet:
    count: float
    dt: float
    angle_forward: float
    angle_abduction: float
    acc_x: float
    acc_y: float
    acc_z: float
    gyr_x: float
    gyr_y: float
    gyr_z: float
    current: float
    position: float
    cpm_count: float
    cpm_duration: float
    percentage_battery: int
    _state: int
    _flags: int
    _cpm_df: int
    _cpm_pf: int
    servo_lo: int
    servo_hi: int
    cpm_range_df: int
    cpm_range_pf: int
    # Optional system-info fields (when count==0)
    is_sysinfo: bool = False
    sys_ver: float = 0.0
    sys_is_left: Optional[int] = None
    sys_df_range: Optional[float] = None
    sys_fw_date: Optional[str] = None

    @property
    def state_gait(self): return self._state & 0x7
    @property
    def state_servo(self): return (self._state >> 3) & 0x3
    @property
    def state_battery(self): return (self._state >> 5) & 0x3
    @property
    def calibrate_error(self): return (self._state >> 7) & 0x1

    @property
    def walk_mode(self): return self._flags & 0x3
    @property
    def flag_early_swing(self): return (self._flags >> 2) & 0x1
    @property
    def flag_enable_motor(self): return (self._flags >> 3) & 0x1
    @property
    def flag_enable_buzzer(self): return (self._flags >> 4) & 0x1
    @property
    def cpm_mode(self): return (self._flags >> 5) & 0x1
    @property
    def is_left(self): return (self._flags >> 6) & 0x1

    @property
    def cpm_df_dt(self): return self._cpm_df & 0xF
    @property
    def cpm_df_wait(self): return (self._cpm_df >> 4) & 0xF
    @property
    def cpm_pf_dt(self): return self._cpm_pf & 0xF
    @property
    def cpm_pf_wait(self): return (self._cpm_pf >> 4) & 0xF


def parse_packet(raw: bytes) -> Optional[Packet]:
    """Parse a single 66-byte payload (including checksum) into a Packet.

    This function also recognizes the new system-info frame format described in
    PACKET_INFO.md by checking for ASCII tokens in the first two float slots
    (bytes 0..7 == b"INFO VER").
    """
    if len(raw) != DATA_SIZE:
        return None
    payload, checksum_byte = raw[:-1], raw[-1]
    checksum = sum(payload) & 0xFF
    expected = (~checksum) & 0xFF  # Complement checksum scheme
    if expected != checksum_byte:
        return None

    # Check for system-info token prefix directly in bytes (more robust than count==0)
    is_sysinfo = False
    try:
        if payload[0:8] == b"INFO VER":
            is_sysinfo = True
    except Exception:
        is_sysinfo = False

    values = struct.unpack(FORMAT_PAYLOAD, payload)
    pkt = Packet(*values)

    # Interpret system-info special payload
    try:
        if is_sysinfo or int(getattr(pkt, 'count', 1)) == 0:
            pkt.is_sysinfo = True
            f = list(values[:FLOAT_COUNT])
            # Per PACKET_INFO.md (system-info slots):
            # 0: "INFO" (packed ASCII), 1: " VER", 2: firmware version,
            # 3: "CFG ", 4: CONFIG_VERSION (ASCII packed), 5: "DATE",
            # 6..9: date ASCII across 4 floats, 10: "TAG", 11: tag, 12: side (1.0 L / 0.0 R), 13: DF range
            try:
                pkt.sys_ver = float(f[2])
            except Exception:
                pkt.sys_ver = 0.0
            try:
                pkt.sys_is_left = 1 if float(f[12]) >= 0.5 else 0
            except Exception:
                pkt.sys_is_left = None
            try:
                pkt.sys_df_range = float(f[13])
            except Exception:
                pkt.sys_df_range = None
            # f[6..9] contain FW_DATE as packed ASCII, 16 chars max
            date_bytes = b''
            for i in range(6, 10):
                try:
                    date_bytes += struct.pack('<f', f[i])
                except Exception:
                    pass
            pkt.sys_fw_date = ''.join(chr(b) for b in date_bytes if 32 <= b <= 126).strip() or None
    except Exception:
        pass
    return pkt


def format_packet(p: Packet, last_count: float) -> str:
    if getattr(p, 'is_sysinfo', False):
        side = 'L' if getattr(p, 'sys_is_left', 1) else 'R'
        rng = getattr(p, 'sys_df_range', None)
        date = getattr(p, 'sys_fw_date', '') or ''
        ver = getattr(p, 'sys_ver', 0)
        try:
            ver_txt = f"{ver:.0f}"
        except Exception:
            ver_txt = str(ver)
        return f"SYSINFO v{ver_txt} {side} DF{int(rng) if rng else ''} {date}"
    n = p.count - last_count
    return (
        f"{p.count:7.0f}: {p.dt:6.0f} | {p.angle_forward:5.1f} {p.angle_abduction:5.1f} | "
        f"{p.acc_x:6.2f} {p.acc_y:6.2f} {p.acc_z:6.2f} | "
        f"{p.gyr_x:6.2f} {p.gyr_y:6.2f} {p.gyr_z:6.2f} | "
        f"{p.current:2.1f}A state:{p.state_battery:2.0f} {p.percentage_battery:2.0f}%  "
        f"{p.state_gait:2.0f} {p.state_servo:2.0f} {p.walk_mode:2.0f} {p.position:5.1f} deg   "
        f"n={p.cpm_count:<5.0f} {p.cpm_duration:5.2f}s  "
        f"{'E' if p.flag_early_swing else 'e'} "
        f"{'M' if p.flag_enable_motor else 'm'} "
        f"{'B' if p.flag_enable_buzzer else 'b'} "
        f"{'T' if p.cpm_mode else 't'}  "
        f"{'L' if p.is_left else 'R'} "
        f"{' error! ' if p.calibrate_error else ''}"
        f"{p.servo_lo:5.0f} ({p.cpm_range_df:3.0f}%) {p.servo_hi:5.0f} ({p.cpm_range_pf:3.0f}%) | "
        f"{p.cpm_df_dt:2.0f} {p.cpm_df_wait:2.0f} {p.cpm_pf_dt:2.0f} {p.cpm_pf_wait:2.0f}"
    )


def run():
    last_count = 0.0
    header_window = [0, 0, 0]
    try:
        with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
            print(f"Connected to {ser.port} at {ser.baudrate} bps...")

            while ser.is_open:
                if ser.in_waiting:
                    b = ser.read(1)
                    if not b:
                        continue

                    header_window[2] = header_window[1]
                    header_window[1] = header_window[0]
                    header_window[0] = b[0]

                    if header_window == [DATA_SIZE, 255, 255]:
                        packet_bytes = ser.read(DATA_SIZE)
                        if len(packet_bytes) != DATA_SIZE:
                            continue

                        pkt = parse_packet(packet_bytes)
                        if not pkt:
                            continue
                        
                        print(format_packet(pkt, last_count))
                        last_count = pkt.count

    except KeyboardInterrupt:
        pass

    finally:
        print("End")


if __name__ == "__main__":
    run()
