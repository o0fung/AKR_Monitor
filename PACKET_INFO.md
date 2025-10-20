## AKR Bluetooth Packet Protocol

This document describes the byte-level protocol implemented in `src/Bluetooth.cpp` / `Bluetooth.h` for the Ankle Robot (AKR). It enables you to write host-side code (Python, C#, etc.) that packs/unpacks packets exchanged with the device over a Bluetooth serial link.

- Serial: 115200 baud, 8 data bits, no parity, 1 stop bit (8N1)
- Endianness for floats: little-endian IEEE‑754 32-bit (AVR “double” is 4 bytes)
- All packets use the same framing: two header bytes, one length byte, then `length` payload bytes


## Framing

Every packet on the wire has this structure:

| Field   | Size (bytes) | Value/Range       | Notes |
|---------|---------------|-------------------|-------|
| Header  | 2             | 0xFF 0xFF         | Constant sync bytes |
| Length  | 1             | 10 or 66          | Number of following payload bytes including checksum |
| Payload | L             | bytes             | Last byte of payload is checksum |

Checksum definition:

| Name     | Definition |
|----------|------------|
| Sum      | `sum(P[0..L-2]) & 0xFF` |
| Checksum | `(~Sum) & 0xFF` equals `P[L-1]` |


## Device → Host: Telemetry Packet (L = 66)

When running normally, the device streams telemetry packets with payload length `L = 66`. The payload is:

Telemetry payload layout (offsets are from start of payload, i.e., immediately after Length):

| Offset | Size | Type   | Name                               | Units     | Notes |
|--------|------|--------|------------------------------------|-----------|-------|
| 0      | 4    | float  | Frame index                        | count     | `stat.count` |
| 4      | 4    | float  | Frame duration                     | µs        | `stat.delta_t` |
| 8      | 4    | float  | Roll angle (forward +)             | deg       | `angle_tilt_forward[0]` |
| 12     | 4    | float  | Pitch angle (abduction +)          | deg       | `angle_abduction[0]` |
| 16     | 4    | float  | Leg accel X                        | m/s²      | `processed[2][0]` |
| 20     | 4    | float  | Leg accel Y                        | m/s²      | `processed[3][0]` |
| 24     | 4    | float  | Leg accel Z                        | m/s²      | `processed[4][0]` |
| 28     | 4    | float  | Leg gyro X                         | deg/s     | `processed[5][0]` |
| 32     | 4    | float  | Leg gyro Y                         | deg/s     | `processed[6][0]` |
| 36     | 4    | float  | Leg gyro Z                         | deg/s     | `processed[7][0]` |
| 40     | 4    | float  | Servo current (EMA)                | A         | `stat.current_ema` |
| 44     | 4    | float  | Servo current position             | unitless  | `mot.get_cur_pos()` (device-specific scale) |
| 48     | 4    | float  | CPM repetition count               | count     | float-encoded integer |
| 52     | 4    | float  | CPM remaining duration             | s         | |
| 56     | 1    | uint8  | Battery level                      | %         | 0–100 |
| 57     | 1    | uint8  | TX GET flags (rBBSSGGG)            | bits      | See bit table below |
| 58     | 1    | uint8  | TX SET flags (XLCZMEWW)            | bits      | See bit table below |
| 59     | 1    | uint8  | CPM DF timing                      | nibble/nd | low=dt(0..15), high=wait(0..15) |
| 60     | 1    | uint8  | CPM PF timing                      | nibble/nd | low=dt(0..15), high=wait(0..15) |
| 61     | 1    | uint8  | DF target (LOCK)                   | 0..255    | |
| 62     | 1    | uint8  | PF target (JUMP)                   | 0..255    | |
| 63     | 1    | uint8  | CPM range DF                       | %         | 0..100 |
| 64     | 1    | uint8  | CPM range PF                       | %         | 0..100 |
| 65     | 1    | uint8  | Checksum                           | —         | `~sum(payload[0..64])` |

Flag byte rBBSSGGG (offset 57):

| Bit | Name          | Meaning |
|-----|---------------|---------|
| 7   | r             | Calibration error (1=error)
| 6-5 | BB            | Battery state (0..3)
| 4-3 | SS            | Servo state (0..3)
| 2-0 | GGG           | Gait state (0..7)

Flag byte XLCZMEWW (offset 58):

| Bit | Name | Meaning |
|-----|------|---------|
| 7   | X    | DF range (1=DF30, 0=DF15)
| 6   | L    | Side (1=LEFT, 0=RIGHT)
| 5   | C    | CPM enable
| 4   | Z    | Buzzer enable
| 3   | M    | Motor enable
| 2   | E    | Early swing enable
| 1-0 | WW   | Gait mode (0..3)

Notes
- “dt” and “wait” are 4-bit values used by firmware timing logic for DF/PF movement segments.
- Exact enumerations for “battery/servo/gait state” come from firmware modules; treat them as small integers unless you mirror the enums.


### Device → Host: System-Info Frame (also L = 66)

On startup (frame count == 0) or when explicitly requested (cmd 32), the device sends a “system-info” payload instead of telemetry. It still uses 14 float slots (56 bytes), then the same 9 bytes as telemetry, then checksum, but the floats carry tagged meta in this order:

| Slot | Type  | Content                         | Example |
|------|-------|---------------------------------|---------|
| 0    | float | ASCII "INFO" (packed)           | "INFO" |
| 1    | float | ASCII " VER" (packed)           | " VER" |
| 2    | float | Firmware version                | 17.1 |
| 3    | float | ASCII "CFG " (packed)           | "CFG " |
| 4    | float | ASCII CONFIG_VERSION            | "v171" |
| 5    | float | ASCII "DATE" (packed)           | "DATE" |
| 6–9  | float | Firmware date ASCII (packed)    | "2025-10-16" across 4 floats |
| 10   | float | ASCII "TAG" (packed)            | "TAG" |
| 11   | float | ASCII tag (packed)              | " L30" or " R15" |
| 12   | float | Side as number                  | 1.0=LEFT, 0.0=RIGHT |
| 13   | float | DF range                        | 30.0 or 15.0 |

Bytes 56..64: battery %, flags, CPM timing/targets/ranges (same as telemetry). Byte 65: checksum.

Host parsing tip: If slots 0..1 decode to ASCII tokens "INFO" and " VER" (each packed into a float), it’s a system-info frame; otherwise, parse as normal telemetry.


## Host → Device: Parameter/Command Packet (L = 10)

Host sends control packets with payload length `L = 10`. The payload bytes are:

Parameter payload layout:

| Byte | Name                   | Type  | Bits           | Meaning |
|------|------------------------|-------|----------------|---------|
| 0    | RX SET flags           | uint8 | C Z M E WW.    | Same bit layout as TX SET except X is reserved in RX |
| 1    | CPM DF timing          | uint8 | wait:4, dt:4   | lower=dt(0..15), upper=wait(0..15) |
| 2    | CPM PF timing          | uint8 | wait:4, dt:4   | lower=dt(0..15), upper=wait(0..15) |
| 3    | DF target (LOCK)       | uint8 | —              | 0..255 |
| 4    | PF target (JUMP)       | uint8 | —              | 0..255 |
| 5    | CPM range DF (%)       | uint8 | —              | 0..100 |
| 6    | CPM range PF (%)       | uint8 | —              | 0..100 |
| 7    | CPM duration (minutes) | uint8 | —              | See notes below |
| 8    | Command byte           | uint8 | id:7, arm:1    | `id=(byte>>1)&0x7F`, arm bit0=1 to execute |
| 9    | Checksum               | uint8 | —              | Ones’ complement of sum bytes 0..8 |

RX SET flags (byte 0):

| Bits | Name | Meaning |
|------|------|---------|
| 1-0  | WW   | Gait mode (0..3) |
| 2    | E    | Early swing enable |
| 3    | M    | Motor enable |
| 4    | Z    | Buzzer enable |
| 5    | C    | CPM enable (applied only when motor disabled) |
| 6    | —    | Ignored (use commands 48/49 to set side) |
| 7    | —    | Ignored |

Side control: The firmware no longer reads a “side” bit from RX flags. Use command 48 (RIGHT) or 49 (LEFT) to change side at runtime.

Command byte (byte 8):

| ID  | Purpose                               |
|-----|---------------------------------------|
| 0   | RF reset/unpair (simulate long press) |
| 1   | RF pair (simulate short press)        |
| 2   | Calibrate servo position feedback     |
| 6   | Calibrate servo PWM                   |
| 8   | Trigger servo move to DF              |
| 16  | Trigger servo move to PF              |
| 32  | Force next packet = system-info; reset frame count |
| 48  | Set robot side RIGHT                  |
| 49  | Set robot side LEFT                   |
| 64  | Factory reset                         |

Notes on duration (byte 7):
- If CPM enabled and value > 0 and motor disabled: starts/updates CPM to `minutes * 60` seconds.
- If CPM disabled and value == 0: stops CPM and clears counters.
- Otherwise: used for pause/resume semantics; see firmware comments.


## Reference: Constants from Firmware

- Header byte: 0xFF
- Telemetry payload length (DATA_SIZE): 66
- Param payload length (PARAM_SIZE): 10
- Firmware version/date: see `setup.h` (e.g., `VERSION`, `FW_DATE`)


## Python examples

These examples use the `struct` module and assume data is read from a serial port that yields bytes.
```python
import struct

HEADER = b"\xFF\xFF"
DATA_SIZE = 66

def find_frame(buf: bytearray):
	# simple resync: find FF FF and check length
	i = buf.find(HEADER)
	if i < 0 or len(buf) < i + 3:
		return None
	L = buf[i+2]
	if len(buf) < i + 3 + L:
		return None
	frame = bytes(buf[i:i+3+L])
	del buf[:i+3+L]
	return frame

def verify_checksum(payload: bytes) -> bool:
	s = sum(payload[:-1]) & 0xFF
	return payload[-1] == ((~s) & 0xFF)

def parse_telemetry(payload: bytes):
	# payload is 66 bytes
	floats = struct.unpack('<14f', payload[0:56])
	b = payload[56:65]
	checksum = payload[65]
	if not verify_checksum(payload):
		raise ValueError('Bad checksum')
	return {
		'frame_index': floats[0],
		'delta_t_us': floats[1],
		'angles': {
			'roll_forward_deg': floats[2],
			'pitch_abduction_deg': floats[3],
		},
		'accel_mps2': floats[4:7],
		'gyro_dps': floats[7:10],
		'servo_current_A': floats[10],
		'servo_pos': floats[11],
		'cpm_count': floats[12],
		'cpm_remaining_s': floats[13],
		'battery_percent': b[0],
		'tx_get_flags': b[1],
		'tx_set_flags': b[2],
		'cpm_df': b[3],
		'cpm_pf': b[4],
		'df_target': b[5],
		'pf_target': b[6],
		'cpm_range_df_pct': b[7],
		'cpm_range_pf_pct': b[8],
	}

# Example usage with a serial loop (pyserial):
# import serial
# ser = serial.Serial('/dev/tty.HC-05-DevB', 115200, timeout=0.1)
# buf = bytearray()
# while True:
#     buf.extend(ser.read(256))
#     frame = find_frame(buf)
#     if not frame:
#         continue
#     header, L, payload = frame[:2], frame[2], frame[3:]
#     if L == DATA_SIZE:
#         data = parse_telemetry(payload)
#         print(data)
```

### Build and send a parameter packet (host → device)

Note: Side selection is performed using commands 48 (RIGHT) and 49 (LEFT). The firmware ignores any “side” bit in RX flags.

```python
import struct

HEADER = b"\xFF\xFF"
PARAM_SIZE = 10

def checksum(payload: bytes) -> int:
	return (~(sum(payload) & 0xFF)) & 0xFF

def build_params(
	gait_mode: int,
	early_swing: bool,
	motor_enable: bool,
	buzzer_enable: bool,
	cpm_enable: bool,
	is_left: bool,
	df_dt: int, df_wait: int,
	pf_dt: int, pf_wait: int,
	df_target: int,
	pf_target: int,
	cpm_range_df: int,
	cpm_range_pf: int,
	cpm_duration_min: int,
	cmd: int | None = None,
):
	rx_set = (
		(gait_mode & 0x3)
		| ((1 if early_swing else 0) << 2)
		| ((1 if motor_enable else 0) << 3)
		| ((1 if buzzer_enable else 0) << 4)
		| ((1 if cpm_enable else 0) << 5)
		| ((1 if is_left else 0) << 6)
	)
	cpm_df = (df_dt & 0xF) | ((df_wait & 0xF) << 4)
	cpm_pf = (pf_dt & 0xF) | ((pf_wait & 0xF) << 4)
	cmd_byte = 0 if cmd is None else ((cmd & 0x7F) << 1) | 0x01
	payload = bytes([
		rx_set,
		cpm_df,
		cpm_pf,
		df_target & 0xFF,
		pf_target & 0xFF,
		cpm_range_df & 0xFF,
		cpm_range_pf & 0xFF,
		cpm_duration_min & 0xFF,
		cmd_byte,
		0,  # placeholder for checksum
	])
	payload = payload[:-1] + bytes([checksum(payload[:-1])])
	return HEADER + bytes([PARAM_SIZE]) + payload

# Example:
# pkt = build_params(
#     gait_mode=0, early_swing=False, motor_enable=False, buzzer_enable=False,
#     cpm_enable=True, is_left=True,
#     df_dt=3, df_wait=2, pf_dt=3, pf_wait=2,
#     df_target=170, pf_target=60,
#     cpm_range_df=50, cpm_range_pf=50,
#     cpm_duration_min=10,
#     cmd=32,  # force system-info frame next
# )
# ser.write(pkt)
```


## C# examples

### Build and send a parameter packet

Note: Side selection is performed using commands 48 (RIGHT) and 49 (LEFT). The firmware ignores any “side” bit in RX flags.

```csharp
byte[] BuildParams(
	int gaitMode, bool earlySwing, bool motorEnable, bool buzzerEnable,
	bool cpmEnable, bool isLeft,
	int dfDt, int dfWait, int pfDt, int pfWait,
	int dfTarget, int pfTarget,
	int cpmRangeDf, int cpmRangePf,
	int cpmDurationMin,
	int? cmd = null)
{
	const byte HEADER = 0xFF;
	const byte PARAM_SIZE = 10;
	byte rxSet = (byte)((gaitMode & 0x3)
		| ((earlySwing ? 1 : 0) << 2)
		| ((motorEnable ? 1 : 0) << 3)
		| ((buzzerEnable ? 1 : 0) << 4)
		| ((cpmEnable ? 1 : 0) << 5)
		| ((isLeft ? 1 : 0) << 6));
	byte cpmDf = (byte)((dfDt & 0xF) | ((dfWait & 0xF) << 4));
	byte cpmPf = (byte)((pfDt & 0xF) | ((pfWait & 0xF) << 4));
	byte cmdByte = (byte)(cmd.HasValue ? ((cmd.Value & 0x7F) << 1) | 0x01 : 0x00);
	Span<byte> payload = stackalloc byte[PARAM_SIZE];
	payload[0] = rxSet;
	payload[1] = cpmDf;
	payload[2] = cpmPf;
	payload[3] = (byte)dfTarget;
	payload[4] = (byte)pfTarget;
	payload[5] = (byte)cpmRangeDf;
	payload[6] = (byte)cpmRangePf;
	payload[7] = (byte)cpmDurationMin;
	payload[8] = cmdByte;
	// checksum
	int sum = 0; for (int i = 0; i < 9; i++) sum = (sum + payload[i]) & 0xFF;
	payload[9] = (byte)(~sum & 0xFF);
	return new byte[] { HEADER, HEADER, PARAM_SIZE }
		.Concat(payload.ToArray())
		.ToArray();
}
```

### Unpack telemetry floats and flags

```csharp
// Assumes you already have the 66-byte payload in `payload` and checksum verified
float[] vals = new float[14];
for (int i = 0; i < 14; i++)
{
	// Little-endian 4-byte float
	vals[i] = BitConverter.ToSingle(payload, i * 4);
}
byte batteryPercent = payload[56];
byte txGet = payload[57];
byte txSet = payload[58];
byte cpmDf = payload[59];
byte cpmPf = payload[60];
byte dfTarget = payload[61];
byte pfTarget = payload[62];
byte cpmRangeDf = payload[63];
byte cpmRangePf = payload[64];
byte checksum = payload[65];

bool calError = (txGet & 0x80) != 0;
int batteryState = (txGet >> 5) & 0x03;
int servoState = (txGet >> 3) & 0x03;
int gaitState = txGet & 0x07;

bool df30 = (txSet & 0x80) != 0;
bool isLeft = (txSet & 0x40) != 0;
bool cpmEnable = (txSet & 0x20) != 0;
bool buzzerEnable = (txSet & 0x10) != 0;
bool motorEnable = (txSet & 0x08) != 0;
bool earlySwing = (txSet & 0x04) != 0;
int gaitMode = txSet & 0x03;
```


## Tips and edge cases

- Resynchronization: If your reader loses sync, scan for the two 0xFF headers, read the length `L`, and then read exactly `L` bytes as the payload; verify checksum before parsing.
- System-info vs telemetry: Detect system-info by checking if float slots 0..1 decode to ASCII tokens "INFO" and " VER" (each packed in a float). Otherwise, parse as normal telemetry.
- Units: Angles are degrees; accelerations are m/s²; angular velocities are deg/s; current is Amps; time fields as noted above.
- Side changes: Use commands 48 (RIGHT) and 49 (LEFT) to change side; the device will update defaults accordingly.
- DF15 vs DF30: Reflected in TX SET flag bit7 (df30), and in system-info slot 13 (15.0 or 30.0).


## Where this comes from (source references)

- `src/Bluetooth.h`: constants (`DATA_SIZE=66`, `PARAM_SIZE=10`), framing, unions, and flags
- `src/Bluetooth.cpp`: packing order for telemetry fields, flag definitions, parameter handling, and commands
- `src/setup.h`: `VERSION`, `FW_DATE`, side defaults, and other runtime defaults

If any of the above code changes, update this document accordingly.
