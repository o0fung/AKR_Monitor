# AKR Monitor — Easy Start 入门指南

Simple app to view sensor data from your AKR device in real time, save CSV logs, and send settings back to the device.

这是一个简单易用的图形界面应用，用来实时查看 AKR 设备的传感器数据、保存 CSV 日志，并向设备发送设置参数。

## What you need 需要准备什么

- A computer running macOS (recommended) or Windows 10/11. Internet connection required for install.

- 一台装有 macOS（推荐）或 Windows 10/11 的电脑。安装时需要可用的网络连接。

- Python 3.10 or newer. If you’re not sure, we’ll show how to install it below.

- Python 3.10 或更高版本。如果不确定是否已安装，下面有安装方法。

- A USB data cable to connect your device. On macOS, the port will look like /dev/cu.usbserial-xxxx or /dev/tty.usbmodemxxxx.

- 一根 USB 数据线连接设备。macOS 上串口名称通常类似 /dev/cu.usbserial-xxxx 或 /dev/tty.usbmodemxxxx。

Tip: Some USB adapters (CH340/CP210x/FTDI) may need drivers. If the port doesn’t appear, install the vendor driver for your adapter and then try again.

提示：某些 USB 转串口芯片（如 CH340/CP210x/FTDI）可能需要驱动。如果找不到串口，请安装对应芯片的驱动后重试。

## Recommended: Use a virtual environment（最推荐：使用虚拟环境）

This is the simplest and most reliable way for new users. It keeps everything self‑contained and avoids PATH problems. After setup, you’ll start the app with a single command: akr.

这是对新手最简单、最稳妥的方法。它把环境与系统隔离，避免 PATH 问题。配置完成后，只需输入 akr 即可启动应用。

### Windows (PowerShell / CMD)

1) Install Python 3.10+ from python.org. During setup, check “Add python.exe to PATH”.

2) In the AKR_Monitor folder, create and activate a virtual environment:

```powershell
py -m venv .venv
.venv\Scripts\activate
```

3) Upgrade pip and install the app into this venv:

```powershell
python -m pip install --upgrade pip
pip install "git+https://github.com/o0fung/AKR_Monitor.git#egg=akr-monitor"
```

4) Start the app (venv must be active):

```powershell
akr
```

Next time you want to use it, just activate and run:

```powershell
.venv\Scripts\activate
akr
```

### macOS (Terminal)

1) Install Python 3.10+ (e.g., from python.org). Reopen Terminal after installing.

2) In the AKR_Monitor folder, create and activate a virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
```

3) Upgrade pip and install the app into this venv:

```bash
python -m pip install --upgrade pip
pip install "git+https://github.com/o0fung/AKR_Monitor.git#egg=akr-monitor"
```

4) Start the app (venv must be active):

```bash
akr
```

Next time you want to use it, just activate and run:

```bash
source .venv/bin/activate
akr
```

If akr doesn’t start for any reason, you can always use the module form inside the same venv:

如果 akr 无法启动，也可在同一虚拟环境中使用模块方式：

```bash
python -m packet_monitor.gui
```

## First-time use 第一次使用

1) Plug in your device via USB. Then open the app and click Refresh to see available ports.

1）用 USB 连接设备。打开应用后点击“Refresh（刷新）”查看可用串口。

2) Choose the Port (for example, /dev/cu.usbserial-xxxx or COM3 on Windows) and leave Baud at 115200 unless your firmware uses a different rate.

2）选择串口（例如 /dev/cu.usbserial-xxxx，或 Windows 上的 COM3）。波特率默认 115200，除非固件使用了其他数值。

3) Click Connect. You should see numbers changing and plots moving. If not, check Troubleshooting.

3）点击“Connect（连接）”。若成功，界面会显示数据刷新和图表变化；否则请查看“故障排查”。

## Basic actions 常用操作

- Start/stop recording: Press the “Start Recording” button or hit Space. CSV files are saved to packet_monitor/logs/ as data_YYYYMMDD_HHMMSS.csv.

- 开始/停止录制：点击“Start Recording（开始录制）”或按空格键。CSV 文件保存在 packet_monitor/logs/ 目录下，文件名类似 data_YYYYMMDD_HHMMSS.csv。

- Pause/resume chart: Click “Pause Chart” or press Backspace. Data still streams and can be recorded while charts are paused.

- 暂停/恢复图表：点击“Pause Chart（暂停图表）”或按退格键。暂停仅影响图表显示，数据接收和录制不受影响。

- Send parameters to device: Open the “Control (Send)” tab. Click “Load From Packet” to fill in current values from the latest packet, adjust if needed, then click “Send Packet”.

- 向设备发送参数：打开“Control (Send)”页签，先点击“Load From Packet（从数据包加载）”获取当前值，按需调整后点击“Send Packet（发送）”。

Shortcuts: Space = Record on/off · Backspace = Pause chart · Enter = Connect · Esc = Exit

快捷键：空格＝录制开/关 · 退格＝图表暂停 · 回车＝连接 · Esc＝退出

## Where files go 文件保存位置

- Logs are saved under packet_monitor/logs/ next to the app files. You can open the CSV in Excel or Numbers.

- 日志保存在应用同级目录的 packet_monitor/logs/ 下。可用 Excel 或 Numbers 打开 CSV。

## Upgrade and uninstall within venv 升级与卸载（在虚拟环境中）

Make sure your venv is active first.

请先确保虚拟环境已被激活。

Upgrade 升级：

```bash
pip install --upgrade akr-monitor
```

Uninstall 卸载：

```bash
pip uninstall akr-monitor
```

## Troubleshooting 故障排查

- I don’t see any serial ports after clicking Refresh.

- 点击“刷新”后没有看到串口：

	- Check the USB cable (use a data cable) and reconnect the device. Try a different USB port.

	- 检查 USB 线是否为数据线，并重插设备；尝试更换 USB 口。

	- Some adapters need drivers (CH340/CP210x/FTDI). Install the driver for your adapter, then reopen the app.

	- 某些转接器需要驱动（CH340/CP210x/FTDI）。安装对应芯片的驱动后重启应用。

- It says “Open port failed”.

- 显示“打开串口失败”：

	- Make sure no other program is using the port (close Arduino/other terminals) and try again.

	- 确认没有其他程序占用串口（关闭 Arduino/串口终端等）然后重试。

- The window doesn’t open after running akr.

- 运行 akr 后没有弹出窗口：

	- Ensure your venv is activated, then run python -m packet_monitor.gui to check for errors.

	- 确认已激活虚拟环境，然后运行 python -m packet_monitor.gui 查看报错信息。

	- Upgrade pip and reinstall inside the venv: python -m pip install --upgrade pip, then pip install -U akr-monitor.

	- 在虚拟环境内升级 pip 并重装：先 python -m pip install --upgrade pip，再 pip install -U akr-monitor。

- No data/flat charts after connecting.

- 连接后没有数据或图表不动：

Tip: Using a venv avoids most PATH issues. If “akr not found”, make sure the venv is active (you should see (.venv) at the prompt), or use python -m packet_monitor.gui.

提示：使用虚拟环境几乎可避免 PATH 问题。若提示“找不到 akr”，请确认已激活虚拟环境（命令行前有 (.venv)），或使用 python -m packet_monitor.gui。

	- Check that Baud matches your firmware (default 115200). Verify the selected Port is correct.

	- 检查波特率是否与固件一致（默认 115200），并确认选择了正确的串口。

## For advanced users 进阶说明

- Command line entry point (within venv): akr. Module entry: packet_monitor.gui:main

- 命令行入口（在虚拟环境内）：akr。模块入口：packet_monitor.gui:main

- Default baud rate is 115200 (see packet_monitor/cli.py). You can change it in the app before connecting.

- 默认波特率为 115200（见 packet_monitor/cli.py）。连接前可在界面中修改。

- CSV columns include time, pps, position, current, angles, acc/gyro, and states. Files are timestamped for easy tracking.

- CSV 列包含时间、pps、位置、电流、角度、加速度/陀螺和状态等，文件名带时间戳便于整理。

## License 许可协议

This project is under the MIT License (see LICENSE).

本项目使用 MIT 许可（见 LICENSE）。
