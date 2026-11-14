# AeroTelem — Embedded C++17 & ROS 2 Telemetry Driver

Lightweight binary telemetry protocol parser and real-time driver written in **C++17**, designed for serial (**UART**) communication between an embedded flight/robot controller (**STM32**) and an onboard Linux computer (**ROS 2**).

## Architecture & Features

* **Binary Frame Protocol (`encode_frame` / `decode_frame`)**:
  * Fixed 8-byte packet structure with dual synchronization headers (`0xAA 0x55`).
  * 16-bit signed attitude encoding (`roll`, `pitch` in decidegrees, MSB/LSB big-endian packing) and 8-bit battery status.
  * XOR payload checksum verification for noise rejection on serial lines.
* **Byte-Stream State Machine (`FrameParser`)**:
  * Non-blocking byte-by-byte parser (`WAIT_HEADER_1` $\rightarrow$ `WAIT_HEADER_2` $\rightarrow$ `READ_PAYLOAD`) resilient to line noise and false header starts.

## Build & Run Tests (Linux / WSL2)

```bash
mkdir -p build && cd build
cmake ..
make
./aerotelem_node
```
