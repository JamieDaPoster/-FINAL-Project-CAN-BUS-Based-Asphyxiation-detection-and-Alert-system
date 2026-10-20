# In-Car Asphyxiation Warning System (CAN Bus) – ATmega328P

Embedded Systems final project – USTH, 2026.

A cabin-mounted device on a custom ATmega328P (16 MHz) PCB that detects a person trapped in a sealed car and raises a 5-level alarm: `EMPTY_CAR`, `SAFE`, `WARNING`, `DANGER`, `EMERGENCY`.

Firmware: [`canh_bao_ngat_khi_v4_B_MQ135_1.cpp`](canh_bao_ngat_khi_v4_B_MQ135_1.cpp)

## Hardware

| Device | Role | Pin (Arduino / AVR) |
|---|---|---|
| PIR HC-SR501 | Motion (primary trigger) | D3 / PD3 |
| MQ-135 | Estimated CO2 (ppm) | A0 / PC0 |
| DHT22 | Cabin temperature | D7 / PD7 |
| Buzzer | Beep pattern per level | D5 / PD5 |
| SD card | Logs 30 points (60 s) as CSV | CS D4 / PD4 |
| MCP2515 + TJA1050 | CAN 125 kbps, frame every 2 s | CS D10 / PB2, INT D2 / PD2 |

SPI (D13/D11/D12) is shared by SD and CAN. Serial log at 9600 baud.

## Algorithm (2 s cycle)

**B1 – Classify the situation**
- **TH1** occupied, ventilated (temperature keeps dropping)
- **TH2** empty (no motion, stable T and CO2, CO2 < 600 ppm)
- **TH3** occupied, sealed (CO2 ≥ 600 ppm in, < 500 ppm out)

**B2 – Danger score (TH3 only)**
- CO2: +3 / +6 / +9 at 1500 / 5000 / 10000 ppm
- Temperature: +1 / +2 at two thresholds
- +2 if no motion for 10 s while CO2/T already scores (possibly unconscious)
- +1 if CO2 and T rise for 2 consecutive samples
- Score ≥3 → WARNING, ≥6 → DANGER, ≥9 → EMERGENCY. Level rises one step per sample and drops only after 3 lower samples.

## CAN Frame (ID 0x001, 8 bytes)

| Byte | 0–1 | 2 | 3 | 4 | 5–6 | 7 |
|---|---|---|---|---|---|---|
| Content | CO2 ppm (BE) | Temp °C | PIR | Level 0–4 | dC (signed) | TH(2b) · reason(2b) · score(4b) |

## Build notes

- Libraries: `SPI`, `SD`, `mcp_can`.
- Set `CAN_CRYSTAL` to `MCP_16MHZ` if your MCP2515 module uses a 16 MHz crystal (default 8 MHz).
- Power on in fresh air: MQ-135 warms up 20 s, then calibrates Ro assuming 400 ppm.

## Limitations

- MQ-135 CO2 is an estimate (Gironi 2014 curve), less accurate than NDIR.
- Time/temperature thresholds are shortened for demo and need recalibration for real use.
