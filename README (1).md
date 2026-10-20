

Embedded Systems final project – USTH, 2026.

A cabin-mounted device on a custom ATmega328P (16 MHz) PCB that detects a person trapped in a sealed car and raises a 5-level alarm.
 PIR HC-SR501 | Motion (primary trigger)
 MQ-135 | Estimated CO2 (ppm) 
 DHT22 
 Buzzer
 SD card 
 MCP2515 + TJA1050 | CAN 125 kbps

SPI (D13/D11/D12) is shared by SD and CAN. Serial log at 9600 baud.



Step one: Check each cases:
Case 1: occupied, ventilated (temperature keeps dropping)
Case 2: empty (no motion, stable T and CO2, CO2 < 600 ppm)
Case 3: occupied, sealed (CO2 ≥ 600 ppm in, < 500 ppm out)

Step 2: Check weighted based on algorithm
- CO2: +3 / +6 / +9 at 1500 / 5000 / 10000 ppm
- Temperature: +1 / +2 at two thresholds
- +2 if no motion for 10 s while CO2/T already scores (possibly unconscious)
- +1 if CO2 and T rise for 2 consecutive samples
- Score ≥3 → WARNING, ≥6 → DANGER, ≥9 → EMERGENCY. Level rises one step per sample and drops only after 3 lower samples.

These are info injected into SD card for further processing and graph calculation.
| Content | CO2 ppm (BE) | Temp °C | PIR | Level 0–4 | dC (signed) | TH(2b) · reason(2b) · score(4b) |

- MQ-135 CO2 is an estimate (Gironi 2014 curve), less accurate than NDIR.
- Time/temperature thresholds are shortened for demo and need recalibration for real use.
