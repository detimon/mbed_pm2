---
glob: include/PESBoardPinMap.h
---

# Regeln für Pin-Zuordnung (PES Board)

## Aktive Board-Version
`NEW_PES_BOARD_VERSION` ist definiert — immer die neuen Pin-Nummern verwenden.

## Kritische Pin-Konflikte
| Pin  | Funktion 1          | Funktion 2         | Regel                              |
|------|---------------------|--------------------|------------------------------------|
| PB_9 | External LED (D14)  | I2C1 SDA (SensorBar) | Bei LINE_FOLLOWER → led1 = PB_10  |
| PB_8 | —                   | I2C1 SCL (SensorBar) | Nicht als DigitalOut verwenden    |
| PB_10| Fallback-LED        | (normalerweise UART) | Nur bei LINE_FOLLOWER für led1    |

## Motor-Pin-Zuordnung (Neue Board-Version)
- M1: PWM=PB_13, ENC_A=PA_6, ENC_B=PC_7
- M2: PWM=PA_9,  ENC_A=PB_6, ENC_B=PB_7
- M3: PWM=PA_10, ENC_A=PA_0, ENC_B=PA_1
- ENABLE: PB_15

## Servo / Digital Out
- PB_D0=PB_2, PB_D1=PC_8, PB_D2=PC_6, PB_D3=PB_12

## SD-Karte (SPI)
- MOSI=PC_12, MISO=PC_11, SCK=PC_10, CS=PD_2

## IMU (I2C2)
- SDA=PC_9, SCL=PA_8 — anderer I2C-Bus als SensorBar (I2C1)

## Änderungen an PESBoardPinMap.h
Nur vornehmen wenn sich die physikalische Hardware ändert.
Immer beide Board-Versionen (`#ifdef NEW_PES_BOARD_VERSION / #else`) synchron halten.
