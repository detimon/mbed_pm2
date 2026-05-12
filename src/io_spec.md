# I/O Specification — PES Board (Nucleo F446RE)

## Shared Pins (all tests)

| Signal          | Mbed Pin | Board Pin | Direction | Description                              |
|-----------------|----------|-----------|-----------|------------------------------------------|
| User Button     | BUTTON1  | PC_13     | Input     | Blue button on Nucleo, toggles task on/off |
| User LED        | LED1     | PA_5      | Output    | Green LED on Nucleo, blinks every cycle  |
| External LED    | PB_9     | D14       | Output    | Lights when main task is active; add 220–500 Ω series resistor |

---

## TEST_SERVO — Servo Motor Test

Select by setting `#define TEST_SERVO` in `test_config.h`.

| Signal       | Mbed Pin | Board Pin | Direction   | Description                                       |
|--------------|----------|-----------|-------------|---------------------------------------------------|
| Servo signal | PB_D1    | PC_8 / D6 | Output (soft PWM) | PWM signal to servo (FS90); pulse range 0.050–0.105 s |

### Behaviour
- Press blue button → servo enables and slowly sweeps from 0 to 100 % position (+0.5 % per second).
- Press again → servo disables, position resets to 0.
- Serial output: `Servo input: X.XXXX`

---

## TEST_COLOR_SENSOR — TCS3200/TCS230 Color Sensor Test

Select by setting `#define TEST_COLOR_SENSOR` in `test_config.h`.

Wiring verified against schematic (CN7A + CN10A):

| Signal          | Mbed Pin | Nucleo Connector | Direction | Description                              |
|-----------------|----------|-----------------|-----------|------------------------------------------|
| Frequency out   | PB_3     | CN10A pin 31    | Input     | PWM frequency output of the sensor       |
| LED / OE enable | PH_0     | CN7A  pin 29    | Output    | Enables the sensor's white illumination LED |
| S0 (freq scale) | PA_4     | CN7A  pin 32    | Output    | Frequency scaling select bit 0           |
| S1 (freq scale) | PB_0     | CN7A  pin 34    | Output    | Frequency scaling select bit 1           |
| S2 (filter sel) | PC_1     | CN7A  pin 36    | Output    | Color filter select bit 0                |
| S3 (filter sel) | PC_0     | CN7A  pin 38    | Output    | Color filter select bit 1                |

### Filter / Frequency Scaling Encoding

| S0 | S1 | Scaling |     | S2 | S3 | Active filter |
|----|-----|---------|-----|----|-----|---------------|
| 0  | 0   | Off     |     | 0  | 0   | Red           |
| 0  | 1   | 2 %     |     | 0  | 1   | Blue          |
| 1  | 0   | 20 %    |     | 1  | 0   | Clear (no filter) |
| 1  | 1   | 100 %   |     | 1  | 1   | Green         |

### Behaviour
- Press blue button → starts reading; terminal updates in-place every ~240 ms (no scroll).
- Press again → stops reading, values reset to 0.
- Detected color name is printed in its actual ANSI color (red/green/blue/white).
- Status shows **[ACTIVE]** (green) or **[STOPPED]** (yellow).
- Requires a terminal with ANSI escape code support (PlatformIO serial monitor: ✓).

---

---

## TEST_COLOR_SENSOR_CALIB — Color Sensor Calibration Helper

Select by setting `#define TEST_COLOR_SENSOR_CALIB` in `test_config.h`.

Same wiring as TEST_COLOR_SENSOR (see table above).

### Behaviour
- Press blue button → starts reading; display updates in-place every ~500 ms (slow refresh for easy reading).
- Each channel (R / G / B / W) is printed in its corresponding ANSI colour.
- Hold a known colour in front of the sensor and **record the Avg Hz values** for each channel.
- Repeat for every colour you want to classify, then update the calibration values in the `ColorSensor` library.
- Press again → stops and resets.

---

## How to Switch Tests

1. Open `src/test_config.h`
2. Uncomment exactly ONE `#define`:

```cpp
// #define TEST_SERVO
// #define TEST_COLOR_SENSOR
#define TEST_COLOR_SENSOR_CALIB
```

3. Rebuild and flash.
