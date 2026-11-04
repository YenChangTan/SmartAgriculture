# Analog level sensor: requirements, test plan and TODO

Replace each tank's digital level switches with one analog level sensor that reports the real water depth. Do it first in a **separate test project**, then bring the same module into SmartAgriculture with as little change as possible.

- Module: `Core/Inc/level_sensor.h`, `Core/Src/level_sensor.c` (written, compiles cleanly, **not yet tested on hardware**)
- Signal chain: level sensor (4–20 mA) → current-to-voltage converter → STM32 ADC pin

---

## 1. Requirements

### 1.1 Non-blocking
- R1. Reading the sensors must never block the main loop. `LevelSensorBus_Service()` is called once per main-loop pass and returns immediately. On one pass it starts an ADC conversion; on a later pass it reads the result.
- R2. The only blocking read is a single one at start-up, inside `LevelSensorBus_Add()`, of at most 5 ms per tank. It is there so each tank has a valid level before the main loop starts.
- R3. Timing uses `HAL_GetTick()` timestamps, not a timer interrupt. A reading may start up to one main-loop pass late, and the delay never builds up from one period to the next.
- R4. The rest of the program never triggers a reading and never waits for one. It only reads the stored values.

### 1.2 Easy to use
- R5. One `LevelSensor` object per tank, and one `LevelSensorBus` per ADC (up to 6 tanks per bus).
- R6. All settings for a tank are in one `LevelSensorConfig` struct. Setting up a tank takes 3 calls: `LevelSensor_Init()`, `LevelSensorBus_Init()` (once per bus) and `LevelSensorBus_Add()`.
- R7. The main loop needs one call: `LevelSensorBus_Service(&bus)`.
- R8. Reading values needs no knowledge of the ADC: `LevelSensor_GetDepth()`, `LevelSensor_IsError()` and `LevelSensor_ReadPoint()`.

### 1.3 Same meaning as the digital switches (drop-in)
- R9. Each tank has 3 points: `LEVEL_SENSOR_LOW`, `LEVEL_SENSOR_HIGH` and `LEVEL_SENSOR_LIMIT`.
  - Reservoir and fertilizer tanks: Warning → `HIGH`, High → `LIMIT`.
  - Mix1 and Mix2: Low → `LOW`, High → `HIGH`, Limit → `LIMIT`.
- R10. `LevelSensor_ReadPoint()` returns `GPIO_PIN_SET` when the water is **below** the point, which is exactly what `ReadInput()` returns for a digital switch today. That way the existing state machines keep working unchanged.
- R11. Each tank has its own point depths (`pointDepth[]`) and can switch off points it doesn't use (`isPointUsed[]`).
- R12. Hysteresis: a point counts as reached at `pointDepth`, and as released only once the depth drops below `pointDepth − hysteresis`. This stops it flickering when the water sits right at that level.

### 1.4 Signal processing
- R13. Scaling: `depth = a × raw + b`. `a` and `b` are set per tank and can be changed at run time with `LevelSensor_SetCalibration()`.
- R14. Exponential averaging: `depth = α × new + (1 − α) × depth`, with α set per tank (0 < α ≤ 1; α = 1 means no averaging).
- R15. Averaging restarts from the next good reading after a calibration change or after a fault clears.
- R16. Fault detection: a raw value outside `rawMin..rawMax` counts as a failed reading (broken or shorted wire). After 3 failed readings in a row (`failThreshold`) the error flag is set. One good reading clears it.

### 1.5 Compare mode (test project, and the first install)
- R17. During testing, keep the digital switches wired and log the digital switch next to the analog point for the same level.
- R18. A mismatch is only a real error if it lasts longer than the hysteresis band and the switch's own tolerance can explain.

### 1.6 For now
- R19. Settings are written into the code (no flash storage yet). Changing them means rebuilding and reflashing.
- R20. The sensor's error flag is not yet used by the SmartAgriculture state machines (see the open decision in section 6).

---

## 2. Hardware setup (converter)

The converter must keep the signal **safe for the pin** and **able to show a broken wire**. Accuracy comes from the software calibration.

1. Set the converter's output-range jumper to **0–3.3 V**. **Never use the 5 V or 10 V range on an STM32 pin.**
2. Power the converter and measure its output with a multimeter **before** connecting it to the STM32.
3. **ZERO:** sensor out of the water or tank empty (≈ 4 mA). Turn ZERO to **0.5 V** (0.3–0.7 V is fine).
4. **MAX:** at the sensor's rated full depth (≈ 20 mA), turn MAX to **3.0 V**.
   - With a real 2 m of water and a 0–2 m sensor, use the water.
   - Without it, set MAX at a measured depth to: V = 0.5 + 2.5 × (depth ÷ rated range).
5. Repeat steps 3–4 once; the two trimmers affect each other a little.
6. Safety check: at the **highest water level the tank can physically reach**, the output must stay **below 3.3 V**. If not, turn MAX down (for example to 2.8 V at full scale).
7. Connect the converter output to the ADC pin and the converter **GND to the STM32 GND**. Optionally add an RC filter at the pin (1 kΩ in series, 100 nF to GND).

Notes:
- The sensor gives 20 mA at its **rated** depth, not at "full tank". A 0–5 m sensor in 2 m of water gives about 10.4 mA. That's normal.
- Fertilizer tanks: calibrate with the real solution. It is denser than water, so the same depth reads higher.

---

## 3. Software calibration (per tank)

Treat the sensor and converter together as one straight line from depth to ADC count. Two measured points correct the errors of both parts at once.

1. Tank empty → note `raw₀`.
2. Fill to a depth measured with a tape, as close to full as possible → note `d₁` and `raw₁`.
3. `a = d₁ / (raw₁ − raw₀)`, `b = −raw₀ × a`.
4. Optional: check a third depth around the middle. It should land within a few mm of the measured depth.
5. Set `rawMin` a little below `raw₀` (for example 0.4 V ≈ 500), and `rawMax` a little above the full-scale value but under 3.3 V (for example ≈ 3950).

Example (0–2 m sensor, real parts slightly off): `raw₀` = 645 at 0 mm, `raw₁` = 3338 at 1800 mm, which gives `a` = 0.6684 mm/count and `b` = −431 mm.

---

## 4. Test project plan

### 4.1 Setup
- New STM32CubeIDE project for **STM32F407VET6**, separate from SmartAgriculture.
- ADC1: channel 1 on **PA1** (the same pin as SmartAgriculture), 12-bit, single conversion, software start. For the multi-tank test, add a second channel (for example PA0 = IN0 or PA4 = IN4).
- One UART for logging (for example USART6, 115200), or watch the variables live in the debugger.
- One GPIO input for a digital level switch, for compare mode (for example PD11, active-low like the main board).
- Copy `level_sensor.h` and `level_sensor.c` **unchanged**.
- Before the real sensor arrives, use a **10 kΩ potentiometer between 3.3 V and GND** with its wiper on PA1 to simulate it.

### 4.2 Test cases
Settings for the potentiometer tests: `a` = 1, `b` = 0 (depth = raw count), `pointDepth` = {1000, 2500, 3000}, `hysteresis` = 100, `samplePeriod` = 100 ms, `rawMin` = 200, `rawMax` = 3900.

| # | Test | How | Expected result |
|---|---|---|---|
| T1 | Start-up | Power on with the potentiometer in the middle | Depth valid on the first loop pass; points match the position |
| T2 | Rising | Turn slowly from 0 to max | LOW, HIGH and LIMIT go `SET` → `RESET` at 1000, 2500 and 3000 |
| T3 | Falling | Turn slowly from max to 0 | Each point goes back to `SET` only below 900, 2400 and 2900 |
| T4 | No chatter | Hold right at 1000 | LOW does not toggle |
| T5 | Broken wire | Pull the wiper wire (raw < 200) | Error flag set after about 300 ms (3 readings); points keep their last value |
| T6 | Recovery | Reconnect | Error clears on the first good reading; averaging restarts with no slow ramp |
| T7 | Over-range | Turn to max (raw > 3900) | Error flag set after 3 readings |
| T8 | Averaging | Step the potentiometer quickly | Depth moves smoothly; settles faster with α = 0.5, slower with α = 0.1 |
| T9 | Calibration change | Call `LevelSensor_SetCalibration()` at run time | Depth jumps straight to the new scale, with no blending |
| T10 | Non-blocking | Toggle a GPIO each loop pass, or log scan time, with and without `Service()` | Loop time unchanged to within microseconds; no pass is held for a conversion |
| T11 | Several tanks | Two channels, periods 100 ms and 500 ms | Each tank keeps its own period; no channel mixing |
| T12 | Compare mode | Digital switch and real sensor in the same tank | Switch and analog point agree within the hysteresis band |

### 4.3 Real-sensor test
1. Do the hardware setup (section 2) and the calibration (section 3).
2. Repeat T2–T5 with real water, and T12 against the existing digital switch.
3. Record `raw₀`, `raw₁`, `a`, `b` and the agreed `pointDepth` values for each tank.

---

## 5. TODO

### Test project
- [ ] Create the test project (F407VET6, ADC1 IN1 on PA1, UART log, one digital input)
- [ ] Copy `level_sensor.h/.c` unchanged
- [ ] Bench tests T1–T11 with a potentiometer
- [ ] Set up the converter: 0–3.3 V range, ZERO 0.5 V, MAX 3.0 V, check it stays under 3.3 V
- [ ] Wire the real sensor and converter, with shared GND and the optional RC filter
- [ ] Two-point calibration with real water; record `a` and `b`
- [ ] Compare test T12 against the digital switch
- [ ] Fix anything found and update the module

### Bring into SmartAgriculture (after the test project passes)
- [ ] Add the Mix1 `LevelSensorConfig` and `LevelSensorBus_Add()` calls to `MachineSettingInit()`
- [ ] Call `LevelSensorBus_Service()` in the main loop
- [ ] Feed the Mix1 Low, High and Limit inputs from `LevelSensor_ReadPoint()` at the end of `IOListUpdate()`
- [ ] Add sensor-fault handling: one check at the top of `StateMachineRunningPro()`, new error `1XX12` (needs a decision)
- [ ] Add Mix1 depth and level error to the status JSON
- [ ] Keep the digital Limit switch wired as an overflow backup during the trial
- [ ] Then the other tanks, one at a time (each needs its own ADC channel in CubeMX)

### Later
- [ ] Save `a`, `b`, `pointDepth`, `hysteresis` and `alpha` in flash (shared settings storage, see the PRD)
- [ ] Let the host set calibration and thresholds
- [ ] Update protocol sheets 3.1 (status fields) and 3.4 (new error code)

---

## 6. Open decisions
- Which tanks get analog sensors, and in what order. Suggested: Mix1 first.
- The sensors' rated range (for example 0–2 m), and which ADC pin each tank uses.
- Depth unit: mm is suggested.
- What to do when a level sensor fails: suggested, a new error `1XX12` that sends the machine to ERROR.
- Whether to keep the digital Limit switch permanently as a hardware overflow guard.

---

## 7. API reference

```c
void LevelSensor_Init(LevelSensor *sensor, const LevelSensorConfig *config);
void LevelSensor_SetCalibration(LevelSensor *sensor, float a, float b);
float LevelSensor_GetDepth(const LevelSensor *sensor);
uint8_t LevelSensor_IsError(const LevelSensor *sensor);
GPIO_PinState LevelSensor_ReadPoint(const LevelSensor *sensor, LevelSensorPoint point); /* SET = water below point */

void LevelSensorBus_Init(LevelSensorBus *bus, ADC_HandleTypeDef *hadc);
uint8_t LevelSensorBus_Add(LevelSensorBus *bus, LevelSensor *sensor);   /* 0 = bus full */
void LevelSensorBus_Service(LevelSensorBus *bus);                       /* every main-loop pass */
```

Minimal use:

```c
LevelSensorBus levelBus;
LevelSensor levelMix1;

LevelSensorConfig mix1Config = {
	.adcChannel = ADC_CHANNEL_1, .samplingTime = ADC_SAMPLETIME_480CYCLES, .samplePeriod = 100,
	.rawMin = 500, .rawMax = 3950, .a = 0.6684f, .b = -431.0f, .alpha = 0.2f, .failThreshold = 3,
	.isPointUsed = {1, 1, 1}, .pointDepth = {100.0f, 800.0f, 900.0f}, .hysteresis = 20.0f,
};
LevelSensor_Init(&levelMix1, &mix1Config);
LevelSensorBus_Init(&levelBus, &hadc1);
LevelSensorBus_Add(&levelBus, &levelMix1);

while (1)
{
	LevelSensorBus_Service(&levelBus);
	if (LevelSensor_ReadPoint(&levelMix1, LEVEL_SENSOR_LOW) == GPIO_PIN_SET) { /* Mix1 is low */ }
}
```
