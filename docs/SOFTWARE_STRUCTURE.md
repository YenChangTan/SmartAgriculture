# SmartAgriculture firmware: structure and rewrite plan

This page sets out how the firmware is split into modules, and the order in which the rewrite is done. We follow it step by step on the `rewrite` branch.

Agreed on 2026-11-04. The host ↔ MCU contract is `docs/Smart Agriculture Communication Protocol v2.xlsx`.

---

## 1. Modules

| # | Section | File(s) | Responsibility |
|---|---|---|---|
| 1 | Communication with PC | `host_link.c/.h` | USART6: receive, detect end of message, parse JSON (jsmn), dispatch commands, build the status JSON, send the reply **without blocking** |
| 2 | Slave communication | `slave_io.c/.h` | RS-485 master to the IO slave board (USART2, PC4 direction): polling, timeouts, a dead slave's inputs set to 0. Moves bits between the wire and the IO layer |
| 3 | IO abstraction | `io.c/.h` | Process image (`RealPinState`), local GPIO, active-low inversion, debounce, logical name → index binding, `WriteOutput` / `ReadOutput` / `ReadInput` |
| 4 | Analog sensor | `level_sensor.c/.h` | Tank depth over ADC1 scan + DMA, and the LOW / HIGH / LIMIT points (done) |
| 5 | Modbus data request | `modbus.c/.h` (generic master) + `field_sensors.c/.h` | `field_sensors` owns the EC sensor and the flow meters A/B: polling schedule, scaling, averaging, error flags |
| 6 | Functional flow | `process.c/.h` + `task.c/.h` | `process`: the plan state machines (IDLE planning, operating states, ABORT, ERROR). `task`: the task queue, task and state history, error and warning codes |
| — | Settings | `settings.c/.h` | Every tunable value in one place (tank calibration, point depths, timeouts, EC gain). Saved to flash later |
| — | `main.c` | | CubeMX init, each module's `Init()`, then the main loop. No logic of its own |

If you'd rather not split sections 5 and 6, merge `field_sensors` into `modbus` and `task` into `process`. Nothing else changes.

## 2. Rules

1. **Calls only go downward:** `host_link` → `process` / `task` → `io`, `slave_io`, `level_sensor`, `field_sensors` → HAL. A lower module never calls a higher one.
2. **Sensor modules only keep values up to date.** Each keeps its latest value and an error flag; `process` reads them and never waits for a reading.
3. **Only `io` touches GPIO.** Everything else uses `WriteOutput()` / `ReadInput()`.
4. **`host_link` changes state only through functions,** for example `Task_Write()`, `Task_Clear()`, `Process_Abort()`, `Process_EStop()`, plus getters for the status JSON.
5. **Interrupts only collect bytes or samples and set flags.** All processing happens in the main loop.
6. **Nothing blocks.** Every `Service()` call returns immediately; timing uses `HAL_GetTick()` timestamps.
7. **`LEVEL_SENSOR_ANALOG`** (in `main.h`) chooses analog level sensors (1) or the digital level switches (0).

## 3. Main loop

```c
while (1) {
    IO_Update();            // read inputs (debounce), write outputs
    SlaveIO_Service();      // exchange remote IO
    LevelSensor_Service();  // tank depths -> level points
    FieldSensors_Service(); // EC / flow meters over Modbus
    Process_Run();          // state machine for the selected plan
    HostLink_Service();     // handle PC command, send status (non-blocking)
}
```

---

## 4. Rewrite plan

**For every step:**
1. Implement it, moving the code it touches into its module.
2. Build with `LEVEL_SENSOR_ANALOG` 1 and 0.
3. Test on the bench board.
4. Commit.
5. Update the Excel if the PC sees a change.

`G#` is the gap number in the PRD.

### Phase A: level sensors
- [x] A1 Commit the level sensor wiring (`LEVEL_SENSOR_ANALOG`, `lvl` in the status JSON)
- [ ] A2 Bench test: `lvl` and the derived `inputs[]` follow the sensors; `LEVEL_SENSOR_ANALOG 0` behaves like the old firmware
- [ ] A3 Sensor fault stops the machine: a tank sensor in `err` raises error `1XX12` and goes to ERROR
- [ ] A4 Calibrate the real tanks: `fullRaw`, real Low/High/Limit depths

### Phase B: communication and safety
- [ ] B1 Non-blocking host reply (`HAL_UART_Transmit_IT`); reply buffer 4 KB (rule 11.4)
- [ ] B2 Fix the misplaced parenthesis in the reservoir-low check (G12); reset the EC-step timers on state entry (G9)
- [ ] B3 EC fault in IDLE gives `10005` (G4); IDLE errors also go into state history (G8)

### Phase C: state machine behaviour (PRO)
- [ ] C1 Reservoir refill works: pump, exits `103` / `110`, timeout `10101`, idle refill below Warning (G1, G2)
- [ ] C2 No safety timeout in watering states (G3)
- [ ] C3 Watering time saved on every exit, including the slave-board-failure exit (G11)
- [ ] C4 No no-flow check in manual watering (G13)

### Phase D: tasks and calibration
- [ ] D1 `write_task` updates the running task: same plot keeps its watering time, a different plot resets it (G5)
- [ ] D2 Task history: fix the `start` timestamp and the indexing after the queue wraps around (G6, G7)
- [ ] D3 `calibrate_ec` (Excel format): fix the inverted condition, set only the gain (G10)

### Phase E: Modbus rework
- [ ] E1 Each device on its own period, timed with `HAL_GetTick()`, no drift (G15)
- [ ] E2 EC fast (1 s) while adding fertilizer or lowering EC, slow (10 s) otherwise; flow meters only during fertilizer adding, retried while faulted (G14)
- [ ] E3 Device error after 3 failed reads; word-order setting per flow meter; EC averaging with α fast / α slow (G18–G20)
- [ ] E4 Fresh flow totals before dosing; IDLE uses an EC reading younger than 10 s (G16, G17)

### Phase F: later
- [ ] F1 Settings saved in flash
- [ ] F2 Clean-up: remove the unused `StateMachineRunning`, decide on `dynamic_io.h`, optional watchdog, optional time-based debounce
- [ ] F3 Spec and rewrite: IO link, manual mode, then the LITE / PRO+ plans
