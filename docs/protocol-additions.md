# Additions to "Smart Agriculture Communication Protocol.xlsx"

**The Excel file is the contract between the host PC and the MCU, and the rewrite keeps it.** This page lists only what gets **added** to it. Nothing already in the Excel changes meaning or format.

Decided on 2026-11-04.

---

## 1. `calibrate_ec`: unchanged (sheets 2.1–2.3)

The request stays exactly as in the Excel:

```json
{"event":"calibrate_ec","manual":{"targetEC":1413.0}}
```

The behaviour behind it gets three fixes. None of them changes the protocol.
- **Gain from one reference.** With the EC sensor in a reference solution, the firmware sets the gain to `targetEC / current EC reading`. The offset is unchanged.
- **Only with a working sensor.** It calibrates only when the EC sensor is **not** faulted. The current firmware has this condition inverted.
- **Kept after power loss.** The calibration is saved in flash.

---

## 2. Sheet 2.1 (events) and 2.2/2.3 (request JSON): add `writeIO`

The current firmware already accepts this event. It was never in the Excel.

| No | Event | Description | Available in state | Available in |
|---|---|---|---|---|
| 10 | `writeIO` | Switch one output ON or OFF directly. Ignored unless the machine is in `STATE_IDLE` | STATE_IDLE | Auto/Manual |

Request:

```json
{"event":"writeIO","writeio":{"index":3,"isOn":1}}
```

| Key | Type | Range | Notes |
|---|---|---|---|
| `writeio.index` | uint8_t | 0–31 | IO index, the same numbering as `outputs[]`. 0–15 local, 16–31 IO slave board |
| `writeio.isOn` | uint8_t | 0 / 1 | 1 = ON |

---

## 3. Sheet 3.1 (status JSON): add these keys

`plan`, `refillReservoirControl`, `IncludePlanReservoirAndIO`, `taskCount`, `outputs` and `inputs` are already sent by the current firmware. `lvl` is new.

| Json key | Type | Description |
|---|---|---|
| `plan` | string | `"LITE"`, `"PRO"`, `"PRO+"` or `"TEST"`. Set by the DIP switches PC0/PC1 at power-up |
| `refillReservoirControl` | 0/1 | 1 = automatic reservoir refill is on (DIP switch PC2) |
| `IncludePlanReservoirAndIO` | 0/1 | DIP switch PC3 |
| `taskCount` | uint8_t | Number of tasks in the queue (0–10) |
| `outputs` | array of 32 × 0/1 | Every output, in IO-index order: 0–15 local, 16–31 IO slave board. 1 = ON |
| `inputs` | array of 32 × 0/1 | Every input, in IO-index order. 1 = ON (active-low, debounced). With analog level sensors, the level inputs show the switch states calculated from the depth |
| `lvl` | array, one entry per tank with an analog level sensor | Water level of each tank. **New**, in the same format as the TestAnalog bench firmware |

Fields of each `lvl` entry:

| Key | Type / unit | Description |
|---|---|---|
| `tank` | string | `Reservoir`, `Mix1`, `Mix2`, `FertA`, `FertB` |
| `st` | string | `ok` = valid reading; `wait` = no reading yet after start-up; `err` = 3 bad readings in a row (open or shorted wire, wrong converter range) |
| `raw` | 0–4095 | ADC reading (average of 16 samples) |
| `mv` | mV | `raw` as a voltage, assuming a 3.3 V reference |
| `mm` | mm | Water depth (filtered). Only meaningful when `st` = `ok` |
| `below` | `[LOW, HIGH, LIMIT]`, each 0/1 | 1 = water is below that point, the same as a digital level switch. Reservoir and fertilizer tanks: Low, Warning, High. Mixing tanks: Low, High, Limit |

Example entry:

```json
"lvl":[{"tank":"Reservoir","st":"ok","raw":2172,"mv":1750,"mm":1000,"below":[0,0,1]},…]
```

---

## 4. Sheet 3.3 (exit codes): add reason 10

| Exit reason | Code (YY) | Used by state | Exit code |
|---|---|---|---|
| Reservoir above warning (a task is waiting) | 10 | STATE_WATER_RESERVOIR_REFILL (1) | **110** |

Meaning: automatic refill stops as soon as the reservoir is above its Warning level **and** a task is waiting, so the task can carry on. Without a waiting task, refill continues until the reservoir High level (exit code 103, already in the Excel).

---

## 5. Sheet 3.4 (error codes): add one column, mark two IDLE codes

**New column:** error 12, **Level sensor fault**.
- Raised when a tank's analog level sensor reports `err` while the machine needs that tank's level.
- Code = `10000 + state × 100 + 12`. For example 10012 in IDLE, 10512 while filling Mix1, 10612 while adding fertilizer.

**Codes in existing columns that become possible in STATE_IDLE** (currently N/A in the Excel):

| Code | Column | Meaning |
|---|---|---|
| **10003** | Reservoir low (3) | IDLE needs reservoir water, the reservoir is low, and auto refill is off. The current firmware can already raise this; it was missing from the Excel |
| **10005** | EC reading error (5) | IDLE needs the EC to decide the next step, but the EC sensor is faulted |
