# FOPI-Controlled Parallel Buck-Boost Converter — FPGA Implementation

---

## Overview

This project implements a **two-phase interleaved parallel buck-boost converter** for Wind Energy Conversion Systems (WECS), controlled by a **Fractional Order Proportional-Integral (FOPI) controller** synthesized on an **Intel DE10-Standard FPGA (Cyclone V)**.

The system accepts a wildly variable DC input (30 V – 400 V, representing real wind conditions) and regulates a stable **55 V DC output** at 1 kW, with the complete closed-loop control loop — ADC sensing → voltage scaling → FOPI computation → PWM generation — running entirely on the FPGA in Verilog.

---

## Key Specifications

| Parameter | Value |
|---|---|
| Input Voltage Range | 30 V – 400 V |
| Regulated Output Voltage | 55 V DC |
| Output Power | 1 kW |
| Output Current | 18.18 A |
| Load Resistance | 3.025 Ω |
| Per-phase Inductance | 0.4 mH |
| Output Capacitance | 363 µF |
| Switching Frequency | 50 kHz |
| Control Update Rate | 10 kHz |
| FPGA Clock | 100 MHz (Cyclone V) |
| ADC Resolution | 12-bit |
| PWM Resolution | 10-bit (0–1000 counts) |
| Dead-time | 20 clock cycles |
| Duty Cycle Limits | 300 – 950 (out of 1000) |

---

## System Architecture

```
                    ┌──────────────────────────────────────────┐
                    │              DE10-Standard FPGA           │
  12-bit ADC ──────►│  voltage_scale ──► fopi_controller_55V  │
  (adc_data)        │                          │               │
  adc_ready ───────►│  control_tick_gen ───────►               │
                    │                    duty_latch             │
                    │                          │               │
                    │                  pwm_deadtime_2mosfet    │
                    └──────────┬───────────────┬───────────────┘
                               │               │
                           pwm_high         pwm_low
                           (QH MOSFET)    (QL MOSFET)
```

### Module Breakdown

| Module | Function |
|---|---|
| `top_fopi_pwm_system` | Top-level interconnect |
| `voltage_scale` | Converts 12-bit ADC code → millivolt value using `(adc_data × 3000mV × 28) / 4095` |
| `control_tick_gen` | Divides 100 MHz clock to generate a 10 kHz one-cycle pulse for control rate |
| `fopi_controller_55V` | Incremental FOPI: computes duty cycle from error and previous error at control rate only |
| `duty_latch` | Holds duty cycle constant between control ticks; updates only on `control_tick` |
| `pwm_deadtime_2mosfet` | 50 kHz PWM counter with complementary high/low outputs and programmable dead-time |
| `buckboost_plant_55V` | First-order averaged plant model (testbench/simulation use only) |

---

## FOPI Controller Design

### Controller Transfer Function

```
C(s) = Kp + Ki / s^λ
```

### Tuned Parameters (SIMC Iso-Damping Method)

| Parameter | Value |
|---|---|
| Kp | 0.000403 |
| Ki | 5.990702 |
| λ (fractional order) | 1.318576 (≈ 0.67 effective) |
| Gain crossover frequency ωc | 0.24 rad/s |

The fractional order λ was selected based on the system's relative delay ratio τ = L/T = 5×10⁻⁶ / 2.48×10⁻⁵. For FOPTD systems with 0.1 < τ < 0.4, optimal λ ranges between 0.6–0.8.

### Discrete-Time FOPI (Verilog approximation)

```
acc[n] = acc[n-1] + (KP × (error[n] - error[n-1])) >> 7
                  + (KI × error[n]) >> 9
```

This incremental form runs **only on `control_tick`** (10 kHz), with the duty latch holding the value between ticks. This is the critical hold-state architecture that prevents PWM jitter.

---

## Stability Analysis Across Operating Points

| Vin (V) | Mode | Duty Cycle | DC Gain (dB) | ωn (rad/s) | Damping ζ | PM (°) | GM (dB) |
|---|---|---|---|---|---|---|---|
| 30 | Boost | 0.4545 | 45.3 | 2024 | 0.2249 | 60.0 | 18.0 |
| 40 | Boost | 0.2727 | 40.3 | 2699 | 0.1687 | 61.6 | 23.5 |
| 55 | Buck | 1.0000 | 34.8 | 3711 | 0.1227 | 62.1 | 142.2 |
| 100 | Buck | 0.5500 | 40.0 | 3711 | 0.1227 | 62.9 | 137.0 |
| 200 | Buck | 0.2750 | 46.0 | 3711 | 0.1227 | 64.6 | 131.0 |
| 400 | Buck | 0.1375 | 52.0 | 3711 | 0.1227 | 68.2 | 125.0 |

Phase margin remains near-constant (60°–68°) across the full operating range — iso-damping is achieved. Gain margin exceeds 120 dB in buck mode, confirming robust operation.

---

## RTL Module Details

### `voltage_scale`

```verilog
// VREF = 3000 mV, Gain = 28, ADC_MAX = 4095
vout_mV = (adc_data × 3000 × 28) / 4095
```

- Registered pipeline: multiply first, divide on the next cycle
- `vout_valid` asserts for one cycle when result is ready

### `control_tick_gen`

```verilog
// CLK_FREQ = 100_000_000, CTRL_FREQ = 10_000
// DIV = 10000 → one-cycle pulse every 100 µs
```

### `fopi_controller_55V`

- Reference: 55000 mV (hardwired as `vref_mV = 16'd55000`)
- Operating point initialised to `acc = 800`, `duty = 800`
- Anti-windup via saturation clamp: `DUTY_MIN = 300`, `DUTY_MAX = 950`
- Control runs only on `control_tick`; pure hold state otherwise

### `pwm_deadtime_2mosfet`

```
pwm_high = 1  when  pwm_cnt < (duty - DEAD_TIME)
pwm_low  = 1  when  pwm_cnt > (duty + DEAD_TIME)
```

- `PWM_PERIOD = 1000` counts at 50 MHz → 50 kHz switching
- `DEAD_TIME = 20` counts → 400 ns dead-time between transitions

---

## Repository Structure

```
.
├── RTL/
│   ├── top_fopi_pwm_system.v       # Top-level
│   ├── voltage_scale.v             # ADC → mV conversion
│   ├── control_tick_gen.v          # 10 kHz tick generator
│   ├── fopi_controller_55V.v       # FOPI control law
│   ├── duty_latch.v                # Control-rate duty hold
│   └── pwm_deadtime_2mosfet.v      # 50 kHz PWM + dead-time
├── SIMULINK/
│   ├── fopi_tuning.m               # Iso-damping parameter derivation
│   └── converter_sim.slx           # MATLAB/Simulink plant model
├── PCB/
│   └── PCB Design Files
|
└── README.md
```

---

## FPGA Target: DE10-Standard (Cyclone V)

| Resource | Usage |
|---|---|
| FPGA Board | Terasic DE10-Standard |
| Device | Intel Cyclone V 5CSXFC6D6F31C6 |
| System Clock | 50 MHz on-board (×2 PLL → 100 MHz) |
| ADC Interface | SPI (12-bit, channel-select command) |
| PWM Outputs | GPIO header pins (QH, QL) |
| Tool | Intel Quartus Prime |

### Pin Assignments (example)

| Signal | DE10-Standard Pin |
|---|---|
| `clk` | PIN_AF14 (50 MHz OSC) |
| `rst` | KEY[0] |
| `adc_data[11:0]` | GPIO (SPI MISO path) |
| `adc_ready` | GPIO |
| `pwm_high` | GPIO_0[0] |
| `pwm_low` | GPIO_0[1] |

> Update `fopi_converter.qsf` for your specific wiring before compiling.

---

## Simulation & Verification

### Testbench coverage

- Reset behaviour and startup from zero voltage
- ADC SPI data transfer correctness
- Voltage scaling accuracy
- PWM frequency (50 kHz) and duty cycle accuracy
- Startup transient (0 → 55 V)
- Sudden load step disturbance
- Input voltage disturbance (30 V ↔ 400 V sweep)
- Controller response direction (sign check)
- Duty cycle saturation at `DUTY_MIN` / `DUTY_MAX`

### Running simulation (ModelSim / Questa)

```bash
vlib work
vlog rtl/*.v tb/top_fopi_pwm_tb.v
vsim -novopt work.top_fopi_pwm_tb
add wave /*
run -all
```

### Quartus compilation

```bash
quartus_sh --flow compile quartus/fopi_converter.qpf
quartus_pgm -m jtag -o "p;quartus/output_files/fopi_converter.sof"
```

---

## Hardware Results

- **Measured output voltage:** ~54.9 V DC (multimeter verified)
- **PWM waveform:** 50 kHz, ~82.5% duty cycle observed on oscilloscope
- **Switching transients:** Minor ringing at transitions due to parasitic inductance — expected and within bounds
- **Voltage regulation:** Stable across 30–400 V input sweep

---


## Converter Design Equations

**Duty cycle:**
```
D_buck  = Vout / Vin
D_boost = (Vout - Vin) / Vin
```

**Inductor (per phase):**
```
L = (Vin × D) / (ΔiL × fsw)  →  0.4 mH
```

**Output capacitor:**
```
C = (Iout × D) / (fsw × ΔVout)  →  363 µF
```

**Ripple budget:** ΔiL = 2.727 A, ΔVout = 0.55 V (1% of Vout)

---

## References

1. Yousaf Haroon, Amjadullah Khattak — "Design and Analysis of FOCS for Buck Converter PV Emulator," *GSJ*, Vol. 8, Feb 2020.
2. Mahmoud F. Mahmoud et al. — "Different Approximation Techniques for a FOPID," *ICM 2022*.
3. Maximiliano Bueno-Lopez, Eduardo Giraldo — "Real-Time Fractional Order PI for Embedded Control of a Synchronous Buck Converter," *Engineering Letters*, Vol. 29(3), Sep 2021.
4. S. Vijayalakshmi et al. — "Modeling and Simulation of Interleaved Buck Boost Converter with PID Controller," *IEEE ISCO 2015*.
5. Manjusha Silas, Surekha Bhusnur — "Optimal Fractional Order Controller Design for a DC Buck Converter," *EVERGREEN*, Vol. 11(2), Jun 2024.
6. Cihan Ersali, Goran Hekimoglu — "FOPID Controller Design for a Buck Converter Using Hybrid Cooperation Search Algorithm," *GU J Sci Part A*, 10(4), 2023.
7. G. Prithivi, P. Madasamy — "Parallel Connected Buck-Boost Converter for PV Application Using PI Controller," *IJAREEIE*, Vol. 7(2), Feb 2018.
8. N H Baharudin et al. — "Performance Analysis of DC-DC Buck Converter for Renewable Energy Application," *IOP Conf. Series*, 2018.
9. H. Shayeghi et al. — "A Buck-Boost Converter; Design, Analysis and Implementation for Renewable Energy Systems," *Iranian J. EEE*, 2021.
10. Seyyed Morteza Ghamari et al. — "Improved Robust Fractional-Order PID Controller for Buck-Boost Converter using Snake Optimization Algorithm," *IET Control Theory & Applications*, Feb 2025.

---

## License

This project is submitted as an academic capstone at VIT Chennai. Reuse with attribution.
