# VibroSense ASIC — Industrial Vibration Anomaly Detector
### chipIgnite Reference Design Contest — April 2026
---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Problem Statement](#2-problem-statement)
3. [Why Custom Silicon](#3-why-custom-silicon)
4. [System Architecture](#4-system-architecture)

---

## 1. Summary

**VibroSense** is a fully custom ASIC that implements an always-on hardware vibration anomaly detector for industrial machinery — motors, pumps, compressors, conveyors, and rotating equipment. The chip accepts raw vibration data from a MEMS accelerometer over SPI, performs a 256-point fixed-point FFT entirely in silicon, compares energy in configurable frequency bins against programmed bearing-fault thresholds, and autonomously triggers a hardware alarm output — all without requiring the host processor to be active.
---

## 2. Problem Statement

### The Industrial Reality

Rotating machinery failure — specifically bearing failure — is the #1 cause of unplanned downtime in manufacturing, oil & gas, and water treatment facilities. According to industry data, bearing failures account for **45–55% of all motor failures**, and a single unplanned stoppage in a continuous process plant costs on average **$260,000 per hour** (ARC Advisory Group, 2023).

### Current Solutions and Their Failures

**Wired vibration monitoring systems** (SKF Multilog, Emerson AMS 9420) cost $800–$2,500 per sensor node, require installation by trained technicians, and depend on wired infrastructure that doesn't exist in retrofit scenarios.

**Cloud-connected wireless sensors** (Samsara, Augury, SparkCognition) solve the installation problem but introduce critical dependencies:
- Continuous WiFi/LTE connectivity on the factory floor (often unreliable near heavy machinery)
- Raw data streaming that consumes 50–200× more battery than edge-processing
- Cloud latency of 10–60 seconds before an alert fires — far too slow for fast-degrading faults
- Subscription costs of $50–$200/month per machine
- GDPR and data sovereignty concerns in EU manufacturing

**Generic MCU-based edge solutions** (ESP32 + FFT library) are limited by:
- The CPU must be continuously active to acquire, buffer, and process accelerometer data
- Software FFT at 3.2kHz sampling on an ESP32 consumes ~18mA continuously — 3–5× more than a purpose-built hardware engine
- No hardware fault output — a software crash means silent failure
- Interrupt latency means the CPU can miss events during other processing

### The Gap VibroSense Fills

A low-cost (<$8 BOM), self-contained, silicon-native vibration intelligence IC that:
- Runs FFT and fault detection **entirely in hardware**, with the CPU off
- Fires a **hardware GPIO alarm** with sub-millisecond latency on fault detection
- Operates continuously at **<2mA** from a CR2032 or small LiPo
- Works **with zero cloud connectivity** — the decision logic is in silicon, not in a server
- Costs **60–80× less** than professional monitoring systems per node

---
## 4. System Architecture

### System-Level Block Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        VibroSense System                                │
│                                                                         │
│  ┌──────────────┐    SPI     ┌────────────────────────────────────────┐ │
│  │  MEMS Accel  │──────────▶│          VibroSense ASIC               │ │
│  │ ADXL345/     │  3.2kHz   │        (Caravel + User Project)        │ │
│  │ ICM-42688    │           │                                         │ │
│  └──────────────┘           │  ┌──────────────────────────────────┐  │ │
│                             │  │     Caravel Management SoC       │  │ │
│  ┌──────────────┐    GPIO   │  │   RISC-V (PicoRV32) @ 10MHz     │  │ │
│  │  Status LEDs │◀──────────│  │   Flash via SPI                  │  │ │
│  │  NORMAL /    │           │  │   Wishbone Master                │  │ │
│  │  WARN / ALARM│           │  └─────────────┬────────────────────┘  │ │
│  └──────────────┘           │                │ Wishbone Bus           │ │
│                             │  ┌─────────────▼────────────────────┐  │ │
│  ┌──────────────┐  UART/SPI │  │       User Project Area          │  │ │
│  │  Host MCU or │◀──────────│  │                                  │  │ │
│  │  Raspberry Pi│           │  │  ┌──────────┐  ┌─────────────┐  │  │ │
│  │  (optional)  │           │  │  │SPI Accel │  │  Wishbone   │  │  │ │
│  └──────────────┘           │  │  │Interface │  │  Slave RegF │  │  │ │
│                             │  │  └────┬─────┘  └──────┬──────┘  │  │ │
│  ┌──────────────┐   HW GPIO │  │       │                │         │  │ │
│  │  Relay/Buzzer│◀──────────│  │  ┌────▼────────────────▼──────┐  │  │ │
│  │  or PLC Input│  ALARM_N  │  │  │   Sample Buffer (SRAM)     │  │  │ │
│  └──────────────┘           │  │  │   256 × 16-bit × 3 axis    │  │  │ │
│                             │  │  └────────────┬───────────────┘  │  │ │
│  ┌──────────────┐           │  │               │                   │  │ │
│  │  OLED Display│  SPI      │  │  ┌────────────▼───────────────┐  │  │ │
│  │  Spectrum /  │◀──────────│  │  │  256-pt Fixed-Point FFT    │  │  │ │
│  │  Status View │           │  │  │  Engine (Radix-2 DIT)      │  │  │ │
│  └──────────────┘           │  │  │  3 Parallel Pipelines      │  │  │ │
│                             │  │  └────────────┬───────────────┘  │  │ │
│                             │  │               │ |Xk|² Magnitudes  │  │ │
│                             │  │  ┌────────────▼───────────────┐  │  │ │
│                             │  │  │  Fault Frequency Detector  │  │  │ │
│                             │  │  │  8× Bin-Energy Accumulators│  │  │ │
│                             │  │  │  Threshold Comparators     │  │  │ │
│                             │  │  └────────────┬───────────────┘  │  │ │
│                             │  │               │                   │  │ │
│                             │  │  ┌────────────▼───────────────┐  │  │ │
│                             │  │  │  Alarm State Machine       │  │  │ │
│                             │  │  │  NORMAL→WARNING→ALARM      │  │  │ │
│                             │  │  │  Hardware ALARM_N GPIO     │  │  │ │
│                             │  │  └────────────────────────────┘  │  │ │
│                             │  └──────────────────────────────────┘  │ │
│                             └────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────┘
```

### Signal Processing Pipeline

```
MEMS Accel (3.2kHz ODR)
        │
        ▼
SPI Interface → 16-bit samples (X, Y, Z)
        │
        ▼
256-Sample Ring Buffer (fill trigger: 80ms window)
        │
        ▼
Hanning Window Apply (multiply by pre-stored coefficients)
        │
        ▼
256-pt Radix-2 DIT FFT (16-bit fixed-point, Q1.15)
  → 8 stages × 128 butterfly operations
  → Twiddle factors in ROM (256 × 16-bit)
        │
        ▼
Magnitude Squared: |Xk|² = Re² + Im²
        │
        ▼
Fault Frequency Bin Extractor
  → Bins: BPFO, BPFI, BSF, FTF (programmable ± 2 bins around each)
        │
        ▼
Threshold Comparators (8× configurable via Wishbone)
        │
        ▼
Alarm FSM → ALARM_N GPIO (active-low, open-drain)
```

---
