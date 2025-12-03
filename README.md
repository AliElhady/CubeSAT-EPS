# CubeSAT-EPS
CubeSat Electrical Power System (EPS)

This project is a custom Electrical Power System for a CubeSat-class satellite, designed from scratch around a 2-cell Li-ion battery pack, solar input, and an STM32G474 microcontroller for smart power management and telemetry.

The EPS handles everything from harvesting solar energy, to charging and protecting the battery, to distributing clean, regulated rails to the rest of the satellite and streaming live power data to a ground station over LoRa.

Key Features

⚡ Solar Energy Harvesting & Battery Charging

MPPT-based solar charger (BQ24650 class) for 2S Li-ion pack
Supports simultaneous battery charging and system load powering
Automatic priority: uses solar power when available, falls back to battery when not

🔋 Battery Management & Protection

2-cell Li-ion battery pack with BMS for over-charge, over-discharge and over-current protection
Ideal-diode & reverse-polarity MOSFET circuitry on the battery input
TVS + LC filtering on critical power rails

📏 Power Monitoring & Telemetry

Multiple INA226 current/voltage monitors on battery, solar input, and main rails
Real-time telemetry sent via DL-SX1278PA LoRa module to a ground station
Designed for logging battery voltage, current, power, and solar harvesting performance

🧠 Control & Housekeeping

STM32G474RE microcontroller for EPS control, data acquisition, and LoRa communication
5 V and 3.3 V regulated rails for payload, OBC, and communication subsystems
Fault monitoring hooks for future safe-mode / load-shedding logic

🛠️ Hardware-Focused Design

KiCad-based 4-layer PCB (EPS + LoRa + sensing on a single board)

U.FL RF connector with optional π-network pads for antenna matching

Designed as a teaching/learning platform for small-sat power electronics
