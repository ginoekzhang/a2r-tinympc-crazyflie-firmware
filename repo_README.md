# A2R TinyMPC Crazyflie Firmware Integration

## Overview
This repository demonstrates a research effort to integrate a high-level TinyMPC (ADMM) algorithm with firmware updates for the Crazyflie drone. By decoupling the control algorithms and parameters from the firmware, this project aims to future-proof the codebase, enabling seamless updates to the drone's firmware without frequent changes to the control logic.

## Repository Structure
- **`TinyMPC @ a5f8637`**: Contains the high-level implementation of the TinyMPC ADMM algorithm for real-time model predictive control.
- **`crazyflie-firmware @ 5642ad5`**: A lightweight wrapper around the Crazyflie drone firmware, enabling live updates while maintaining compatibility with the control algorithms.
- Repository was built around existing working version for TinyMPC 

## Key Features
1. **High-Level Control Algorithms**: The TinyMPC algorithm is designed for efficient real-time control and operates independently of the drone firmware.
2. **Firmware Decoupling**: The system architecture allows pulling live updates from Crazyflie without altering control logic, ensuring adaptability to future firmware changes.
3. **Seamless Integration**: Custom wrappers handle communication between the control logic and drone firmware, minimizing manual intervention.

## Getting Started
### Prerequisites
- Crazyflie drone and compatible hardware
- Python 3 (latest version preferred)
- GCC for compiling the Crazyflie firmware (works easiest on Linux-based systems)

### Installation
1. Clone this repository and its submodules:
   ```bash
   git clone --recurse-submodules <repository-url>
   ```
2. 
3. 

### Running the System
1. 
   ```
2. 

## Updating Firmware
To fetch the latest Crazyflie firmware updates without affecting the control algorithms:
1. Pull updates in the `crazyflie-firmware` submodule:
   ```bash
   git submodule update --remote crazyflie-firmware
   ```
2. Recompile and flash the firmware:
   ```bash
   make && make flash
   ```

## Future Work
- Enhance compatibility with other drones and robotic platforms.
- Add support for additional optimization methods and control algorithims beyond TinyMPC.
- Investigate real-time parameter tuning to further reduce control algorithm modifications.

## Acknowledgments
- **Crazyflie (Bitcraze)**
- **TinyMPC**
- **The Barnard Accessible and Accelerated Robotics Lab**


## License
PENDING
