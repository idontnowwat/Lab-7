# ESP32-Webots Lab 7 Robot Navigation

This project demonstrates real-time robot navigation in a Webots simulation using an ESP32 microcontroller running MicroPython. The ESP32 performs Dijkstra pathfinding and communicates with the Webots simulation, which controls an e-puck robot. The system supports dynamic obstacle detection and visualization.

## Dependencies

### Webots Controller (`webots_Lab7.py`)
- **Python 3.7+** (recommended: 3.8 or 3.9)
- **Webots** (robot simulator, https://cyberbotics.com/)
- **pip packages:**
  - `matplotlib`
  - `numpy` (if used elsewhere)
- **Webots Python API:** Provided by Webots installation

### ESP32 MicroPython Controller (`# Thonny.py`)
- **ESP32 board** with MicroPython firmware (https://micropython.org/download/esp32/)
- **MicroPython packages:**
  - `network`
  - `socket`
  - `machine`
  - `gc`
  - `math`
  - `json`
  - `time`

## Setup and Running the Experiment

### 1. Flash ESP32 with MicroPython
- Download and flash the latest MicroPython firmware to your ESP32.
- Use a tool like `esptool.py` for flashing.

### 2. Upload ESP32 Script
- Upload [`# Thonny.py`] to the ESP32 using `ampy`, `mpremote`, or Thonny.
- Edit the WiFi credentials (`WIFI_SSID`, `WIFI_PASSWORD`) in the script to match your network.
- Reset or reboot the ESP32 to start the script.

### 3. Prepare Webots Simulation
- Open Webots and load your e-puck world.
- Ensure the robot controller is set to [`webots_Lab7.py`](webots_Lab7.py).
- Install required Python packages:
  ```sh
  pip install matplotlib
  ```
- Adjust `ESP32_IP_ADDRESS` in [`webots_Lab7.py`](webots_Lab7.py) to match the ESP32's IP (printed on ESP32 boot).

### 4. Run the Experiment
- Start the ESP32 script and wait for "ESP32 server listening on port 8080".
- Run the Webots simulation. The controller will attempt to connect to the ESP32.
- The robot will follow the planned path, avoid obstacles, and visualize its progress in real time.

## Reproducing Results

- The robot starts at a predefined grid position and navigates to the goal using Dijkstra pathfinding.
- Obstacles detected by the robot are reported to the ESP32, which updates its map and replans as needed.
- The main result is the robot's ability to reach the goal while dynamically avoiding obstacles, as visualized in the matplotlib window.

## Notes

- Ensure both devices are on the same WiFi network.
- If you change the grid map, update it in both scripts.
- For troubleshooting, check the console output of both the ESP32 and Webots for connection and pathfinding status.

---
**Authors:**  
Freelo Kamara 458509
