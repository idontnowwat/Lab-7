# Lab-7

This repository contains the Python and MicroPython code used for a Webots robot simulation with an ESP32 microcontroller.

## Dependencies

- **Python 3.10+** with pip
- **Webots** (2023 or newer) including the Python controller API
- **Matplotlib** (`pip install matplotlib`)
- **MicroPython** firmware flashed onto an ESP32

## Reproducing the Simulation

1. Flash `Thonny.py` to the ESP32 using your preferred MicroPython IDE (e.g. Thonny).
2. Obtain the ESP32 IP address and update `ESP32_IP_ADDRESS` in `Webots_Lab7.py` accordingly.
3. Launch Webots and load your world with a robot controller referencing `Webots_Lab7.py`.
4. Run the Webots simulation. The controller will connect to the ESP32 and plan a path on the grid using Dijkstra's algorithm.

The robot drives toward the configured goal cell while visualizing its path and detected obstacles.
