# ESP32 MicroPython Controller with Dijkstra's Algorithm for Webots HIL
# Albert Jestin Sutedja / 466092 Hanze
# Code designed by Albert + Inspo & Assistance from Simon
# ESP Connection design from Simon

# Step-by-step
# 1. Run ESP Code
# 2. Get ESP IP Address (Shell)
# 3. Edit webots code

import network
import socket
import json
import time
import gc
from machine import Pin
import math

# --- Configs ---
# WiFi Configuration
WIFI_SSID = 'KPNF91B36' # WiFi SSID
WIFI_PASSWORD = 'vJhc7vJjPxJXMqxr' # WiFi password
SERVER_PORT = 8080

# Onboard LED
led = Pin(2, Pin.OUT) # ESP32 onboard LED, usually GPIO2

# Grid Configuration (Must match Webots)
GRID_ROWS = 15
GRID_COLS = 21
# 0 = BLACK LINE (pathable)
# 1 = WHITE SPACE (obstacle)
grid_map = [
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,0,1,0,1,0],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,0,1,0,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0],
    [0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0],
    [0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0],
    [0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0],
    [0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0],
    [0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,1,0,1,0,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [0,1,0,1,0,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
]

# Path Planning
current_robot_grid_pos_actual = None
current_robot_grid_pos_path = None
goal_grid_pos = None
planned_path = []
current_path_index = 0
path_needs_replan = True
last_replan_time = 0
REPLAN_INTERVAL_MS = 1000
# action_if_dijkstra_fails variable removed

# Dijkstra Algorithm
class SimplePriorityQueue:
    def __init__(self):
        self._queue = []
    def put(self, item, priority):
        self._queue.append({'item': item, 'priority': priority})
        self._queue.sort(key=lambda x: x['priority'])
    def get(self):
        if not self.is_empty():
            return self._queue.pop(0)['item']
        return None
    def is_empty(self):
        return len(self._queue) == 0

def get_valid_neighbors(r, c, rows, cols, grid):
    neighbors = []
    for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
        nr, nc = r + dr, c + dc
        if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0:
            neighbors.append(((nr, nc), 1))
    return neighbors

def dijkstra(grid, start_node, end_node):
    rows, cols = len(grid), len(grid[0])
    if not (0 <= start_node[0] < rows and 0 <= start_node[1] < cols and grid[start_node[0]][start_node[1]] == 0):
        print(f"Dijkstra Error: Start node {start_node} is invalid or not on a pathable line.")
        return []
    if not (0 <= end_node[0] < rows and 0 <= end_node[1] < cols and grid[end_node[0]][end_node[1]] == 0):
        print(f"Dijkstra Error: End node {end_node} is invalid or not on a pathable line.")
        return []

    pq = SimplePriorityQueue()
    pq.put(start_node, 0)
    came_from = {start_node: None}
    cost_so_far = {start_node: 0}
    path_found = False
    nodes_explored_count = 0

    while not pq.is_empty():
        current_node = pq.get()
        nodes_explored_count +=1
        if current_node == end_node:
            path_found = True
            break
        for next_node, cost in get_valid_neighbors(current_node[0], current_node[1], rows, cols, grid):
            new_cost = cost_so_far[current_node] + cost
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost
                pq.put(next_node, priority)
                came_from[next_node] = current_node # *** CORRECTED TYPO HERE ***
    
    if not path_found:
        print(f"Dijkstra: No path found from {start_node} to {end_node} after exploring {nodes_explored_count} nodes.")
        return []

    path = []
    node = end_node
    while node is not None:
        path.append(node)
        node = came_from.get(node)
    path.reverse()
    
    if not path or path[0] != start_node : # Added 'not path' for safety
        print(f"Dijkstra WARNING: Path issue. Starts at {path[0] if path else 'N/A'}, expected {start_node}")
        return []
    # print(f"Dijkstra: Path from {start_node} to {end_node} has {len(path)} steps (explored {nodes_explored_count}).") # Optional: for debugging
    return path

# WiFi Connection
def connect_wifi(ssid, password):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print(f'Attempting to connect to WiFi SSID: {ssid}')
        wlan.connect(ssid, password)
        timeout = 10
        while not wlan.isconnected() and timeout > 0:
            # print('.', end='') # Optional: for visual feedback during connection
            # led.value(not led.value()) # Optional: for visual feedback
            time.sleep(1)
            timeout -= 1
    if wlan.isconnected():
        led.on()
        print(f'\nWiFi Connected! IP address: {wlan.ifconfig()[0]}')
        return wlan
    else:
        led.off()
        print('\nWiFi Connection Failed.')
        return None

# Server Setup
def start_server(port):
    addr = socket.getaddrinfo('0.0.0.0', port)[0][-1]
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(addr)
    s.listen(1)
    print(f'ESP32 server listening on port {port}')
    return s

# Main Logic to Determine Action based on Path
def get_action_from_path(robot_pos_on_path, world_theta_rad, webots_line_sensors_binary):
    global planned_path, current_path_index, goal_grid_pos

    if not planned_path or not robot_pos_on_path:
        return 'stop', robot_pos_on_path
    if robot_pos_on_path == goal_grid_pos:
        return 'stop', robot_pos_on_path
    if not (0 <= current_path_index < len(planned_path) -1 ):
        if current_path_index == len(planned_path) -1 and robot_pos_on_path == planned_path[-1]:
             return 'stop', robot_pos_on_path
        print(f"WARN: Path index {current_path_index} out of bounds for path length {len(planned_path)} or already at goal. Robot@Path: {robot_pos_on_path}")
        return 'stop', robot_pos_on_path

    current_node_on_path = planned_path[current_path_index]
    next_node_on_path = planned_path[current_path_index + 1]

    if robot_pos_on_path != current_node_on_path:
        print(f"WARN: Robot's path position {robot_pos_on_path} differs from indexed path node {current_node_on_path}. Re-aligning index.")
        try:
            current_path_index = planned_path.index(robot_pos_on_path, current_path_index)
            current_node_on_path = planned_path[current_path_index]
            if current_path_index >= len(planned_path) -1:
                 return 'stop', robot_pos_on_path
            next_node_on_path = planned_path[current_path_index + 1]
        except ValueError:
            print(f"ERROR: Robot pos {robot_pos_on_path} not found in remaining path. Stopping. Replan needed.")
            return 'stop', robot_pos_on_path

    dr = next_node_on_path[0] - current_node_on_path[0]
    dc = next_node_on_path[1] - current_node_on_path[1]

    target_theta_rad = None
    if dc == 1 and dr == 0: target_theta_rad = 0.0
    elif dc == -1 and dr == 0: target_theta_rad = math.pi
    elif dr == 1 and dc == 0: target_theta_rad = math.pi / 2.0
    elif dr == -1 and dc == 0: target_theta_rad = -math.pi / 2.0
    else:
        print(f"WARN: Non-adjacent nodes in path? {current_node_on_path} -> {next_node_on_path}")
        return 'stop', current_node_on_path

    current_theta_norm = math.atan2(math.sin(world_theta_rad), math.cos(world_theta_rad))
    angle_diff = target_theta_rad - current_theta_norm
    angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

    ANGLE_THRESHOLD_RAD = math.radians(40)

    if abs(angle_diff) > ANGLE_THRESHOLD_RAD:
        return 'turn_left' if angle_diff > 0 else 'turn_right', current_node_on_path
    else:
        return 'forward', current_node_on_path

# --- Main Program ---
if __name__ == "__main__":
    wlan = connect_wifi(WIFI_SSID, WIFI_PASSWORD)
    
    if not wlan or not wlan.isconnected():
        print("Stopping. No WiFi.")
    else:
        server_socket = start_server(SERVER_PORT)
        conn = None

        while True:
            gc.collect()
            current_time_ms = time.ticks_ms()
            # action_if_dijkstra_fails variable removed and its reset here is removed.

            if conn is None:
                print("Waiting for Webots connection...")
                led.off()
                try:
                    server_socket.settimeout(1.0)
                    conn, addr = server_socket.accept()
                    conn.settimeout(0.1)
                    print(f"Connected by Webots: {addr}")
                    led.on()
                    path_needs_replan = True
                except OSError as e:
                    if e.args[0] == 116: # ETIMEDOUT - For server_socket.accept()
                        pass 
                    else:
                        print(f"Accept error: {e}") 
                        conn = None 
                    time.sleep(0.5) 
                    continue 
                except Exception as e:
                    print(f"Unexpected accept error: {e}")
                    conn = None
                    time.sleep(1)
                    continue

            try:
                data_bytes = conn.recv(512)
                if data_bytes:
                    data_str = data_bytes.decode('utf-8').strip()
                    
                    for msg_part in data_str.split('\n'):
                        if not msg_part.strip(): continue
                        try:
                            webots_data = json.loads(msg_part)

                            if webots_data.get('type') == 'webots_status':
                                new_robot_pos_actual = tuple(webots_data.get('robot_grid_pos'))
                                new_goal_pos_from_webots = tuple(webots_data.get('goal_grid_pos'))
                                world_pose_data = webots_data.get('world_pose', {})
                                robot_theta_rad_from_webots = world_pose_data.get('theta_rad', 0.0)
                                line_sensors_binary_from_webots = webots_data.get('sensors_binary', [0,0,0])
                                
                                obstacles_from_webots = webots_data.get('detected_obstacles', [])
                                map_updated_by_obstacles = False
                                if obstacles_from_webots:
                                    for obs_coord_list in obstacles_from_webots:
                                        if isinstance(obs_coord_list, list) and len(obs_coord_list) == 2:
                                            obs_row, obs_col = obs_coord_list[0], obs_coord_list[1]
                                            if 0 <= obs_row < GRID_ROWS and 0 <= obs_col < GRID_COLS:
                                                if grid_map[obs_row][obs_col] == 0:
                                                    grid_map[obs_row][obs_col] = 1
                                                    map_updated_by_obstacles = True
                                                    print(f"ESP Map Updated: Obstacle added at ({obs_row}, {obs_col})")
                                
                                if map_updated_by_obstacles:
                                    path_needs_replan = True
                                    print("ESP: Grid map updated, forcing replan.")

                                if new_robot_pos_actual != current_robot_grid_pos_actual:
                                    current_robot_grid_pos_actual = new_robot_pos_actual
                                    if current_robot_grid_pos_path and \
                                       (abs(current_robot_grid_pos_actual[0] - current_robot_grid_pos_path[0]) > 1 or \
                                        abs(current_robot_grid_pos_actual[1] - current_robot_grid_pos_path[1]) > 1) :
                                        path_needs_replan = True
                                    if current_robot_grid_pos_path is None:
                                        current_robot_grid_pos_path = current_robot_grid_pos_actual

                                if new_goal_pos_from_webots != goal_grid_pos:
                                    goal_grid_pos = new_goal_pos_from_webots
                                    path_needs_replan = True
                                    print(f"New goal received from Webots: {goal_grid_pos}")
                                
                                if current_robot_grid_pos_actual is None: current_robot_grid_pos_actual = new_robot_pos_actual
                                if current_robot_grid_pos_path is None: current_robot_grid_pos_path = new_robot_pos_actual
                                if goal_grid_pos is None: goal_grid_pos = new_goal_pos_from_webots; path_needs_replan = True
                                
                                if path_needs_replan or (time.ticks_diff(current_time_ms, last_replan_time) > REPLAN_INTERVAL_MS):
                                    if current_robot_grid_pos_actual and goal_grid_pos:
                                        gc.collect()
                                        new_path = dijkstra(grid_map, current_robot_grid_pos_actual, goal_grid_pos)
                                        gc.collect()
                                        
                                        if new_path:
                                            planned_path = new_path
                                            current_path_index = 0
                                            if planned_path[0] == current_robot_grid_pos_actual:
                                                current_robot_grid_pos_path = planned_path[0]
                                            else:
                                                print(f"WARN: Dijkstra path starts at {planned_path[0]}, but robot is at {current_robot_grid_pos_actual}.")
                                                try:
                                                    current_path_index = planned_path.index(current_robot_grid_pos_actual)
                                                    current_robot_grid_pos_path = current_robot_grid_pos_actual
                                                except ValueError:
                                                    current_robot_grid_pos_path = planned_path[0]
                                                    current_path_index = 0
                                            path_needs_replan = False
                                            last_replan_time = current_time_ms
                                        else:
                                            print("ESP: Dijkstra failed to generate new path.") 
                                            planned_path = [] 
                                            path_needs_replan = True 
                                    else:
                                        print("Cannot replan: robot current position or goal position is unknown.")
                                
                                action_to_send = 'stop' 
                                
                                if planned_path and current_robot_grid_pos_path and goal_grid_pos: 
                                    action_to_send, _ = get_action_from_path(
                                        current_robot_grid_pos_path,
                                        robot_theta_rad_from_webots,
                                        line_sensors_binary_from_webots
                                    )
                                    
                                    if current_path_index < len(planned_path) - 1:
                                        prospective_next_node_on_path = planned_path[current_path_index + 1]
                                        if current_robot_grid_pos_actual == prospective_next_node_on_path:
                                            current_path_index += 1
                                            current_robot_grid_pos_path = prospective_next_node_on_path

                                elif current_robot_grid_pos_actual == goal_grid_pos: 
                                    action_to_send = 'stop'
                                    print("🎉 Goal Reached (actual pos matches goal)! Sending STOP.")
                                    planned_path = [] 
                                    path_needs_replan = False
                                elif not planned_path: # If no path, and not at goal, ensure it stops and prints a message
                                    action_to_send = 'stop'
                                    print("ESP: No path. Sending STOP.")


                                command_to_webots = {
                                    'type': 'esp32_command',
                                    'action': action_to_send,
                                    'path': planned_path,
                                    'robot_pos_on_path_esp_thinks': list(current_robot_grid_pos_path) if current_robot_grid_pos_path else None,
                                    'current_path_idx_esp': current_path_index
                                }
                                response_json = json.dumps(command_to_webots) + '\n'
                                conn.sendall(response_json.encode('utf-8'))

                        except json.JSONDecodeError as e:
                            print(f"JSON Decode Error from Webots: '{msg_part}', Error: {e}")
                        except Exception as e:
                            print(f"Error processing Webots message: {e} (Data: '{data_str[:100]}')")
                else:
                    print("Webots disconnected (received empty data).")
                    conn.close()
                    conn = None
                    led.off()
                    current_robot_grid_pos_actual = None
                    current_robot_grid_pos_path = None
                    planned_path = []
                    path_needs_replan = True

            except OSError as e:
                if e.args[0] == 116: # ETIMEDOUT on recv
                    pass 
                elif e.args[0] == 104: # ECONNRESET 
                    print("Webots connection reset by peer.")
                    if conn: conn.close()
                    conn = None
                    led.off()
                else: 
                    print(f"Socket recv/send OS error: {e}")
                    if conn: conn.close()
                    conn = None 
                    led.off()
            except Exception as e: 
                print(f"Main communication loop error: {e}")
                if conn: conn.close()
                conn = None 
                led.off()
                time.sleep(1) 

            time.sleep(0.02)


