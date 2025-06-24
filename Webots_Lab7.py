"""
Webots Lab 7 Robot Controller

This script implements a robot controller for the e-puck robot in a Webots simulation environment.
Key features:
- Connects to an ESP32 microcontroller over TCP/IP to receive navigation commands and send robot status.
- Maintains a grid-based map of the environment, including static and dynamically detected obstacles.
- Uses ground sensors for line following and distance sensors for obstacle detection.
- Executes commands such as forward movement, turning, and stopping, with robust logic for reacquiring the line if lost.
- Provides real-time visualization of the robot's position, path, detected obstacles, and grid map using matplotlib.
- Performs odometry to estimate the robot's position and orientation.
- Designed for integration with external path planning and navigation logic running on the ESP32.

Classes:
- GridVisualizer: Handles graphical display of the grid, robot, path, and obstacles.
- WebotsRobotController: Main controller class for robot operation, sensor processing, communication, and control logic.

Usage:
Run this script as the main controller in a Webots simulation. The robot will attempt to connect to the ESP32, follow line paths, avoid obstacles, and visualize its progress.
"""
import socket
import time
import math
import matplotlib.pyplot as plt
import json
from controller import Robot, DistanceSensor, Motor

# --- Constants ---#
ESP32_IP_ADDRESS = "192.168.2.22"  # IP address of the ESP32 microcontroller
ESP32_PORT = 8080                  # Port for TCP/IP communication

WHEEL_RADIUS = 0.0205              # Radius of the e-puck's wheels (meters)
AXLE_LENGTH = 0.058                # Distance between the wheels (meters)

GRID_ROWS = 15                     # Number of rows in the grid map
GRID_COLS = 21                     # Number of columns in the grid map
GRID_CELL_SIZE = 0.051             # Size of each grid cell (meters)
GRID_ORIGIN_X = 0.050002           # X coordinate of grid origin (meters)
GRID_ORIGIN_Z = -0.639e-05         # Z coordinate of grid origin (meters)

GOAL_ROW = 14                      # Goal cell row index
GOAL_COL = 0                       # Goal cell column index

FORWARD_SPEED = 3                  # Default forward speed for motors
LINE_THRESHOLD = 600               # Threshold for ground sensor to detect line

DISTANCE_SENSOR_THRESHOLD = 100    # Threshold for distance sensor to detect obstacle
OBSTACLE_DETECTION_ENABLED = True  # Enable/disable obstacle detection

TURN_SPEED_FACTOR = 1              # Multiplier for turn speed
MIN_INITIAL_SPIN_DURATION = 0.35   # Minimum duration for initial spin phase (seconds)
MAX_SEARCH_SPIN_DURATION = 20.0    # Max duration to search for line during turn (seconds)
MAX_ADJUST_DURATION = 5.0          # Max duration for adjustment phase after reacquiring line (seconds)
TURN_ADJUST_BASE_SPEED = FORWARD_SPEED * 0.5  # Base speed for fine adjustment after turn
TURN_UNTIL_LINE_FOUND = True       # Continue turning until line is found

AGGRESSIVE_CORRECTION_DIFFERENTIAL = FORWARD_SPEED * 1.3  # Speed diff for aggressive correction
MODERATE_CORRECTION_DIFFERENTIAL = FORWARD_SPEED * 1.2    # Speed diff for moderate correction

SIMULATED_STATIC_OBSTACLES = [(0, 18)]  # List of static obstacles (row, col) for simulation

# --- Utility Functions ---#
def world_to_grid(world_x, world_z):
    """
    Convert world coordinates (meters) to grid cell indices (row, col).
    Ensures indices are within grid bounds.
    """
    col = round((world_x - GRID_ORIGIN_X) / GRID_CELL_SIZE)
    row = round((world_z - GRID_ORIGIN_Z) / GRID_CELL_SIZE)
    return max(0, min(row, GRID_ROWS - 1)), max(0, min(col, GRID_COLS - 1))

def grid_to_world_center(row, col):
    """
    Convert grid cell indices to the center world coordinates (x, z).
    """
    return GRID_ORIGIN_X + col * GRID_CELL_SIZE, GRID_ORIGIN_Z + row * GRID_CELL_SIZE

def get_line_centered_position(rwp_val, crgp_val, ldf_val):
    """
    Optionally adjust robot's displayed position for visualization
    based on line sensor readings. Currently returns estimated pose.
    """
    return rwp_val['x'], rwp_val['z']

# --- Visualization Class ---
class GridVisualizer:
    """
    Handles graphical display of the grid, robot, path, and obstacles using matplotlib.
    """
    def __init__(self, world_grid, detected_obstacles_grid):
        self.world_grid = world_grid
        self.detected_obstacles_grid = detected_obstacles_grid
        self.fig, self.ax = None, None
        self.robot_trail_world = []  # Stores robot's path for trail visualization

    def setup(self):
        """
        Initialize the matplotlib figure and draw static grid lines and legend.
        """
        plt.style.use('Solarize_Light2')
        self.fig, self.ax = plt.subplots(figsize=(12, 9))
        self.ax.set_aspect('equal')
        self.ax.set_title('Grid Map', fontsize=14, fontweight='bold')
        self.ax.set_xlabel(' X (m)')
        self.ax.set_ylabel(' Y (m)')
        # Draw horizontal grid lines
        for r_idx in range(GRID_ROWS + 1):
            self.ax.plot([GRID_ORIGIN_X, GRID_ORIGIN_X + GRID_COLS * GRID_CELL_SIZE],
                         [GRID_ORIGIN_Z + r_idx * GRID_CELL_SIZE]*2, 'k-', alpha=0.2, lw=0.5)
        # Draw vertical grid lines
        for c_idx in range(GRID_COLS + 1):
            self.ax.plot([GRID_ORIGIN_X + c_idx * GRID_CELL_SIZE]*2,
                         [GRID_ORIGIN_Z, GRID_ORIGIN_Z + GRID_ROWS * GRID_CELL_SIZE], 'k-', alpha=0.2, lw=0.5)
        # Set plot limits with margin
        margin = GRID_CELL_SIZE * 2
        self.ax.set_xlim(GRID_ORIGIN_X - margin, GRID_ORIGIN_X + GRID_COLS * GRID_CELL_SIZE + margin)
        self.ax.set_ylim(GRID_ORIGIN_Z - margin, GRID_ORIGIN_Z + GRID_ROWS * GRID_CELL_SIZE + margin)
        # Add legend for map elements
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(fc='white',label='Pathable'),
            Patch(fc='dimgray',label='Obstacle (Grid)'),
            Patch(fc='orange',label='Detected Obstacle'),
            plt.Line2D([0],[0],color='blue',lw=2,label='Trail'),
            plt.Line2D([0],[0],color='purple',marker='o',ls='--',label='Path (ESP)'),
            plt.Line2D([0],[0],color='gold',marker='o',ls='',label='Robot'),
            plt.Line2D([0],[0],color='limegreen',marker='*',ls='',label='Goal')
        ]
        self.ax.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(1.02,1))
        plt.tight_layout()
        plt.show(block=False)
        plt.pause(0.01)

    def update(self, rwp_val, crgp_val, path_esp, gs_wb, LINE_THRESHOLD):
        """
        Update the visualization with the robot's current state, path, and obstacles.
        """
        if self.fig is None:
            self.setup()
        # Remove previous dynamic patches (cells, robot, etc.)
        for patch_item in self.ax.patches[:]: patch_item.remove()
        # Remove previous dynamic lines (trail, path, etc.)
        num_static_lines = (GRID_ROWS + 1) + (GRID_COLS + 1)
        for line_item in self.ax.lines[num_static_lines:]: line_item.remove()
        # Draw grid cells with color based on occupancy and detection
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                cx, cz = grid_to_world_center(r,c)
                is_det_obs = (r,c) in self.detected_obstacles_grid
                # Orange for detected obstacles, white for pathable, dimgray for static obstacles
                clr, alp = ('orange',0.8) if is_det_obs else (('white',0.8) if self.world_grid[r][c]==0 else ('dimgray',0.5))
                self.ax.add_patch(plt.Rectangle((cx-GRID_CELL_SIZE/2, cz-GRID_CELL_SIZE/2), GRID_CELL_SIZE,GRID_CELL_SIZE, fc=clr,alpha=alp,ec='gray',lw=0.5))
        # Read ground sensors for visualization (may fail if not initialized)
        try:
            raw_gs_vals = [s.getValue() for s in gs_wb]
            line_det_viz = [1 if v < LINE_THRESHOLD else 0 for v in raw_gs_vals]
        except:
            line_det_viz = [0,0,0]
        # Get robot's display position (optionally adjusted by line sensors)
        disp_x,disp_z = get_line_centered_position(rwp_val,crgp_val,line_det_viz)
        # Store robot's trail for path visualization
        self.robot_trail_world.append((disp_x,disp_z))
        if len(self.robot_trail_world)>200: self.robot_trail_world.pop(0)
        # Draw robot's trail
        if len(self.robot_trail_world)>1:
            trail_x,trail_z=zip(*self.robot_trail_world)
            self.ax.plot(trail_x,trail_z,'blue',lw=2,alpha=0.7)
        # Draw planned path from ESP32 if available
        if path_esp and len(path_esp)>1:
            path_w = [grid_to_world_center(r,c) for r,c in path_esp]
            if path_w and len(path_w) > 0: 
                p_x,p_z=zip(*path_w)
                self.ax.plot(p_x,p_z,'purple',lw=2,ms=5,alpha=0.8,linestyle='--')
                if p_x:
                    self.ax.plot(p_x[0], p_z[0], marker='^', color='purple', ms=8)   # Start of path
                    self.ax.plot(p_x[-1], p_z[-1], marker='*', color='purple', ms=8) # End of path
        # Draw robot as a gold circle with heading arrow
        self.ax.plot(disp_x,disp_z,'o',ms=10,mec='goldenrod',mew=2, color='gold')
        arr_len=GRID_CELL_SIZE*0.7
        dx,dz = arr_len*math.cos(rwp_val['theta']), arr_len*math.sin(rwp_val['theta'])
        self.ax.add_patch(plt.matplotlib.patches.FancyArrowPatch((disp_x,disp_z),(disp_x+dx,disp_z+dz),arrowstyle='->',mutation_scale=15,color='goldenrod',lw=2))
        # Highlight current grid cell
        if crgp_val:
            cx,cz=grid_to_world_center(crgp_val[0],crgp_val[1])
            sensors_on_viz = any(line_det_viz) 
            hl_clr = 'cyan' if sensors_on_viz else 'magenta'
            self.ax.add_patch(plt.Rectangle((cx-GRID_CELL_SIZE/2,cz-GRID_CELL_SIZE/2),GRID_CELL_SIZE,GRID_CELL_SIZE,ec=hl_clr,facecolor='none',alpha=1.0,lw=3,ls='--'))
        # Draw goal as a green star
        goal_x,goal_z=grid_to_world_center(GOAL_ROW,GOAL_COL)
        self.ax.plot(goal_x,goal_z,'*',ms=15,mec='limegreen',mew=2, color='limegreen')
        # Remove previous info panel if present
        if hasattr(self.ax, 'info_panel_text_obj') and self.ax.info_panel_text_obj in self.ax.texts:
            self.ax.info_panel_text_obj.remove()
        # Display robot/grid info in a panel
        info_txt = (f"Grid:{crgp_val} G:({GOAL_ROW},{GOAL_COL})\nLine:{'ON' if any(line_det_viz) else 'OFF'} {line_det_viz}\n"
                    f"Obs:{len(self.detected_obstacles_grid)}\n"
                    f"X={rwp_val['x']:.2f},Z={rwp_val['z']:.2f},Θ={math.degrees(rwp_val['theta']):.0f}°")
        self.ax.info_panel_text_obj = self.ax.text(0.02,0.98,info_txt,transform=self.ax.transAxes,va='top',fontsize=8,bbox=dict(boxstyle='round,pad=0.4',facecolor='lightblue',alpha=0.8))
        plt.draw()
        plt.pause(0.001)

# --- Main Robot Controller Class ---
class WebotsRobotController:
    """
    Main controller for the e-puck robot in Webots.
    Handles sensor processing, communication, control, and visualization.
    """
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.time_step_sec = self.timestep / 1000.0
        self.world_grid = self.init_world_grid()
        self.detected_obstacles_grid = set()  # Set of (row, col) for detected obstacles
        self.recent_new_obstacles = []        # List of newly detected obstacles to send to ESP32
        self.add_simulated_obstacles()        # Add any static obstacles for simulation
        self.visualizer = GridVisualizer(self.world_grid, self.detected_obstacles_grid)
        self.rwp_estimate = {'x': 0.0, 'z': 0.0, 'theta': 0.0}  # Robot world pose estimate
        # Initialize ground sensors (gs0, gs1, gs2)
        self.gs_wb = [self.robot.getDevice(name) for name in ['gs0', 'gs1', 'gs2']]
        for sensor in self.gs_wb: sensor.enable(self.timestep)
        # Initialize all distance sensors (ps0-ps7)
        self.ps_all = [self.robot.getDevice(f'ps{i}') for i in range(8)]
        for sensor in self.ps_all: sensor.enable(self.timestep)
        # Select sensors for obstacle detection (front, left, right)
        self.distance_sensors_selected = [self.ps_all[0], self.ps_all[7], self.ps_all[5]]
        # Initialize motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))  # Set to velocity control mode
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        # Communication and control state
        self.client_socket = None
        self.is_connected = False
        self.esp32_command = 'stop'
        self.planned_path_grid = []
        # Turn state machine variables
        self.webots_internal_turn_phase = 'NONE'
        self.webots_turn_command_active = None
        self.turn_phase_start_time = 0.0
        self.current_left_speed = 0.0
        self.current_right_speed = 0.0
        # 180 turn due to obstacle avoidance
        self.turn_180_due_to_obstacle = False
        self.turned_for_obstacles = set()
        self.turn_180_phase = 'NONE'
        self.turn_180_start_theta = None
        self.turn_180_start_time = 0.0
        self.init_pose()  # Set initial robot pose

    def init_world_grid(self):
        """
        Initialize the grid map for the environment.
        0 = pathable, 1 = obstacle.
        """
        return [
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

    def add_simulated_obstacles(self):
        """
        Add any static obstacles to the grid and detected set for simulation.
        """
        for obs_row, obs_col in SIMULATED_STATIC_OBSTACLES:
            if 0 <= obs_row < GRID_ROWS and 0 <= obs_col < GRID_COLS:
                self.world_grid[obs_row][obs_col] = 1
                self.detected_obstacles_grid.add((obs_row, obs_col))
                self.recent_new_obstacles.append((obs_row, obs_col))
                print(f"INFO: Simulated static obstacle added at ({obs_row}, {obs_col})")

    def init_pose(self):
        """
        Set the robot's initial pose and grid cell.
        """
        INITIAL_GRID_ROW, INITIAL_GRID_COL = 2, 20
        self.rwp_estimate['x'], self.rwp_estimate['z'] = grid_to_world_center(INITIAL_GRID_ROW, INITIAL_GRID_COL)
        self.rwp_estimate['theta'] = math.pi / 2  # Facing "up" (positive Z)
        self.crgp_estimated = world_to_grid(self.rwp_estimate['x'], self.rwp_estimate['z'])
        print(f"Robot init @ grid {self.crgp_estimated}, est.world ({self.rwp_estimate['x']:.2f},{self.rwp_estimate['z']:.2f}), Θ={math.degrees(self.rwp_estimate['theta']):.0f}°")
        print(f"Goal: ({GOAL_ROW},{GOAL_COL}). Obstacle Det: {'ON' if OBSTACLE_DETECTION_ENABLED else 'OFF'}")
        print("-" * 60)

    def connect_to_esp32(self):
        """
        Attempt to connect to the ESP32 over TCP/IP.
        Sets up the socket and connection state.
        """
        print(f"Connecting to ESP32 ({ESP32_IP_ADDRESS}:{ESP32_PORT})...")
        try:
            if self.client_socket: self.client_socket.close()
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.settimeout(2.0)  # Timeout for initial connect
            self.client_socket.connect((ESP32_IP_ADDRESS, ESP32_PORT))
            self.client_socket.settimeout(0.05)  # Short timeout for non-blocking recv
            self.is_connected = True
            print("✅ ESP32 Connected.")
        except Exception as e:
            print(f"❌ ESP32 Conn Fail: {e}")
            self.is_connected = False
            self.client_socket = None

    def detect_obstacles_from_distance_sensors(self, rwp_val, robot_theta, distance_values):
        if not OBSTACLE_DETECTION_ENABLED: return []
        newly_detected = []
        current_row, current_col = world_to_grid(rwp_val['x'], rwp_val['z'])
        theta_deg_norm = math.degrees(robot_theta % (2 * math.pi))
        if theta_deg_norm < 0: theta_deg_norm += 360
        obstacle_relative_positions = {}
        # Determine which direction the robot is facing and map sensor indices to grid offsets
        if (315 <= theta_deg_norm <= 360) or (0 <= theta_deg_norm <= 45): # RIGHT
            obstacle_relative_positions = {0: (0, 1), 1: (-1, 1), 2: (1, 1)}
        elif 45 < theta_deg_norm <= 135: # DOWN
            obstacle_relative_positions = {0: (1, 0), 1: (1, 1), 2: (1, -1)}
        elif 135 < theta_deg_norm <= 225: # LEFT
            obstacle_relative_positions = {0: (0, -1), 1: (1, -1), 2: (-1, -1)}
        elif 225 < theta_deg_norm <= 315: # UP
            obstacle_relative_positions = {0: (-1, 0), 1: (-1, -1), 2: (-1, 1)}
        for i, dist_val in enumerate(distance_values):
            if dist_val > DISTANCE_SENSOR_THRESHOLD:
                # If sensor detects an obstacle, calculate its grid position relative to robot
                if i in obstacle_relative_positions:
                    dr, dc = obstacle_relative_positions[i]
                    obs_r, obs_c = current_row + dr, current_col + dc
                    if 0 <= obs_r < GRID_ROWS and 0 <= obs_c < GRID_COLS:
                        if (obs_r, obs_c) not in self.detected_obstacles_grid:
                            newly_detected.append((obs_r, obs_c))
                            self.detected_obstacles_grid.add((obs_r, obs_c))
        return newly_detected

    def main_loop(self):
        iteration, last_conn_attempt, last_data_send_time, last_obs_check_time = 0,0,0,0
        while self.robot.step(self.timestep) != -1:
            if iteration == 0:
                self.connect_to_esp32()
                self.visualizer.update(self.rwp_estimate, self.crgp_estimated, self.planned_path_grid, self.gs_wb, LINE_THRESHOLD)
            iteration += 1
            current_sim_time = self.robot.getTime()
            # -------------------------------------------------------
            # --- Sensor Readings ---
            raw_gs_values = [s.getValue() for s in self.gs_wb]
            line_detected = [1 if v < LINE_THRESHOLD else 0 for v in raw_gs_values]
            left_gs, center_gs, right_gs = line_detected
            ps_values_all = [p.getValue() for p in self.ps_all]
            current_dist_sens_vals = [ps_values_all[0], ps_values_all[7], ps_values_all[5]]
            # -------------------------------------------------------
            # --- Odometry ---
            v_left = self.current_left_speed * WHEEL_RADIUS
            v_right = self.current_right_speed * WHEEL_RADIUS
            linear_velocity = (v_left + v_right) / 2.0
            angular_velocity = (v_right - v_left) / AXLE_LENGTH
            # Update robot orientation (theta) using angular velocity
            self.rwp_estimate['theta'] += angular_velocity * self.time_step_sec
            # Normalize theta to [-pi, pi]
            self.rwp_estimate['theta'] = math.atan2(math.sin(self.rwp_estimate['theta']), math.cos(self.rwp_estimate['theta']))
            # Use average theta for more accurate position update
            avg_theta = self.rwp_estimate['theta'] - (angular_velocity * self.time_step_sec / 2.0)
            self.rwp_estimate['x'] += linear_velocity * math.cos(avg_theta) * self.time_step_sec
            self.rwp_estimate['z'] += linear_velocity * math.sin(avg_theta) * self.time_step_sec
            new_crgp_est = world_to_grid(self.rwp_estimate['x'], self.rwp_estimate['z'])
            if new_crgp_est != self.crgp_estimated: self.crgp_estimated = new_crgp_est
            # -------------------------------------------------------
            # --- Obstacle Detection ---
            if current_sim_time - last_obs_check_time > 0.2:
                if OBSTACLE_DETECTION_ENABLED:
                    new_obs = self.detect_obstacles_from_distance_sensors(self.rwp_estimate, self.rwp_estimate['theta'], current_dist_sens_vals)
                    for obs in new_obs:
                        if obs not in self.turned_for_obstacles:
                            self.turn_180_due_to_obstacle = True
                            self.turned_for_obstacles.add(obs)
                            print(f"Triggering 180 turn for new obstacle at {obs}")
                    if new_obs: print(f"🚧 {len(new_obs)} new obstacles by sensors!"); self.recent_new_obstacles.extend(new_obs)
                last_obs_check_time = current_sim_time
            # -------------------------------------------------------
            # --- ESP Connection & Communication ---
            if not self.is_connected:
                # If not connected, periodically attempt to reconnect and stop motors
                if current_sim_time - last_conn_attempt > 3.0: self.connect_to_esp32(); last_conn_attempt = current_sim_time
                self.current_left_speed, self.current_right_speed = 0.0, 0.0
                self.left_motor.setVelocity(0.0); self.right_motor.setVelocity(0.0)
                if iteration % 10 == 0: self.visualizer.update(self.rwp_estimate, self.crgp_estimated, self.planned_path_grid, self.gs_wb, LINE_THRESHOLD)
                continue
            if current_sim_time - last_data_send_time > 0.1:
                try:
                    # Send robot status to ESP32 every 0.1s
                    status_data = {'type':'webots_status','robot_grid_pos':list(self.crgp_estimated), 'goal_grid_pos':[GOAL_ROW,GOAL_COL],
                                   'world_pose':{'x':round(self.rwp_estimate['x'],3),'z':round(self.rwp_estimate['z'],3),'theta_rad':round(self.rwp_estimate['theta'],3)},
                                   'sensors_binary':line_detected, 'detected_obstacles':self.recent_new_obstacles.copy()}
                    self.client_socket.sendall((json.dumps(status_data) + '\n').encode('utf-8'))
                    last_data_send_time = current_sim_time
                    if self.recent_new_obstacles: self.recent_new_obstacles.clear()
                except Exception as e: print(f"Send Err: {e}"); self.is_connected=False; self.client_socket.close(); self.client_socket=None; continue
            try:
                # Try to receive commands from ESP32 (non-blocking)
                resp = self.client_socket.recv(1024)
                if resp:
                    dec_resps = resp.decode('utf-8').strip().split('\n')
                    for msg in dec_resps:
                        if not msg.strip(): continue
                        try:
                            esp_rcv_data = json.loads(msg)
                            if esp_rcv_data.get('type') == 'esp32_command':
                                new_cmd = esp_rcv_data.get('action','stop')
                                # If a turn command is interrupted, reset turn phase
                                if new_cmd!=self.esp32_command and self.esp32_command in ['turn_left','turn_right'] and new_cmd not in ['turn_left','turn_right']:
                                    self.webots_internal_turn_phase = 'NONE'; self.webots_turn_command_active=None
                                self.esp32_command = new_cmd
                                self.planned_path_grid = esp_rcv_data.get('path', self.planned_path_grid)
                        except json.JSONDecodeError as e: print(f"JSON Err ESP: {e} (Msg:'{msg}')")
                        except Exception as e: print(f"Proc ESP Err: {e}")
            except socket.timeout: pass
            except Exception as e: print(f"Recv Err: {e}"); self.is_connected=False; self.client_socket.close(); self.client_socket=None; continue
            # -------------------------------------------------------
            # --- Determine Effective Command ---
            effective_command = self.esp32_command 
            sensors_on_line = any(line_detected)

            # --- 180 TURN LOGIC ---
            if self.turn_180_due_to_obstacle or self.turn_180_phase != 'NONE':
                effective_command = 'turn_180'

            # If a turn just finished, reset turn phase
            if self.esp32_command not in ['turn_left','turn_right'] and self.webots_internal_turn_phase!='NONE':
                self.webots_internal_turn_phase='NONE'
                self.webots_turn_command_active=None
            # If in the middle of a turn, keep executing the turn phase logic
            if self.webots_internal_turn_phase != 'NONE':
                if self.esp32_command == 'stop':
                    effective_command = 'stop'
                    self.webots_internal_turn_phase = 'NONE'
                    self.webots_turn_command_active = None
                else:
                    effective_command = self.webots_turn_command_active
            # If on the line and not turning, always go forward
            elif sensors_on_line and self.esp32_command not in ['turn_left', 'turn_right', 'stop']:
                effective_command = 'forward'
            # If off the line but commanded to go forward, start a left turn to reacquire line
            elif not sensors_on_line and self.esp32_command == 'forward':
                effective_command = 'turn_left'
            # If ESP says stop but robot is off the line, override to turn left to reacquire line
            if self.esp32_command == 'stop' and not sensors_on_line and effective_command == 'stop':
                print(f"INFO: ESP 'stop', off-line. Webots overriding to 'turn_left'.")
                effective_command = 'turn_left'
            # -------------------------------------------------------
            # --- Execute Command & Set Motor Speeds ---
            next_left_speed, next_right_speed = 0.0, 0.0
            if effective_command == 'turn_180':
                # 180-degree turn state machine
                if self.turn_180_phase == 'NONE':
                    self.turn_180_phase = 'INIT'
                    self.turn_180_start_theta = self.rwp_estimate['theta']
                    self.turn_180_start_time = current_sim_time
                    print("Starting 180-degree turn")
                # Calculate angle turned so far
                angle_turned = abs((self.rwp_estimate['theta'] - self.turn_180_start_theta + math.pi) % (2*math.pi) - math.pi)
                # Set turning speeds
                next_left_speed = FORWARD_SPEED
                next_right_speed = -FORWARD_SPEED
                # If turned at least ~175 degrees or timeout, finish
                if angle_turned >= math.radians(175) or (current_sim_time - self.turn_180_start_time) > 5.0:
                    self.turn_180_phase = 'NONE'
                    self.turn_180_due_to_obstacle = False
                    print("Completed 180-degree turn")
                    next_left_speed = 0.0
                    next_right_speed = 0.0
            elif effective_command == 'stop':
                # Stop both motors
                next_left_speed = 0.0
                next_right_speed = 0.0
                self.turn_180_phase = 'NONE'
                self.turn_180_due_to_obstacle = False
            elif effective_command == 'forward':
                self.webots_internal_turn_phase = 'NONE' 
                self.webots_turn_command_active = None
                base_s = FORWARD_SPEED
                # Line following logic: adjust speeds based on which sensors detect the line
                if not left_gs and center_gs and not right_gs: 
                    next_left_speed, next_right_speed = base_s, base_s
                elif left_gs and center_gs and not right_gs: 
                    next_left_speed = base_s - MODERATE_CORRECTION_DIFFERENTIAL
                    next_right_speed = base_s
                elif not left_gs and center_gs and right_gs: 
                    next_left_speed = base_s
                    next_right_speed = base_s - MODERATE_CORRECTION_DIFFERENTIAL
                elif left_gs and not center_gs and not right_gs: 
                    next_left_speed = base_s - AGGRESSIVE_CORRECTION_DIFFERENTIAL
                    next_right_speed = base_s
                elif not left_gs and not center_gs and right_gs: 
                    next_left_speed = base_s
                    next_right_speed = base_s - AGGRESSIVE_CORRECTION_DIFFERENTIAL
                elif left_gs and center_gs and right_gs: 
                    next_left_speed, next_right_speed = base_s * 0.6, base_s * 0.6
                elif not sensors_on_line: 
                    next_left_speed, next_right_speed = base_s * 0.2, base_s * 0.2
                else: 
                    next_left_speed, next_right_speed = base_s * 0.3, base_s * 0.3
            elif effective_command in ['turn_left', 'turn_right']:
                current_turn_cmd = effective_command
                # If a new turn command is received, start the turn phase
                if self.webots_turn_command_active != current_turn_cmd or self.webots_internal_turn_phase == 'NONE':
                    self.webots_turn_command_active = current_turn_cmd
                    self.webots_internal_turn_phase = 'INITIATE_SPIN'
                    self.turn_phase_start_time = current_sim_time
                    print(f"Turn {self.webots_turn_command_active} initiated.")
                if self.webots_internal_turn_phase == 'INITIATE_SPIN':
                    # Initial fast spin to start the turn
                    s_i, s_o = -FORWARD_SPEED * TURN_SPEED_FACTOR * 0.7, FORWARD_SPEED * TURN_SPEED_FACTOR * 1
                    next_left_speed, next_right_speed = (s_i, s_o) if self.webots_turn_command_active == 'turn_left' else (s_o, s_i)
                    if current_sim_time - self.turn_phase_start_time > MIN_INITIAL_SPIN_DURATION:
                        self.webots_internal_turn_phase = 'SEARCHING_LINE'
                        self.turn_phase_start_time = current_sim_time
                        print(f"🔍 Search for {self.webots_turn_command_active}")
                elif self.webots_internal_turn_phase == 'SEARCHING_LINE':
                    # Slow down and search for the line
                    s_i, s_o = -FORWARD_SPEED * TURN_SPEED_FACTOR * 0.3, FORWARD_SPEED * TURN_SPEED_FACTOR * 0.6
                    next_left_speed, next_right_speed = (s_i, s_o) if self.webots_turn_command_active == 'turn_left' else (s_o, s_i)
                    if sensors_on_line:
                        # If line is found, start adjusting to center on it
                        self.webots_internal_turn_phase = 'ADJUSTING_ON_LINE'
                        self.turn_phase_start_time = current_sim_time
                        print(f"Line acquired {self.webots_turn_command_active}")
                    elif not TURN_UNTIL_LINE_FOUND and current_sim_time - self.turn_phase_start_time > MAX_SEARCH_SPIN_DURATION:
                        print(f"Search Timeout {self.webots_turn_command_active}")
                        self.webots_internal_turn_phase = 'NONE'
                        next_left_speed, next_right_speed = 0,0
                elif self.webots_internal_turn_phase == 'ADJUSTING_ON_LINE':
                    # Fine-tune the robot's position on the line after reacquisition
                    b = TURN_ADJUST_BASE_SPEED
                    m_d = MODERATE_CORRECTION_DIFFERENTIAL * (b / FORWARD_SPEED)
                    a_d = AGGRESSIVE_CORRECTION_DIFFERENTIAL * (b / FORWARD_SPEED)
                    if not left_gs and center_gs and not right_gs:
                        next_left_speed, next_right_speed = b * 0.6, b * 0.6
                        self.webots_internal_turn_phase = 'NONE'
                        self.webots_turn_command_active = None
                    elif left_gs and center_gs and not right_gs: next_left_speed,next_right_speed = b-m_d,b
                    elif not left_gs and center_gs and right_gs: next_left_speed,next_right_speed = b,b-m_d
                    elif left_gs and not center_gs and not right_gs: next_left_speed,next_right_speed = b-a_d,b
                    elif not left_gs and not center_gs and right_gs: next_left_speed,next_right_speed = b,b-a_d
                    elif not sensors_on_line:
                        # If line is lost again, go back to searching
                        print(f"Line lost during adjust. Re-search.")
                        self.webots_internal_turn_phase = 'SEARCHING_LINE'
                        self.turn_phase_start_time = current_sim_time
                    # Timeout for adjustment phase to prevent getting stuck
                    if current_sim_time - self.turn_phase_start_time > MAX_ADJUST_DURATION and self.webots_internal_turn_phase == 'ADJUSTING_ON_LINE':
                        print(f"Adjust Timeout {self.webots_turn_command_active}")
                        self.webots_internal_turn_phase = 'NONE'
                        self.webots_turn_command_active = None
                        next_left_speed,next_right_speed=0,0
            # Set the calculated speeds to the motors
            self.left_motor.setVelocity(next_left_speed)
            self.right_motor.setVelocity(next_right_speed)
            self.current_left_speed, self.current_right_speed = next_left_speed, next_right_speed
            # -------------------------------------------------------
            # --- Visualization & Logging ---
            if iteration % 3 == 0:
                self.visualizer.update(self.rwp_estimate, self.crgp_estimated, self.planned_path_grid, self.gs_wb, LINE_THRESHOLD)
            if iteration % 25 == 0:
                conn_stat = ("Yes" if self.is_connected else "No")
                line_s = ("ON" if sensors_on_line else "OFF") 
                obs_s = f"Obst:{len(self.detected_obstacles_grid)}" if self.detected_obstacles_grid else "Null"
                turn_s = f"T:{self.webots_internal_turn_phase}" if self.webots_internal_turn_phase!='NONE' else ""
                cmd_d = f"{self.esp32_command}" if self.esp32_command==effective_command else f"{self.esp32_command}→{effective_command}"
                print(f"T:{current_sim_time:.1f}|ESP:{conn_stat}|Status:{cmd_d}|Grid:{self.crgp_estimated}|L:{line_s}{line_detected}|{obs_s}|{turn_s}")

    def cleanup(self):
        if self.client_socket:
            try: self.client_socket.close()
            except: pass
        # If visualization window is open, block until user closes it
        if self.visualizer.fig: print("\nSim End."); plt.ioff(); plt.show(block=True)
        print("Ctrl Finish.")

# --- Run Controller ---
if __name__ == "__main__":
    controller = WebotsRobotController()
    try:
        controller.main_loop()
    finally:
        controller.cleanup()
