from .nebosim_core.range_sensing import calc_detected_pos
from qpsolvers import Problem, solve_problem
import numpy as np

def calc_lahead_pos(pos, theta, ell):
    return np.array([pos[0] + ell*np.cos(theta), 
                     pos[1] + ell*np.sin(theta), 
                     pos[2] ])    

class Estimation():
    def __init__(self, robot_ID, param_dict):
        self.robot_ID = robot_ID

        # SENSOR-BASED Variables
        self.pos = None
        self.theta = None
        self.range_data = None
        self.range_pos = None
        self.obs_pos = np.empty((0, 3)) 

        self.Ts = param_dict.Ts
        self.look_ahead_dist = param_dict.ell
        self.lahead_pos = None

        sensing_resolution = 360 
        self.beam_angles = np.linspace(0., 2*np.pi, num=sensing_resolution, endpoint=False)

        # CONSENSUS-BASED & COMMUNICATION EXCHANGE Variables
        self.neigh_ids = [] 
        self.neighbours_data = {} 

        self.goal = np.zeros(3)
        self.vel_command = np.zeros(3)

    def update_state_reading(self, pos, theta):
        self.pos = pos
        self.theta = theta
        self.lahead_pos = calc_lahead_pos(pos, theta, self.look_ahead_dist)

    def update_range_sensors(self, range_data, beam_angles = None):
        self.range_data = range_data
        if beam_angles is None: beam_angles = self.beam_angles
        offset = np.pi/2
        self.range_pos = calc_detected_pos(range_data, self.pos, self.theta + offset, beam_angles)
        
        # Filter valid obstacles within sensor range limits
        valid_mask = (range_data > 0.05) & (range_data < 3.5)
        self.obs_pos = self.range_pos[valid_mask]

    def update_neigh_pose(self, robot_id, pos, theta):
        neigh_lahead = calc_lahead_pos(pos, theta, self.look_ahead_dist)
        try:
            self.neighbours_data[robot_id]['pos'] = pos
            self.neighbours_data[robot_id]['theta'] = theta
            self.neighbours_data[robot_id]['lahead'] = neigh_lahead
        except:
            self.neighbours_data[robot_id] = {'pos': pos, 'theta':theta, 'lahead': neigh_lahead}


class Controller():
    def __init__(self, robot_ID, scenario_dict):
        self.robot_ID = int(robot_ID)

        # -----------------------------------------------------------------
        # HARD SPLIT TOPOLOGY INTO COMPACT PAIRS
        # -----------------------------------------------------------------
        if self.robot_ID in [1, 2]:
            # Group A (Top Half): Robot 1 only cares about 2, Robot 2 only cares about 1
            self.neigh_ids = [2] if self.robot_ID == 1 else [1]
        elif self.robot_ID in [3, 4]:
            # Group B (Bottom Half): Robot 3 only cares about 4, Robot 4 only cares about 3
            self.neigh_ids = [4] if self.robot_ID == 3 else [3]
        else:
            self.neigh_ids = scenario_dict.get_neigh_ids(robot_ID)

        # -----------------------------------------------------------------
        # SCOPE-RESTRICTED MILESTONE SWITCHES
        # -----------------------------------------------------------------
        # Options: 
        #   'COLLISION'     : Pure Nominal + Inter-Robot Avoidance
        #   'UNKNOWN_OBS'   : Collision + LiDAR Wall Avoidance
        #   'VCC_UNIFORM'   : Uniform Voronoi Coverage
        #   'VCC_NON_UNIFORM': Non-Uniform Voronoi Coverage
        #   'CONNECTIVITY'  : Path Tracking + Active Communication Link Maintenance
        self.avoidance_mode = 'CONNECTIVITY'

        # Set a safe, realistic communication range for independent pairs
        self.R_safe = 1.6  
        self.R_max  = 2.2  

        # State Machine Trackers
        self.STATE_ENTRANCE = 0
        self.STATE_ENTER_ROOM = 1
        self.STATE_MONITOR_PLANTS = 2
        self.STATE_EXIT_ROOM = 3
        self.STATE_EXIT_ALL = 4
        self.current_state = self.STATE_ENTRANCE
        
        # Optimized path maps ensuring cleaner entry separation
        paths_config = {
            1: {"gate": [3.5, 2.25],  "orbit": [[3.0, 2.8], [1.5, 2.8], [1.5, 1.6], [3.0, 1.6]]},
            2: {"gate": [4.5, 2.25],  "orbit": [[5.0, 2.8], [6.5, 2.8], [6.5, 1.6], [5.0, 1.6]]},
            3: {"gate": [3.5, -2.25], "orbit": [[3.0, -1.6], [1.5, -1.6], [1.5, -2.8], [3.0, -2.8]]},
            4: {"gate": [4.5, -2.25], "orbit": [[5.0, -1.6], [6.5, -1.6], [6.5, -2.8], [5.0, -2.8]]}
        }
        
        my_config = paths_config.get(self.robot_ID, paths_config[1])
        self.gate_target = np.array(my_config["gate"])
        self.orbit_waypoints = [np.array(wp) for wp in my_config["orbit"]]
        self.orbit_index = 0
        self.wp_threshold = 0.30 
        
        self.current_goal = np.zeros(3)

    def compute_control_input(self, estimation_dict):
        # -----------------------------------------------------------------
        # DYNAMIC TOPOLOGY STATE MANAGER
        # -----------------------------------------------------------------
        if self.current_state == self.STATE_EXIT_ALL:
            # Re-engage global team awareness so they can merge into a unified exit convoy
            self.neigh_ids = [1, 2, 3, 4]
            self.neigh_ids.remove(self.robot_ID)
        else:
            # Keep isolated pair-wise groupings during the entry and monitoring phases
            if self.robot_ID in [1, 2]:
                self.neigh_ids = [2] if self.robot_ID == 1 else [1]
            elif self.robot_ID in [3, 4]:
                self.neigh_ids = [4] if self.robot_ID == 3 else [3]

        # Force the updated neighbor structure into the communication dictionary
        estimation_dict.neigh_ids = self.neigh_ids
        current_pos = estimation_dict.lahead_pos[0:2]
        
        # Adjust transmission limits based on current execution needs
        if self.current_state in [self.STATE_ENTRANCE, self.STATE_EXIT_ALL]:
            self.R_safe, self.R_max = 1.4, 2.0
        elif self.current_state == self.STATE_ENTER_ROOM:
            self.R_safe, self.R_max = 1.8, 2.4
        elif self.current_state == self.STATE_MONITOR_PLANTS:
            self.R_safe, self.R_max = 5.0, 6.0

        # Base Trajectory Guidance
        u_nom = self.get_nominal_velocity(current_pos)
        
        # Coverage Overrides
        if self.current_state == self.STATE_MONITOR_PLANTS:
            if self.avoidance_mode == 'VCC_UNIFORM':
                u_nom = self.get_voronoi_coverage(current_pos, estimation_dict)
            elif self.avoidance_mode == 'VCC_NON_UNIFORM':
                u_nom = self.get_voronoi_coverage_non_uniform(current_pos, estimation_dict)

        # Consolidate Safety & Communication Vectors
        u_rep_robots = self.get_robot_repulsion(current_pos, estimation_dict)
        u_rep_walls = self.get_lidar_wall_repulsion(current_pos, estimation_dict)
        u_conn = self.get_connectivity_maintenance(current_pos, estimation_dict)

        # Merge execution velocities
        u_total = u_nom + u_rep_robots + u_rep_walls + u_conn
        
        # -----------------------------------------------------------------
        # DEADLOCK BREAKOUT LAYER (Resolves Multi-Agent Doorway / Local Minima Traps)
        # -----------------------------------------------------------------
        # Detect if the robot is active but local potential fields have pinned it to a standstill
        dist_to_goal = np.linalg.norm(current_pos - self.get_nominal_velocity(current_pos))
        if np.linalg.norm(u_total) < 0.03 and self.current_state != self.STATE_EXIT_ALL:
            if self.robot_ID % 2 == 0:
                nudge = np.array([0.08, -0.04])
            else:
                nudge = np.array([-0.04, 0.08])
                
            if self.current_state in [self.STATE_ENTER_ROOM, self.STATE_EXIT_ROOM, self.STATE_ENTRANCE]:
                u_total += nudge

        # -----------------------------------------------------------------
        # SPEED LIMITER & BOUNDING
        # -----------------------------------------------------------------
        vx, vy = 1.1 * u_total[0], 1.1 * u_total[1]
        max_speed = 0.35
        speed = np.hypot(vx, vy)
        if speed > max_speed:
            vx = (vx / speed) * max_speed
            vy = (vy / speed) * max_speed

        vel_command = np.array([vx, vy, 0.0])
        self.current_goal = np.array([u_total[0], u_total[1], 0.0])
        
        estimation_dict.goal = self.current_goal
        estimation_dict.vel_command = vel_command
        
        return vel_command

    # -----------------------------------------------------------------
    # NOMINAL PATH TRACKING LOGIC (MILESTONE 1)
    # -----------------------------------------------------------------
    def get_nominal_velocity(self, current_pos):
        u_nom = np.array([0.0, 0.0])
        
        speed_multiplier = 0.35 if self.robot_ID in [1, 3] else 0.22

        if self.current_state == self.STATE_ENTRANCE:
            hall_x = 4.0 if self.robot_ID in [1, 3] else 4.5
            target = np.array([hall_x, 0.0])
            u_nom = target - current_pos
            
            if np.linalg.norm(target - current_pos) < 0.25:
                self.current_state = self.STATE_ENTER_ROOM
                self.gate_stage = 0 

        elif self.current_state == self.STATE_ENTER_ROOM:
            speed_multiplier = 0.28
            
            if self.gate_stage == 0:
                prep_target = np.array([self.gate_target[0], 0.0])
                u_nom = prep_target - current_pos
                # Require a tighter tolerance so they line up perfectly before turning
                if np.linalg.norm(prep_target - current_pos) < 0.15:
                    self.gate_stage = 1
            
            else:
                u_nom = self.gate_target - current_pos
                if np.linalg.norm(self.gate_target - current_pos) < self.wp_threshold:
                    self.current_state = self.STATE_MONITOR_PLANTS

        elif self.current_state == self.STATE_MONITOR_PLANTS:
            target = self.orbit_waypoints[self.orbit_index]
            u_nom = target - current_pos
            speed_multiplier = 0.28
            
            if np.linalg.norm(target - current_pos) < self.wp_threshold:
                self.orbit_index += 1
                if self.orbit_index >= len(self.orbit_waypoints):
                    self.current_state = self.STATE_EXIT_ROOM
                    self.exit_stage = 0  # Initialize the exit alignment handler

        elif self.current_state == self.STATE_EXIT_ROOM:
            speed_multiplier = 0.25
            
            if self.exit_stage == 0:
                escape_gate = np.array([self.gate_target[0], 0.0])
                u_nom = escape_gate - current_pos
                
                if np.linalg.norm(escape_gate - current_pos) < 0.15:
                    self.exit_stage = 1
            
            else:
                self.current_state = self.STATE_EXIT_ALL

        elif self.current_state == self.STATE_EXIT_ALL:
            target = np.array([8.3, 0.0])
            u_nom = target - current_pos
            speed_multiplier = 0.25
            
            if np.linalg.norm(target - current_pos) < self.wp_threshold:
                u_nom = np.array([0.0, 0.0])

        # Apply vector normalization and scaling
        norm_nom = np.linalg.norm(u_nom)
        if norm_nom > 0.001:
            u_nom = (u_nom / norm_nom) * speed_multiplier
        return u_nom

    # -----------------------------------------------------------------
    # INTER-ROBOT APF REPULSION 
    # -----------------------------------------------------------------
    def get_robot_repulsion(self, current_pos, estimation_dict):
        u_rep_robots = np.array([0.0, 0.0])
        d_safe_robot = 0.32  
        k_rep_robot = 0.20   
        
        for neigh_id in self.neigh_ids:
            if neigh_id in estimation_dict.neighbours_data and 'lahead' in estimation_dict.neighbours_data[neigh_id]:
                neigh_pos = estimation_dict.neighbours_data[neigh_id]['lahead'][0:2]
                v_dist = current_pos - neigh_pos
                dist = np.linalg.norm(v_dist)
                if dist < d_safe_robot and dist > 0.01:
                    u_rep_robots += k_rep_robot * (1.0/dist - 1.0/d_safe_robot) * (1.0/(dist**2)) * (v_dist / dist)
        return u_rep_robots

    # -----------------------------------------------------------------
    # SENSOR-BASED WALL REPULSION 
    # -----------------------------------------------------------------
    def get_lidar_wall_repulsion(self, current_pos, estimation_dict):
        u_rep_walls = np.array([0.0, 0.0])
        d_safe_wall = 0.28   
        k_rep_wall = 0.12    
        
        if estimation_dict.obs_pos is not None and len(estimation_dict.obs_pos) > 0:
            obs_xy = estimation_dict.obs_pos[:, 0:2]
            for obs in obs_xy:
                v_dist = current_pos - obs
                dist = np.linalg.norm(v_dist)
                
                if dist < d_safe_wall and dist > 0.01:
                    u_rep_walls += k_rep_wall * (1.0/dist - 1.0/d_safe_wall) * (1.0/(dist**2)) * (v_dist / dist)
        return u_rep_walls
    
    # -----------------------------------------------------------------
    # VORONOI-BASED COVERAGE CONTROL METHOD
    # -----------------------------------------------------------------
    def get_voronoi_coverage(self, current_pos, estimation_dict):
        """
        Computes a bounded Voronoi coverage control vector to distribute 
        the robots uniformly over their designated plot spaces.
        """
        u_cov = np.array([0.0, 0.0])
        
        # Define the physical boundaries of each robot's plot area
        # Format: [xmin, xmax, ymin, ymax] matching your map layout coordinates
        plot_bounds = {
            1: [1.0, 4.0,  1.0, 3.0],  # Top-Left Room
            2: [4.0, 7.0,  1.0, 3.0],  # Top-Right Room
            3: [1.0, 4.0, -3.0, -1.0], # Bottom-Left Room
            4: [4.0, 7.0, -3.0, -1.0]  # Bottom-Right Room
        }
        
        bounds = plot_bounds.get(self.robot_ID, [1.0, 4.0, 1.0, 3.0])
        xmin, xmax, ymin, ymax = bounds

        # Collect the positions of all active tracking participants
        # Include this robot's position and all communicated neighbor positions
        active_positions = {self.robot_ID: current_pos}
        
        for neigh_id in self.neigh_ids:
            if neigh_id in estimation_dict.neighbours_data:
                active_positions[neigh_id] = estimation_dict.neighbours_data[neigh_id]['lahead'][0:2]

        # Discretized Grid Sampling to calculate the Centroid
        resolution = 0.15  # Distance spacing between sampling points (meters)
        x_space = np.arange(xmin, xmax, resolution)
        y_space = np.arange(ymin, ymax, resolution)
        
        sum_x = 0.0
        sum_y = 0.0
        points_in_cell = 0

        for gx in x_space:
            for gy in y_space:
                grid_point = np.array([gx, gy])
                
                # Find which active robot ID is closest to this specific grid point
                closest_id = None
                min_dist = float('inf')
                
                for r_id, r_pos in active_positions.items():
                    dist = np.linalg.norm(grid_point - r_pos)
                    if dist < min_dist:
                        min_dist = dist
                        closest_id = r_id
                
                # If this robot is the closest owner, accumulate the point into its Voronoi mass
                if closest_id == self.robot_ID:
                    sum_x += gx
                    sum_y += gy
                    points_in_cell += 1

        # Compute Centroid and apply proportional tracking force
        if points_in_cell > 0:
            centroid = np.array([sum_x / points_in_cell, sum_y / points_in_cell])
            
            # Control law driving vector toward the center of mass
            k_cover = 0.65
            u_cov = k_cover * (centroid - current_pos)
            
        return u_cov
    
    # Non-Uniform Density Centroid Calculation Update:
    # -----------------------------------------------------------------
    def get_voronoi_coverage_non_uniform(self, current_pos, estimation_dict):
        """
        Computes a bounded non-uniform Voronoi coverage control vector 
        to deploy the robot toward a specific high-priority hotspot inside its room.
        """
        u_cov = np.array([0.0, 0.0])
        
        plot_bounds = {
            1: [1.0, 4.0,  1.0, 3.0],  # Room 1: Top-Left
            2: [4.0, 7.0,  1.0, 3.0],  # Room 2: Top-Right
            3: [1.0, 4.0, -3.0, -1.0], # Room 3: Bottom-Left
            4: [4.0, 7.0, -3.0, -1.0]  # Room 4: Bottom-Right
        }
        
        # Fetch bounds for this specific robot ID (default to Room 1 if not found)
        bounds = plot_bounds.get(self.robot_ID, [1.0, 4.0, 1.0, 3.0])
        xmin, xmax, ymin, ymax = bounds

        # Assign the target priority "Hotspot" coordinate inside each room
        room_hotspots = {
            1: np.array([2.0, 2.0]),   
            2: np.array([6.0, 2.0]),   
            3: np.array([2.0, -2.0]),  
            4: np.array([6.0, -2.0])   
        }
        hotspot = room_hotspots.get(self.robot_ID, np.array([2.0, 2.0]))

        # Safely collect positions of all tracking participants over the network
        active_positions = {self.robot_ID: current_pos}
        for neigh_id in self.neigh_ids:
            if neigh_id in estimation_dict.neighbours_data and 'lahead' in estimation_dict.neighbours_data[neigh_id]:
                active_positions[neigh_id] = estimation_dict.neighbours_data[neigh_id]['lahead'][0:2]

        # Generate the full discrete sample spaces using an explicit step size (meters)
        resolution = 0.15  
        x_space = np.arange(xmin, xmax, resolution)
        y_space = np.arange(ymin, ymax, resolution)
        
        # Initialize mass summation variables for Center of Mass tracking
        sum_weighted_x = 0.0
        sum_weighted_y = 0.0
        total_mass = 0.0 

        # Evaluate the custom density distribution across the grid cells
        for gx in x_space:
            for gy in y_space:
                grid_point = np.array([gx, gy])
                
                # Voronoi Territory Check: Who is closest to this point?
                closest_id = None
                min_dist = float('inf')
                
                for r_id, r_pos in active_positions.items():
                    if r_pos is not None:
                        dist = np.linalg.norm(grid_point - r_pos)
                        if dist < min_dist:
                            min_dist = dist
                            closest_id = r_id
                
                if closest_id == self.robot_ID:
                    # Mathematical Gaussian Density Function: phi(q)
                    # Points closer to the hotspot yield a higher weight factor (phi)
                    dist_to_hotspot = np.linalg.norm(grid_point - hotspot)
                    phi = np.exp(-0.5 * (dist_to_hotspot / 1.0)**2) + 0.1 
                    
                    # Accumulate position values multiplied by their local importance weight
                    sum_weighted_x += gx * phi
                    sum_weighted_y += gy * phi
                    total_mass += phi

        if total_mass > 0:
            centroid = np.array([sum_weighted_x / total_mass, sum_weighted_y / total_mass])
            k_cover = 0.65
            u_cov = k_cover * (centroid - current_pos)
        else:
            room_center = np.array([(xmin + xmax)/2.0, (ymin + ymax)/2.0])
            u_cov = 0.35 * (room_center - current_pos)
            
        return u_cov
    
    # -----------------------------------------------------------------
    # POTENTIAL-BASED CONNECTIVITY MAINTENANCE
    # -----------------------------------------------------------------
    def get_connectivity_maintenance(self, current_pos, estimation_dict):
        """
        Computes an attractive potential field vector pulling the robot back 
        toward its assigned neighbors if their distance approaches R_max.
        """
        u_conn = np.array([0.0, 0.0])
        k_conn = 0.45  
        
        for neigh_id in self.neigh_ids:
            if neigh_id in estimation_dict.neighbours_data and 'lahead' in estimation_dict.neighbours_data[neigh_id]:
                neigh_pos = estimation_dict.neighbours_data[neigh_id]['lahead'][0:2]
                
                v_dist = neigh_pos - current_pos  
                dist = np.linalg.norm(v_dist)
                
                if dist > self.R_safe and dist < self.R_max:
                    denom = max(self.R_max - dist, 0.02)
                    pull_magnitude = (dist - self.R_safe) / denom
                    
                    u_conn += k_conn * pull_magnitude * (v_dist / dist)
                    
                elif dist >= self.R_max:
                    u_conn += 1.2 * (v_dist / dist)
                    
        return u_conn

    @staticmethod
    def si_to_unicycle(u, theta, ell):
        vel_lin = u[0]*np.cos(theta) + u[1]*np.sin(theta)
        vel_ang = (- u[0]*np.sin(theta) + u[1]*np.cos(theta))/ell
        return vel_lin, vel_ang
