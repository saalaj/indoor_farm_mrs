# main_controller.py
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
        self.sim_time = 0.0 

        # =================================================================
        # DEMO TOGGLE: Final Integration Sequence
        # =================================================================
        self.demo_mode = 'FINAL_INTEGRATION' 

        if self.robot_ID in [1, 2]:
            self.neigh_ids = [2] if self.robot_ID == 1 else [1]
        elif self.robot_ID in [3, 4]:
            self.neigh_ids = [4] if self.robot_ID == 3 else [3]
        else:
            self.neigh_ids = scenario_dict.get_neigh_ids(robot_ID)

        self.avoidance_mode = 'VCC_UNIFORM'   # Final integration: uniform coverage only

        # =================================================================
        # Connectivity Parameters (DELIBERATELY LENIENT)
        # During uniform coverage the paired drones settle on centroids that are
        # ~3.0 m apart (e.g. R1 -> [2.5, 2.0], R2 -> [5.5, 2.0]). With a tight
        # ceiling (the old R_max = 2.0) the tether fires its emergency pull and
        # drags the pair back together, fighting the coverage spread and stalling
        # them. It is loosen so connectivity acts only as a far-drift safety net:
        # no correction during normal monitoring, just a catch if a link is about
        # to be lost completely.
        # =================================================================
        self.R_safe = 3.8  
        self.R_max  = 5.5   

        self.STATE_ENTRANCE = 0
        self.STATE_ENTER_ROOM = 1
        self.STATE_MONITOR_PLANTS = 2
        self.STATE_PAUSE = 3       # deliberate hold on the uniform centroid
        self.STATE_EXIT = 4        # single waypoint-driven exit to the right side

        self.current_state = self.STATE_ENTRANCE
        self.gate_stage = 0
        self.exit_idx = 0          # index along the exit waypoint trail

        
        self.MONITOR_DURATION = 30.0
        self.monitor_start_time = None

        # Brief, deliberate hold once settled on the uniform centroid, so the
        # uniform-coverage behaviour is clearly demonstrated before exiting.
        self.PAUSE_DURATION = 6.0
        self.pause_start_time = None

        paths_config = {
            1: {"gate": [3.5, 2.25]},
            2: {"gate": [4.5, 2.25]},
            3: {"gate": [3.5, -2.25]},
            4: {"gate": [4.5, -2.25]}
        }
        my_config = paths_config.get(self.robot_ID, paths_config[1])
        self.gate_target = np.array(my_config["gate"])
        self.wp_threshold = 0.30

        # =================================================================
        # EXIT WAYPOINT TRAILS
        # room center -> out the gate -> drop/climb into the central E-W corridor
        # lane (y in (-1, 1)) -> straight out to the right-side exit (~x=8.5),
        # finishing in the same vertical order they started on the left.
        # Lanes are staggered so the four drones don't pile up at the crossroad.
        # =================================================================
        self.exit_paths = {
            1: [[2.5,  2.0], [3.6,  2.0], [3.7,  0.70], [8.5,  0.70]],
            2: [[5.5,  2.0], [4.4,  2.0], [4.3,  0.25], [8.5,  0.25]],
            3: [[2.5, -2.0], [3.6, -2.0], [3.7, -0.70], [8.5, -0.70]],
            4: [[5.5, -2.0], [4.4, -2.0], [4.3, -0.25], [8.5, -0.25]],
        }

        self.current_goal = np.zeros(3)

    def compute_control_input(self, estimation_dict):
        self.sim_time += 0.1 

        estimation_dict.neigh_ids = self.neigh_ids
        current_pos = estimation_dict.lahead_pos[0:2]

        # =================================================================
        # FINAL INTEGRATION SEQUENCE
        #   waypoints -> gate -> UNIFORM Voronoi coverage (MONITOR_DURATION s)
        #   -> brief deliberate PAUSE on the centroid (PAUSE_DURATION s)
        #   -> exit trail out to the right side.
        # Coverage is uniform only; there is no non-uniform / emergency phase.
        # =================================================================
        self.avoidance_mode = 'VCC_UNIFORM'

        if self.current_state == self.STATE_MONITOR_PLANTS:
            if self.monitor_start_time is None:
                self.monitor_start_time = self.sim_time            # start the clock on arrival
            elif (self.sim_time - self.monitor_start_time) >= self.MONITOR_DURATION:
                self.current_state = self.STATE_PAUSE              # ~30 s done -> hold
                self.pause_start_time = self.sim_time

        elif self.current_state == self.STATE_PAUSE:
            if (self.sim_time - self.pause_start_time) >= self.PAUSE_DURATION:
                self.current_state = self.STATE_EXIT              # pause done -> leave
                self.exit_idx = 0

        if self.current_state == self.STATE_PAUSE:
            vel_command = np.array([0.0, 0.0, 0.0])
            self.current_goal = np.zeros(3)
            estimation_dict.goal = self.current_goal
            estimation_dict.vel_command = vel_command
            return vel_command

        u_nom = self.get_nominal_velocity(current_pos)

        if self.current_state == self.STATE_MONITOR_PLANTS:
            u_nom = self.get_voronoi_coverage(current_pos, estimation_dict)

        u_rep_robots = self.get_robot_repulsion(current_pos, estimation_dict)
        u_rep_walls  = self.get_lidar_wall_repulsion(current_pos, estimation_dict)
        u_conn       = self.get_connectivity_maintenance(current_pos, estimation_dict)

        u_total = u_nom + u_rep_robots + u_rep_walls + u_conn
        
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

    def get_nominal_velocity(self, current_pos):
        u_nom = np.array([0.0, 0.0])
        speed_multiplier = 0.35 if self.robot_ID in [1, 3] else 0.22

        if self.current_state == self.STATE_ENTRANCE:
            hall_x = 4.0 if self.robot_ID in [1, 3] else 4.5
            target = np.array([hall_x, 0.0])
            u_nom = target - current_pos
            if np.linalg.norm(u_nom) < 0.25:
                self.current_state = self.STATE_ENTER_ROOM
                self.gate_stage = 0  

        elif self.current_state == self.STATE_ENTER_ROOM:
            speed_multiplier = 0.28
            if self.gate_stage == 0:
                prep_target = np.array([self.gate_target[0], 0.0])
                u_nom = prep_target - current_pos
                if np.linalg.norm(u_nom) < 0.15:
                    self.gate_stage = 1
            else:
                u_nom = self.gate_target - current_pos
                if np.linalg.norm(u_nom) < self.wp_threshold:
                    self.current_state = self.STATE_MONITOR_PLANTS

        elif self.current_state == self.STATE_MONITOR_PLANTS:
            u_nom = np.array([0.0, 0.0])

        elif self.current_state == self.STATE_EXIT:
            speed_multiplier = 0.35
            path = self.exit_paths.get(self.robot_ID, self.exit_paths[1])
            target = np.array(path[self.exit_idx])
            u_nom = target - current_pos
            dist = np.linalg.norm(u_nom)

            if self.exit_idx < len(path) - 1:
                if dist < 0.30:                 # reached a waypoint -> advance
                    self.exit_idx += 1
            else:
                if dist < 0.20:                 # reached the final exit point -> stop
                    speed_multiplier = 0.0

        norm_nom = np.linalg.norm(u_nom)
        if norm_nom > 0.001:
            u_nom = (u_nom / norm_nom) * speed_multiplier
        return u_nom

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
    
    def get_voronoi_coverage(self, current_pos, estimation_dict):
        u_cov = np.array([0.0, 0.0])
        plot_bounds = {1: [1.0, 4.0, 1.0, 3.0], 2: [4.0, 7.0, 1.0, 3.0], 3: [1.0, 4.0, -3.0, -1.0], 4: [4.0, 7.0, -3.0, -1.0]}
        bounds = plot_bounds.get(self.robot_ID, [1.0, 4.0, 1.0, 3.0])
        xmin, xmax, ymin, ymax = bounds

        active_positions = {self.robot_ID: current_pos}
        for neigh_id in self.neigh_ids:
            if neigh_id in estimation_dict.neighbours_data:
                active_positions[neigh_id] = estimation_dict.neighbours_data[neigh_id]['lahead'][0:2]

        resolution = 0.15  
        x_space = np.arange(xmin, xmax, resolution)
        y_space = np.arange(ymin, ymax, resolution)
        
        sum_x, sum_y, points_in_cell = 0.0, 0.0, 0

        for gx in x_space:
            for gy in y_space:
                grid_point = np.array([gx, gy])
                closest_id, min_dist = None, float('inf')
                for r_id, r_pos in active_positions.items():
                    dist = np.linalg.norm(grid_point - r_pos)
                    if dist < min_dist:
                        min_dist = dist
                        closest_id = r_id
                
                if closest_id == self.robot_ID:
                    sum_x += gx
                    sum_y += gy
                    points_in_cell += 1

        if points_in_cell > 0:
            centroid = np.array([sum_x / points_in_cell, sum_y / points_in_cell])
            u_cov = 0.65 * (centroid - current_pos)
        return u_cov
    
    def get_voronoi_coverage_non_uniform(self, current_pos, estimation_dict):
        # NOTE: not used in the final integration (uniform coverage only).
        # Kept here for reference / Approach #4.
        u_cov = np.array([0.0, 0.0])
        plot_bounds = {1: [1.0, 4.0, 1.0, 3.0], 2: [4.0, 7.0, 1.0, 3.0], 3: [1.0, 4.0, -3.0, -1.0], 4: [4.0, 7.0, -3.0, -1.0]}
        bounds = plot_bounds.get(self.robot_ID, [1.0, 4.0, 1.0, 3.0])
        xmin, xmax, ymin, ymax = bounds

        room_hotspots = {1: np.array([1.5, 1.5]), 2: np.array([6.5, 1.5]), 3: np.array([1.5, -1.5]), 4: np.array([6.5, -1.5])}
        hotspot = room_hotspots.get(self.robot_ID, np.array([2.0, 2.0]))

        active_positions = {self.robot_ID: current_pos}
        for neigh_id in self.neigh_ids:
            if neigh_id in estimation_dict.neighbours_data and 'lahead' in estimation_dict.neighbours_data[neigh_id]:
                active_positions[neigh_id] = estimation_dict.neighbours_data[neigh_id]['lahead'][0:2]

        resolution = 0.15  
        x_space = np.arange(xmin, xmax, resolution)
        y_space = np.arange(ymin, ymax, resolution)
        sum_weighted_x, sum_weighted_y, total_mass = 0.0, 0.0, 0.0 

        for gx in x_space:
            for gy in y_space:
                grid_point = np.array([gx, gy])
                closest_id, min_dist = None, float('inf')
                for r_id, r_pos in active_positions.items():
                    if r_pos is not None:
                        dist = np.linalg.norm(grid_point - r_pos)
                        if dist < min_dist:
                            min_dist = dist
                            closest_id = r_id
                
                if closest_id == self.robot_ID:
                    dist_to_hotspot = np.linalg.norm(grid_point - hotspot)
                    phi = 100.0 * np.exp(-0.5 * (dist_to_hotspot / 0.4)**2) + 0.01 
                    sum_weighted_x += gx * phi
                    sum_weighted_y += gy * phi
                    total_mass += phi

        if total_mass > 0:
            centroid = np.array([sum_weighted_x / total_mass, sum_weighted_y / total_mass])
            u_cov = 0.65 * (centroid - current_pos)
        else:
            room_center = np.array([(xmin + xmax)/2.0, (ymin + ymax)/2.0])
            u_cov = 0.35 * (room_center - current_pos)
        return u_cov
    
    def get_connectivity_maintenance(self, current_pos, estimation_dict):
        u_conn = np.array([0.0, 0.0])
        k_conn = 0.3   # gentle attraction (lenient, so it never fights coverage)
        
        # =================================================================
        # During the exit run, fully relax the leash so paired drones can take
        # their separate lanes to the right without dragging each other.
        # =================================================================
        current_R_safe = self.R_safe
        current_R_max = self.R_max
        
        if self.current_state == self.STATE_EXIT:
            current_R_max = 12.0
        
        for neigh_id in self.neigh_ids:
            if neigh_id in estimation_dict.neighbours_data and 'lahead' in estimation_dict.neighbours_data[neigh_id]:
                neigh_pos = estimation_dict.neighbours_data[neigh_id]['lahead'][0:2]
                v_dist = neigh_pos - current_pos 
                dist = np.linalg.norm(v_dist)
                
                if dist > current_R_safe:
                    if dist >= current_R_max - 0.05:
                        pull_strength = 5.0
                    else:
                        pull_strength = k_conn * (dist - current_R_safe) / ((current_R_max - dist)**2)
                    u_conn += pull_strength * (v_dist / dist)
        return u_conn

    @staticmethod
    def si_to_unicycle(u, theta, ell):
        vel_lin = u[0]*np.cos(theta) + u[1]*np.sin(theta)
        vel_ang = (- u[0]*np.sin(theta) + u[1]*np.cos(theta))/ell
        return vel_lin, vel_ang