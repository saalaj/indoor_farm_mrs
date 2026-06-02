#!/usr/bin/python3
import rclpy, signal
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from armrs_msgs.msg import StateExchange, FleetInformation

from functools import partial
import os
import numpy as np
from scipy.spatial import Voronoi, QhullError 
from matplotlib.patches import Polygon
from matplotlib.path import Path  

from .yaml_loader import ParamLoader, ScenarioLoader
from .visualizer import PlotVisualizer
from .main_controller import Estimation
from .cent_evaluator import CentralizedEvaluator
from .nebosim_core.range_sensing import calc_detected_pos

from . import ROS2_py_common as ros2py

VIDEO_OUT = False
if VIDEO_OUT: from .nebosim_core.video import video

class Computation(Node):
    def __init__(self, ROS_NODE_NAME):
        super().__init__(ROS_NODE_NAME)
        
        # =================================================================
        # VISUALIZER MODE TOGGLE:
        # Options: 'UNIFORM' or 'NON_UNIFORM'
        # =================================================================
        self.density_mode = 'NON_UNIFORM' 
        
        self.declare_parameter('param_yaml', '')
        self.declare_parameter('scenario_yaml', '')
        param_file = self.get_parameter('param_yaml').get_parameter_value().string_value
        scenario_file = self.get_parameter('scenario_yaml').get_parameter_value().string_value

        param = ParamLoader(param_file)
        scenario = ScenarioLoader(scenario_file)

        self.robot_est = {}
        for id in scenario.list_robot_ID:
            self.robot_est[id] = Estimation(id, param)

        self.plot_vis = PlotVisualizer(param, scenario)
        self.evaluator = CentralizedEvaluator(scenario)

        # Plotting Caches
        self._custom_com_lines = {}
        self._voronoi_patches = {}
        self._centroid_markers = {}
        self._centroid_lines = {}
        
        self.plot_vis.SHOW_COMMUNICATION = False 

        # Draw static obstacles
        for key in param.obstacles:
            obj_vertices = param.obstacles[key]
            self.plot_vis.ax_2D.plot(obj_vertices[:, 0], obj_vertices[:, 1], 'k')

        for robot_index in scenario.list_robot_ID:
            tb_name = f'tb4_0{robot_index}'
            vrpn_name = f'tb_0{robot_index}'

            self.pose_sub = self.create_subscription(PoseStamped,
                                    f'/vrpn_mocap/{vrpn_name}/pose',
                                    partial(self.pose_callback, index=robot_index),
                                    qos_profile=qos_profile_sensor_data)

            self.create_subscription(LaserScan,
                                     f'/{tb_name}/scan',
                                     partial(self.scan_LIDAR_callback, index=robot_index),
                                     qos_profile=qos_profile_sensor_data)

            self.state_sub = self.create_subscription(StateExchange,
                                                    f'/{tb_name}/state',
                                                    partial(self.state_callback, index=robot_index),
                                                    qos_profile=qos_profile_sensor_data)

        for f_id in self.evaluator.form_ids:
            fleet_name = f'fleet_{f_id}'
            self.fleet_sub = self.create_subscription(FleetInformation,
                                                    f'/{fleet_name}/diagnosis',
                                                    partial(self.fleet_callback, index=f_id),
                                                    qos_profile=qos_profile_sensor_data)

        self.Ts = 0.1 
        self.ROS_RATE = round(1/self.Ts)
        self.sim_timer = self.create_timer(self.Ts, self.vis_loop)
        self.it = 0
        self.start_t = self.time()
        self.check_t = self.time()

        self.plot_vis.Ts = self.Ts
        self.plot_vis.tseries_data_num = round(self.plot_vis.time_series_window / self.plot_vis.Ts)
        self.plot_vis.array_time = [None] * self.plot_vis.tseries_data_num

        if VIDEO_OUT:
            self.vid_fname = "sim_video.avi"
            self.video_out = video(self.plot_vis.fig, self.vid_fname, self.ROS_RATE)

    def time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def pose_callback(self, msg, index):
        pos, yaw = ros2py.get_pos_yaw(msg)
        self.robot_est[index].update_state_reading(np.array([pos.x, pos.y, 0]), yaw)

    def state_callback(self, msg, index):
        self.robot_est[index].goal, self.robot_est[index].vel_command = ros2py.get_navigation_data(msg)

    def scan_LIDAR_callback(self, msg, index): 
        scan_data, beam_angles = ros2py.get_scan_data(msg)
        if self.robot_est[index].pos is not None:
            self.robot_est[index].update_range_sensors(scan_data, beam_angles)

    def fleet_callback(self, msg, index):
        ros2py.msg_to_cent_evaluator_data(index, msg, self.evaluator)

    def vis_loop(self):
        now = self.time()
        diff = (now - self.check_t)
        if diff > (1.1 * self.Ts):
            pass 
        self.check_t = now

        self.plot_vis.SHOW_COMMUNICATION = False

        in_rooms = True
        for r_id in [1, 2, 3, 4]:
            if r_id in self.robot_est and self.robot_est[r_id].pos is not None:
                if self.robot_est[r_id].pos[0] < 3.8:
                    in_rooms = False
                    break
        r_max = 3.8 if in_rooms else 2.0

        elapsed_time = (now - self.start_t)
        self.plot_vis.update(elapsed_time, self.robot_est, self.evaluator)

        if hasattr(self.plot_vis, 'ax_2D') and self.plot_vis.ax_2D is not None:
            
            # =================================================================
            # CONNECTIVITY TETHER LINES
            # =================================================================
            all_possible_pairs = [(1, 2), (1, 3), (1, 4), (2, 3), (2, 4), (3, 4)]
            for r1_id, r2_id in all_possible_pairs:
                pair_key = (r1_id, r2_id)
                show_link = False

                if r1_id in self.robot_est and r2_id in self.robot_est:
                    est1 = self.robot_est[r1_id]
                    est2 = self.robot_est[r2_id]

                    if est1.pos is not None and est2.pos is not None:
                        dist = np.linalg.norm(est1.pos[0:2] - est2.pos[0:2])
                        is_same_group = (r1_id in [1, 2] and r2_id in [1, 2]) or (r1_id in [3, 4] and r2_id in [3, 4])
                        show_link = (is_same_group and dist <= r_max) if not in_rooms else (dist <= r_max)

                if show_link:
                    x_data = [self.robot_est[r1_id].pos[0], self.robot_est[r2_id].pos[0]]
                    y_data = [self.robot_est[r1_id].pos[1], self.robot_est[r2_id].pos[1]]

                    if pair_key in self._custom_com_lines:
                        self._custom_com_lines[pair_key].set_data(x_data, y_data)
                        self._custom_com_lines[pair_key].set_visible(True)
                    else:
                        line, = self.plot_vis.ax_2D.plot(x_data, y_data, color='#1f77b4', linewidth=3.5, linestyle='-', alpha=0.9, zorder=1)
                        self._custom_com_lines[pair_key] = line
                else:
                    if pair_key in self._custom_com_lines:
                        self._custom_com_lines[pair_key].set_visible(False)

            # =================================================================
            # BOUNDED VORONOI COVERAGE CELLS & CENTROIDS
            # =================================================================
            b_xmin, b_xmax = -1.0, 9.0
            b_ymin, b_ymax = -4.0, 4.0
            
            room_hotspots = {
                1: np.array([2.0, 2.0]),   
                2: np.array([6.0, 2.0]),   
                3: np.array([2.0, -2.0]),  
                4: np.array([6.0, -2.0])   
            }
            
            robot_positions = []
            active_ids = []
            
            for id, est in self.robot_est.items():
                if est.pos is not None:
                    robot_positions.append(est.pos[:2]) 
                    active_ids.append(id)
                    
            active_vor_ids = set()
            
            if len(robot_positions) >= 2:
                region_colors = {
                    1: '#1f77b4',  
                    2: '#2ca02c',  
                    3: '#d62728',  
                    4: '#ff7f0e'   
                }

                try:
                    pts = list(robot_positions)
                    for rx, ry in robot_positions:
                        # 8-Point Mirroring safely seals all walls AND corners
                        pts.append([2 * b_xmin - rx, ry]) 
                        pts.append([2 * b_xmax - rx, ry]) 
                        pts.append([rx, 2 * b_ymin - ry]) 
                        pts.append([rx, 2 * b_ymax - ry]) 
                        pts.append([2 * b_xmin - rx, 2 * b_ymin - ry]) 
                        pts.append([2 * b_xmax - rx, 2 * b_ymin - ry]) 
                        pts.append([2 * b_xmin - rx, 2 * b_ymax - ry]) 
                        pts.append([2 * b_xmax - rx, 2 * b_ymax - ry]) 
                    
                    pts = np.array(pts)
                    vor = Voronoi(pts, qhull_options='QJ')
                    
                    for i, r_id in enumerate(active_ids):
                        region_idx = vor.point_region[i]
                        region = vor.regions[region_idx]
                        
                        c = region_colors.get(r_id, 'gray')
                        
                        if -1 not in region and len(region) > 0:
                            verts = np.array([vor.vertices[v] for v in region])
                            active_vor_ids.add(r_id)
                            
                            # --- Render Polygon ---
                            if r_id in self._voronoi_patches:
                                self._voronoi_patches[r_id].set_xy(verts)
                                self._voronoi_patches[r_id].set_facecolor(c)
                                self._voronoi_patches[r_id].set_edgecolor(c)
                                self._voronoi_patches[r_id].set_visible(True)
                            else:
                                poly = Polygon(verts, edgecolor=c, facecolor=c, alpha=0.25, zorder=2)
                                self.plot_vis.ax_2D.add_patch(poly)
                                self._voronoi_patches[r_id] = poly

                            # --- Calculate Unified Grid Centroid ---
                            vx_min, vx_max = np.min(verts[:, 0]), np.max(verts[:, 0])
                            vy_min, vy_max = np.min(verts[:, 1]), np.max(verts[:, 1])
                            
                            resolution = 0.15
                            xx, yy = np.meshgrid(np.arange(vx_min, vx_max, resolution), 
                                                 np.arange(vy_min, vy_max, resolution))
                            grid_points = np.c_[xx.ravel(), yy.ravel()]
                            
                            poly_path = Path(verts)
                            inside_mask = poly_path.contains_points(grid_points)
                            valid_points = grid_points[inside_mask]
                            
                            if len(valid_points) > 0:
                                # Apply the math based on the mode toggle
                                if self.density_mode == 'UNIFORM':
                                    phis = np.ones(len(valid_points))
                                elif self.density_mode == 'NON_UNIFORM':
                                    hotspot = room_hotspots.get(r_id, np.array([2.0, 2.0]))
                                    dists = np.linalg.norm(valid_points - hotspot, axis=1)
                                    phis = np.exp(-0.5 * (dists / 1.0)**2) + 0.1
                                    
                                total_mass = np.sum(phis)
                                cx = np.sum(valid_points[:, 0] * phis) / total_mass
                                cy = np.sum(valid_points[:, 1] * phis) / total_mass
                            else:
                                cx, cy = np.mean(verts, axis=0) 

                            # --- Render Centroid Marker (Star) ---
                            if r_id in self._centroid_markers:
                                self._centroid_markers[r_id].set_data([cx], [cy])
                                self._centroid_markers[r_id].set_color(c)
                                self._centroid_markers[r_id].set_visible(True)
                            else:
                                marker, = self.plot_vis.ax_2D.plot([cx], [cy], marker='*', color=c, markersize=10, zorder=3)
                                self._centroid_markers[r_id] = marker

                            # --- Render VCC Pull Vector ---
                            rx, ry = robot_positions[i]
                            if r_id in self._centroid_lines:
                                self._centroid_lines[r_id].set_data([rx, cx], [ry, cy])
                                self._centroid_lines[r_id].set_color(c)
                                self._centroid_lines[r_id].set_visible(True)
                            else:
                                line, = self.plot_vis.ax_2D.plot([rx, cx], [ry, cy], linestyle='--', color=c, alpha=0.8, linewidth=1.5, zorder=2)
                                self._centroid_lines[r_id] = line
                                
                except (QhullError, Exception):
                    pass 

            for r_id in list(self._voronoi_patches.keys()):
                if r_id not in active_vor_ids:
                    self._voronoi_patches[r_id].set_visible(False)
                    if r_id in self._centroid_markers: self._centroid_markers[r_id].set_visible(False)
                    if r_id in self._centroid_lines: self._centroid_lines[r_id].set_visible(False)

            if hasattr(self.plot_vis, 'fig') and self.plot_vis.fig is not None:
                self.plot_vis.fig.canvas.draw_idle()

        if VIDEO_OUT: 
            self.video_out.save_image()

def main(args=None):
    ROS_NODE_NAME = 'mrs_visualizer'
    rclpy.init(args=args)
    node = Computation(ROS_NODE_NAME)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()