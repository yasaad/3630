import numpy as np
import pybullet as p


def within_rect (point_x, point_y, x_min, x_max, y_min, y_max):
    return (x_min <= point_x <= x_max) and (y_min <= point_y <= y_max)


def ray_test (start_point, end_point, obstacles):
    start_point = np.array(start_point)
    end_point = np.array(end_point)
    vec = end_point - start_point

    for obstacle in obstacles:
        x_min = obstacle['x_m']
        x_max = obstacle['x_m'] + obstacle['xsize_m']
        y_min = obstacle['y_m']
        y_max = obstacle['y_m'] + obstacle['ysize_m']
        z_min = 0.0
        z_max = obstacle['zsize_m']
        bound_min = [x_min, y_min, z_min]
        bound_max = [x_max, y_max, z_max]

        # find inersection with each plane
        rate_x_min = 0
        rate_x_max = 0
        rate_y_min = 0
        rate_y_max = 0
        rate_z_min = 0
        rate_z_max = 0
        if vec[0]!=0:
            rate_x_min = (x_min - start_point[0])/vec[0]
            rate_x_max = (x_max - start_point[0])/vec[0]
        if vec[1]!=0:
            rate_y_min = (y_min - start_point[1])/vec[1]
            rate_y_max = (y_max - start_point[1])/vec[1]
        if vec[2]!=0:
            rate_z_min = (z_min - start_point[2])/vec[2]
            rate_z_max = (z_max - start_point[2])/vec[2]

        for (rate, axis1, axis2) in zip([rate_x_min, rate_x_max, rate_y_min, rate_y_max, rate_z_min, rate_z_max], [1, 1, 0, 0, 0, 0], [2, 2, 2, 2, 1, 1]):
            if (rate > 0):
                intercept_point = start_point + vec * rate
                if within_rect(intercept_point[axis1], intercept_point[axis2], bound_min[axis1], bound_max[axis1], bound_min[axis2], bound_max[axis2]):
                    return True

    return False


def compute_bearing_range(pose, marker_w, bearing_sig, range_sig):
    R = p.getMatrixFromQuaternion(p.getQuaternionFromEuler([0.0, 0.0, pose[2]]))
    t = [pose[0], pose[1], 0.04902103]
    R_inv = np.array([[R[0], R[3], R[6]], [R[1], R[4], R[7]], [R[2], R[5], R[8]]])
    rel_loc = np.array([marker_w[0] - t[0], marker_w[1] - t[1], marker_w[2] - t[2]])
    marker_r = np.dot(R_inv, rel_loc)
    x = marker_r[0]
    y = marker_r[1]
    if (x > 0):
        range_meas = (x**2 + y**2) ** 0.5
        bearing_meas = np.arctan2(y, x)
        noisy_bearing_meas = np.random.normal(bearing_meas, bearing_sig)
        noisy_range_meas = np.random.normal(range_meas, range_sig)
        # print(bearing_meas, range_meas)
        return [True, noisy_bearing_meas, noisy_range_meas]
    else:
        return [False, 0, 0]


import math


def vertices(x_m, y_m, theta_rad):
    robot_width_m = 0.2
    robot_height_m = 0.15
    robot_xpts = [x_m + (0.5 * robot_width_m * math.cos(theta_rad) - 0.5 * robot_height_m * math.sin(theta_rad)),
                  x_m + (-0.5 * robot_width_m * math.cos(theta_rad) - 0.5 * robot_height_m * math.sin(theta_rad)),
                  x_m + (-0.5 * robot_width_m * math.cos(theta_rad) + 0.5 * robot_height_m * math.sin(theta_rad)),
                  x_m + (0.5 * robot_width_m * math.cos(theta_rad) + 0.5 * robot_height_m * math.sin(theta_rad))]
    robot_ypts = [y_m + (0.5 * robot_width_m * math.sin(theta_rad) + 0.5 * robot_height_m * math.cos(theta_rad)),
                  y_m + (-0.5 * robot_width_m * math.sin(theta_rad) + 0.5 * robot_height_m * math.cos(theta_rad)),
                  y_m + (-0.5 * robot_width_m * math.sin(theta_rad) - 0.5 * robot_height_m * math.cos(theta_rad)),
                  y_m + (0.5 * robot_width_m * math.sin(theta_rad) - 0.5 * robot_height_m * math.cos(theta_rad))]
    return (robot_xpts, robot_ypts)

import json
import copy
import pybullet as p
import pathlib
import plotly.graph_objects as go

from duckie_msgs.msg import Obstacle, ObstacleList


OBJECT_SCALING = {"microwave": 0.75, "refrigerator": 1.75, "oven": 0.82, "tv": 0.75}


class EnvManager:

    def __init__(self, map_fpath: str):
        if not isinstance(map_fpath, str):
            raise TypeError("Expected map_fpath to be a 'str', not '%s'" % type(map_fpath))

        map_file = pathlib.Path(map_fpath)
        if not map_file.is_file():
            raise FileNotFoundError("Expected map file at given path '%s'" % map_fpath)

        with map_file.open() as f:
            self.map_json = json.load(f)

        self.shapes = []
        self.labels = {'x': [], 'y': [], 'text': []}
        self.obs_msg = None
        self.Q_obs = None

    def create_box(self, x_m, y_m, z_m, xsize_m, ysize_m, zsize_m, rgba=None):
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[xsize_m/2, ysize_m/2, zsize_m/2], rgbaColor=rgba)
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[xsize_m/2, ysize_m/2, zsize_m/2])
        p.createMultiBody(baseMass=0,
                          baseCollisionShapeIndex=collision,
                          baseVisualShapeIndex=visual,
                          basePosition=[x_m + xsize_m/2, y_m + ysize_m/2, z_m + zsize_m/2])
        return [x_m + xsize_m/2, y_m + ysize_m/2, zsize_m/2]

    def create_floor(self, x_m, y_m, z_m, xsize_m, ysize_m, zsize_m, rgba=None):
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[xsize_m/2, ysize_m/2, zsize_m/2], rgbaColor=rgba)
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[xsize_m/2, ysize_m/2, zsize_m/2])
        fid = p.createMultiBody(baseMass=0,
                          baseCollisionShapeIndex=collision,
                          baseVisualShapeIndex=visual,
                          basePosition=[x_m + xsize_m/2, y_m + ysize_m/2, z_m + zsize_m/2])
        p.changeDynamics(fid, -1, mass=0.0, lateralFriction=10.0, rollingFriction=0.02, frictionAnchor=1)
        return [x_m + xsize_m/2, y_m + ysize_m/2, zsize_m/2]

    def plot_rect(self, x_m, y_m, xsize_m, ysize_m, rgba=None):
        self.shapes.append(dict(
            type='rect',
            x0=x_m, y0=y_m,
            x1=x_m + xsize_m, y1=y_m + ysize_m,
            fillcolor=f"rgba({rgba[0]}, {rgba[1]}, {rgba[2]}, {rgba[3]})" if rgba else "rgb(255, 255, 255)",
            line_width=0,
            layer="below"
        ))

    def plot_circle(self, x_m, y_m, r_m, rgba=None):
        self.shapes.append(dict(
            type='circle',
            x0=x_m - r_m, y0=y_m - r_m,
            x1=x_m + r_m, y1=y_m + r_m,
            fillcolor=f"rgba({rgba[0]}, {rgba[1]}, {rgba[2]}, {rgba[3]})" if rgba else "rgb(255, 255, 255)",
            line_width=0,
            layer="below"
        ))

    def load_robot(self, initial_pose=None):
        init_rpose = self.map_json['agent']['initial_robot_pose']
        init_x, init_y, init_theta = initial_pose if initial_pose else (init_rpose['x_m'], init_rpose['y_m'], init_rpose['theta_rad'])
        rid = p.loadURDF("{0}/{0}.urdf".format(self.map_json['agent']['robot']),
                   basePosition=[init_x, init_y, init_rpose['z_m']],
                   baseOrientation=p.getQuaternionFromEuler([0, 0, init_theta]))
        return rid

    def load_room(self):
        map_outline = self.map_json['room']
        self.create_floor(0.0, 0.0, -0.1, map_outline['xsize_m'], map_outline['ysize_m'], 0.1, map_outline['floor_rgba'])
        self.plot_rect(0.0, 0.0, map_outline['xsize_m'], map_outline['ysize_m'])
        if map_outline['walls_outside']:
            self.create_box(0.0, -0.1, -0.1, map_outline['xsize_m'], 0.1, map_outline['wall_height_m'] + 0.1, map_outline['walls_rgba']) # bottom wall
            self.create_box(0.0, map_outline['ysize_m'], -0.1, map_outline['xsize_m'], 0.1, map_outline['wall_height_m'] + 0.1, map_outline['walls_rgba']) # top wall
            self.create_box(-0.1, 0.0, -0.1, 0.1, map_outline['ysize_m'], map_outline['wall_height_m'] + 0.1, map_outline['walls_rgba']) # left wall
            self.create_box(map_outline['xsize_m'], 0.0, -0.1, 0.1, map_outline['ysize_m'], map_outline['wall_height_m'] + 0.1, map_outline['walls_rgba']) # right wall

    def load_obstacles(self):
        map_obstacles = self.map_json['obstacles']
        for obstacle in map_obstacles:
            self.plot_rect(obstacle['x_m'], obstacle['y_m'], obstacle['xsize_m'], obstacle['ysize_m'], rgba=[0, 0, 0, 255])
            self.create_box(obstacle['x_m'], obstacle['y_m'], 0.0, obstacle['xsize_m'], obstacle['ysize_m'], obstacle['zsize_m'], obstacle['rgba'])

        # Prepare obstacles message
        self.obs_msg = ObstacleList()
        self.obs_msg.map_width = self.map_json['room']['xsize_m']
        self.obs_msg.map_height = self.map_json['room']['ysize_m']
        map_obstacles = self.map_json['obstacles']
        for obstacle in map_obstacles:
            ob = Obstacle()
            ob.x = obstacle['x_m']
            ob.y = obstacle['y_m']
            ob.width = obstacle['xsize_m']
            ob.height = obstacle['ysize_m']
            self.obs_msg.obs.append(ob)

        # Prepare obstacle configuration space
        self.Q_obs = []
        local_obstacles = copy.deepcopy(self.obs_msg)
        for obstacle in local_obstacles.obs:
            obstacle.x -= 0.15
            obstacle.y -= 0.15
            obstacle.width += 0.3
            obstacle.height += 0.3
            self.Q_obs.append(obstacle)

    def load_markers(self):
        markers = self.map_json['markers']
        for marker in markers:
            self.plot_circle(marker['cx_m'], marker['cy_m'], 0.1, rgba=[252, 186, 3, 255])
            self.labels['x'].append(marker['cx_m'])
            self.labels['y'].append(marker['cy_m'])
            self.labels['text'].append(marker['id'])
            p.loadURDF("{0}/{0}.urdf".format(marker['id']),
                       basePosition=[marker['cx_m'], marker['cy_m'], marker['cz_m']],
                       baseOrientation=p.getQuaternionFromEuler([0.0, 0.0, marker['angle_rad']]),
                       globalScaling=OBJECT_SCALING.get(marker['id'], 1.0))

#
# Duckie Environment Simulator 0.0.0
#
# ROS2 node hosting the pybullet simulation of
# the Duckiebot and its environment.
#

from duckie_msgs.msg import Wheels, RangeBearingLandmark, RangeBearingLandmarkList

import sys
import math
import time
import copy
import pathlib
import argparse
import numpy as np
import pybullet as p
import plotly.graph_objects as go


SIM_TIMESTEP = 1/240
MAX_ANGULAR_RATE = 17.4 # rad/s


class Simulator():

    def __init__(self, map_fpath, load_landmarks=True,
                 odom_sigma_x=2e-3, odom_sigma_y=2e-3, odom_sigma_h=4e-3,
                 marker_sigma_range=0.5, marker_sigma_bearing=0.3,
                 initial_pose=None):
        # pybullet setup
        self.env_manager = EnvManager(map_fpath)
        p.connect(p.DIRECT)
        p.resetSimulation()
        urdfs_fpath = pathlib.Path() / 'urdf'
        p.setAdditionalSearchPath(str(urdfs_fpath))
        self.env_manager.load_room()
        self.env_manager.load_obstacles()
        if load_landmarks:
            self.env_manager.load_markers()
        self.rid = self.env_manager.load_robot(initial_pose)
        p.setGravity(0, 0, -9.8066)

        # sim/viz setup
        self.pose_history = []
        self.particles_history = []
        self.odom_sigma_x = odom_sigma_x
        self.odom_sigma_y = odom_sigma_y
        self.odom_sigma_h = odom_sigma_h
        self.marker_sigma_r = marker_sigma_range
        self.marker_sigma_b = marker_sigma_bearing

        # motors setup
        self.left_wheel_joint = 'left_wheel_joint'
        self.left_wheel_joint_id = None
        self.right_wheel_joint = 'right_wheel_joint'
        self.right_wheel_joint_id = None
        self.caster_joint = 'caster_joint'
        self.caster_joint_id = None
        for jid in range(p.getNumJoints(self.rid)):
            if p.getJointInfo(self.rid, jid)[1].decode('UTF-8') == self.left_wheel_joint:
                self.left_wheel_joint_id = jid
            if p.getJointInfo(self.rid, jid)[1].decode('UTF-8') == self.right_wheel_joint:
                self.right_wheel_joint_id = jid
            if p.getJointInfo(self.rid, jid)[1].decode('UTF-8') == self.caster_joint:
                self.caster_joint_id = jid

        if not isinstance(self.left_wheel_joint_id, int):
            raise ValueError("Unable to find joint '%s' in Duckiebot URDF" % self.left_wheel_joint)
        if not isinstance(self.right_wheel_joint_id, int):
            raise ValueError("Unable to find joint '%s' in Duckiebot URDF" % self.right_wheel_joint)
        if not isinstance(self.caster_joint_id, int):
            raise ValueError("Unable to find joint '%s' in Duckiebot URDF" % self.caster_joint)

        # caster setup
        p.setJointMotorControlMultiDof(self.rid,
                                       self.caster_joint_id,
                                       p.POSITION_CONTROL,
                                       [0, 0, 0],
                                       targetVelocity=[100000, 100000, 100000],
                                       positionGain=0,
                                       velocityGain=1,
                                       force=[0, 0, 0])

    def get_obstacles(self):
        return self.env_manager.obs_msg

    def is_free(self, x, y):
        """Checks if a robot modeled as a circle of radius
        `rr` is in a free spot in the map
        """
        if self.env_manager.Q_obs is None:
            return False

        if (x < 0.0 or y < 0.0 or
            x > self.env_manager.map_json['room']['xsize_m'] or
            y > self.env_manager.map_json['room']['xsize_m']):
            return False

        for ob in self.env_manager.Q_obs:
            if x >= ob.x and x <= ob.x + ob.width and y >= ob.y and y <= ob.y + ob.height:
                return False
        return True

    def visible_markers(self, pose, is_gt=False):
        robot_t = [pose[0], pose[1], 0.04902103]
        landmarks = []
        for marker in self.env_manager.map_json['markers']:
            marker_location = [marker['cx_m'], marker['cy_m'], marker['cz_m']]
            if (not ray_test(robot_t, marker_location, self.env_manager.map_json['obstacles'])):
                marker_sigma_b = 0 if is_gt else self.marker_sigma_b
                marker_sigma_r = 0 if is_gt else self.marker_sigma_r
                br_result = compute_bearing_range(pose, marker_location, marker_sigma_b, marker_sigma_r)
                if br_result[0]:
                    landmark_msg = RangeBearingLandmark()
                    landmark_msg.bearing = br_result[1]
                    landmark_msg.range = br_result[2]
                    landmark_msg.id = marker['id']
                    landmarks.append(landmark_msg)
        return landmarks

    def visible_markers_gt(self, pose):
        return self.visible_markers(pose, True)

    def compute_relative(self, pose_i, pose_j):
        """Computes the transform T_i^j (from pose_i to pose_j)

        Parameters
        ----------
        pose_i: tuple(x, y, theta)
            a tuple containing (x, y, theta) values in world frame
        pose_j: tuple(x, y, theta)
            a tuple containing (x, y, theta) values in world frame

        Returns
        -------
        tuple(dx, dy, dtheta)
            the calculated transform T_i^j
        """
        x_i, y_i, theta_i = pose_i
        x_j, y_j, theta_j = pose_j
        delta_x_w = x_j - x_i
        delta_y_w = y_j - y_i
        delta_theta = theta_j - theta_i

        iRw = np.array([[np.cos(theta_i), np.sin(theta_i)], 
                        [-np.sin(theta_i), np.cos(theta_i)]])
        delta_xy_w = np.array([delta_x_w, delta_y_w])
        delta_xy_i = np.dot(iRw, delta_xy_w)

        delta_x_i = delta_xy_i[0]
        delta_y_i = delta_xy_i[1]

        return (delta_x_i, delta_y_i, delta_theta)

    def step(self, cmd_msg, particles, duration=0.1):
        # get true robot position
        pos_prev, quat_prev = p.getBasePositionAndOrientation(self.rid)
        euler_prev = p.getEulerFromQuaternion(quat_prev)
        pose_prev = (pos_prev[0], pos_prev[1], euler_prev[2])
        self.pose_history.append(pose_prev)
        self.particles_history.append(particles if particles is not None else np.empty((0, 3)))

        # load command
        target_left_motor_radps = min(max(cmd_msg.left_wheel, -MAX_ANGULAR_RATE), MAX_ANGULAR_RATE)
        target_right_motor_radps = min(max(cmd_msg.right_wheel, -MAX_ANGULAR_RATE), MAX_ANGULAR_RATE)
        p.setJointMotorControlArray(self.rid,
            [self.left_wheel_joint_id, self.right_wheel_joint_id], p.VELOCITY_CONTROL,
            targetVelocities=[target_left_motor_radps, target_right_motor_radps]
        )

        # step until duration has passed
        num_steps = duration / SIM_TIMESTEP
        for step_counter in range(int(num_steps)):
            p.stepSimulation()

        # apply noise to relative pose
        pos_curr, quat_curr = p.getBasePositionAndOrientation(self.rid)
        euler_curr = p.getEulerFromQuaternion(quat_curr)
        pose_curr = (pos_curr[0], pos_curr[1], euler_curr[2])
        rel_pose = self.compute_relative(pose_prev, pose_curr)
        step_sigma_x = self.odom_sigma_x * math.sqrt(num_steps)
        step_sigma_y = self.odom_sigma_y * math.sqrt(num_steps)
        step_sigma_h = self.odom_sigma_h * math.sqrt(num_steps)
        rel_pose_x, rel_pose_y, rel_pose_h = rel_pose
        rel_pose_x += np.random.normal(0, step_sigma_x)
        rel_pose_y += np.random.normal(0, step_sigma_y)
        rel_pose_h += np.random.normal(0, step_sigma_h)
        
        # calculate markers seen by the robot
        landmarks_msg = self.visible_markers((pos_curr[0], pos_curr[1], euler_curr[2]))

        rel_pose = (rel_pose_x, rel_pose_y, rel_pose_h)
        step_sigmas = (step_sigma_x, step_sigma_y, step_sigma_h)
        return rel_pose, step_sigmas, landmarks_msg

    def visualize(self, plan=None, show_grid_lines=False, duration=0.1):
        # Setup data
        labels = [str(i) for i in range(len(self.pose_history))]
        first_data = None
        frames = []
        for history_index in range(len(self.pose_history)):
            xs, ys = vertices(*self.pose_history[history_index])
            viz_particles = self.particles_history[history_index]
            data = [
                go.Scatter(
                    x=viz_particles[:, 0], y=viz_particles[:, 1],
                    mode="markers",
                    showlegend=False
                ),
                go.Scatter(
                    x=xs, y=ys,
                    mode="markers",
                    marker=dict(color='Red', size=4),
                    fill="toself",
                    showlegend=False
                ),
                go.Scatter(
                    x=self.env_manager.labels['x'],
                    y=self.env_manager.labels['y'],
                    text=self.env_manager.labels['text'],
                    mode="text",
                    textfont=dict(
                        family="Courier New, monospace",
                        color='rgb(230, 230, 230)'
                    ),
                    showlegend=False
                )
            ]
            frames.append(go.Frame(name=labels[history_index], data=data))
            if first_data is None:
                first_data = data

        # Include plan
        local_shapes = copy.deepcopy(self.env_manager.shapes)
        if plan is not None:
            for path_segment in zip(plan.poses, plan.poses[1:]):
                local_shapes.append(dict(
                    type='line',
                    x0=path_segment[0].pose.position.x, y0=path_segment[0].pose.position.y,
                    x1=path_segment[1].pose.position.x, y1=path_segment[1].pose.position.y,
                    line=dict(
                        color="rgb(0, 255, 0)",
                        width=2
                    )
                ))

        # Show play button when enough frames
        play_button = []
        if len(frames) >= 2:
            play_button = [dict(
                buttons=[dict(
                    args=[None, dict(
                        frame=dict(duration=1000 * duration),
                        fromcurrent=True
                    )],
                    label="Play",
                    method="animate"
                )],
                direction="left",
                pad=dict(r=10, t=30),
                showactive=False,
                type="buttons",
                x=0.05,
                xanchor="right",
                y=0,
                yanchor="top",
                font=dict(
                    family="Courier New, monospace",
                    color='rgb(230, 230, 230)'
                )
            )]

        # Setup layout
        layout = go.Layout(
            xaxis=dict(dtick=1.0,
                    range=[-1, self.env_manager.map_json['room']['xsize_m'] + 1],
                    showticklabels=show_grid_lines,
                    showgrid=show_grid_lines,
                    gridcolor='rgba(175, 175, 175, 255)',
                    zeroline=False),
            yaxis=dict(dtick=1.0,
                    range=[-1, self.env_manager.map_json['room']['ysize_m'] + 1],
                    showticklabels=show_grid_lines,
                    showgrid=show_grid_lines,
                    gridcolor='rgba(175, 175, 175, 255)',
                    zeroline=False,
                    scaleanchor="x",
                    scaleratio=1),
            margin=dict(r=30, l=30, b=30, t=30),
            paper_bgcolor='rgba(50, 50, 60, 255)',
            plot_bgcolor='rgba(50, 50, 60, 255)',
            font=dict(
                family="Courier New, monospace",
                color='rgba(230, 230, 230, 255)' if show_grid_lines else 'rgba(255, 255, 255, 0)'
            ),
            shapes=local_shapes,
            dragmode='pan',
            hovermode=False,
            sliders=[dict(
                active=0,
                yanchor="top",
                xanchor="left",
                currentvalue=dict(
                    font=dict(
                        family="Courier New, monospace",
                        color='rgb(230, 230, 230)'
                    ),
                    prefix="Step ",
                    visible=True,
                    xanchor="right"
                ),
                pad=dict(b=10, t=0),
                len=0.95,
                x=0.05,
                y=0,
                font=dict(
                    family="Courier New, monospace",
                    color='rgb(230, 230, 230)'
                ),
                steps=[dict(
                        args=[[local_label], dict(
                            frame=dict(duration=1000 * duration),
                            mode="immediate"
                        )],
                        label=local_label,
                        method="animate") for local_label in labels]
            )],
            updatemenus=play_button,
        )

        fig = go.Figure(data=first_data, layout=layout, frames=frames)
        fig.show()

