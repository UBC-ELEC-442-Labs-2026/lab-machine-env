from pal.products.qarm import QArm
from hal.products.qarm import QArmUtilities
import time
import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import importlib.util
import sys
import constants

# Import the QArm interface class
file_path = constants.path_to_interface
class_name = "QArm_Lab_interface"
module_name = "QArm_Lab_interface_module"
spec = importlib.util.spec_from_file_location(module_name, file_path)
module = importlib.util.module_from_spec(spec)
sys.modules[module_name] = module
spec.loader.exec_module(module)
QArm_Lab_interface = getattr(module, class_name)

# ---------------------------------------------------------
# Base Controller
# ---------------------------------------------------------
class BaseQArmController:
    def __init__(self, waypoints):
        """
        waypoints: List or numpy array of points in [x, y, z, t] format.
        """
        self.waypoints = np.array(waypoints)
        self.position_anchors = self.waypoints[:, :3]
        self.time_anchors = self.waypoints[:, 3]
        
        self.myArmUtilities = QArm_Lab_interface()
        self.gamma = 0
        self.gripCmd = 0
        
        self.startTime = 0
        self.last_time = 0
        self.animated_points = []
        
        self.fig = None
        self.ax = None
        self.trail_graph = None
        self.leading_dot = None
        self.live_drop_line = None
        self.live_ground_shadow = None

    def elapsed_time(self):
        return time.time() - self.startTime

    def setup_3d_plot(self):
        self.fig = plt.figure(figsize=(8, 6))
        self.ax = self.fig.add_subplot(projection='3d')

        self.trail_graph, = self.ax.plot([], [], [], color='blue', alpha=0.5, linewidth=1.5, label="Trajectory Trail")
        self.leading_dot, = self.ax.plot([], [], [], color='royalblue', marker='o', markersize=6, zorder=10)

        self.ax.view_init(elev=25, azim=45)
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.ax.set_zlim(0, 1)

        self.ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        self.ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        self.ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        self.ax.grid(False)

        self.ax.plot([-1, 1], [0, 0], [0, 0], color='gray', linewidth=1, linestyle='--') 
        self.ax.plot([0, 0], [-1, 1], [0, 0], color='gray', linewidth=1, linestyle='--') 
        self.ax.plot([0, 0], [0, 0], [0, 1], color='gray', linewidth=1, linestyle='--')  

        self.ax.text(1.1, 0, 0, 'X', horizontalalignment='center', fontweight='bold')
        self.ax.text(0, 1.1, 0, 'Y', horizontalalignment='center', fontweight='bold')
        self.ax.text(0, 0, 1.1, 'Z', horizontalalignment='center', fontweight='bold')

        colors = plt.cm.rainbow(np.linspace(0, 1, len(self.position_anchors)))
        for idx, pt in enumerate(self.position_anchors):
            c = colors[idx]
            self.ax.scatter(pt[0], pt[1], pt[2], color=c, marker='o', s=40, zorder=15)
            self.ax.text(pt[0] + 0.05, pt[1], pt[2] + 0.05, f'WP {idx}', color=c, fontweight='bold')
            self.ax.plot([pt[0], pt[0]], [pt[1], pt[1]], [pt[2], 0], color=c, linestyle='--', alpha=0.4, linewidth=1.0)

        self.live_drop_line, = self.ax.plot([], [], [], color='purple', linestyle='-', alpha=0.7, linewidth=1.5)
        self.live_ground_shadow, = self.ax.plot([], [], [], color='black', alpha=0.2, linewidth=1)

    def update_plot_elements(self, x, y, z):
        self.animated_points.append([x, y, z])
        matrix = np.array(self.animated_points)
        self.trail_graph.set_data_3d(matrix[:, 0], matrix[:, 1], matrix[:, 2])
        self.leading_dot.set_data_3d([x], [y], [z])
        self.live_drop_line.set_data_3d([x, x], [y, y], [z, 0])
        self.live_ground_shadow.set_data_3d([0, x], [0, y], [0, 0])

# ---------------------------------------------------------
# Cartesian Jacobian Controller
# ---------------------------------------------------------
class CartesianJacobianController(BaseQArmController):
    def __init__(self, waypoints, hardware):
        super().__init__(waypoints)
        self.Kp = 1.5  
        self.spline = CubicSpline(self.time_anchors, self.position_anchors, axis=0, bc_type='clamped')
        self.spline_velocity = self.spline.derivative()
        self.q_next = None
        
        self.last_drop_time = 0
        self.DROP_INTERVAL = 0.5 
        self.hardware = hardware

    def plot_kinematic_profiles(self):
        t_plot = np.linspace(self.time_anchors[0], self.time_anchors[-1], 500)
        positions = self.spline(t_plot)
        velocities = self.spline_velocity(t_plot)
        accelerations = self.spline_velocity.derivative()(t_plot)
        jerks = self.spline_velocity.derivative().derivative()(t_plot)

        fig_diag, axs = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
        colors = ['crimson', 'orange', 'forestgreen']
        axes_labels = ['X', 'Y', 'Z']

        for i in range(3): 
            axs[0].plot(t_plot, positions[:, i], color=colors[i], label=f'{axes_labels[i]} Position')
            axs[1].plot(t_plot, velocities[:, i], color=colors[i], label=f'{axes_labels[i]} Velocity')
            axs[2].plot(t_plot, accelerations[:, i], color=colors[i], label=f'{axes_labels[i]} Acceleration')
            axs[3].plot(t_plot, jerks[:, i], color=colors[i], label=f'{axes_labels[i]} Jerk')

        for idx, ax_sub in enumerate(axs):
            ax_sub.grid(True, linestyle='--', alpha=0.6)
            if idx == 0:
                ax_sub.legend(loc='upper right')

        plt.suptitle('Cubic Spline Kinematic Profiles Over Time', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.show(block=False)

    def _animation_update(self, frame, myArm):
        t_max = self.time_anchors[-1]
        t = np.clip(self.elapsed_time(), 0.0, t_max)
        
        if t >= t_max:
            return self.trail_graph, self.leading_dot, self.live_drop_line, self.live_ground_shadow
            
        current_time = time.time()
        dt = current_time - self.last_time

        location, _ = self.myArmUtilities.forward_kinematics(np.append(self.q_next, self.gamma))
        x, y, z = location[0], location[1], location[2]
        
        self.update_plot_elements(x, y, z)
        
        if current_time - self.last_drop_time >= self.DROP_INTERVAL and t < t_max:
            self.ax.plot([x, x], [y, y], [z, 0], color='purple', linestyle=':', alpha=0.3, linewidth=1.2)
            self.ax.plot([x], [y], [0], color='purple', marker='+', alpha=0.2, markersize=4)
            self.last_drop_time = current_time

        v_cmd_xyz = self.spline_velocity(t) + self.Kp * (self.spline(t) - location) 
        v_cmd = np.append(v_cmd_xyz, 0) 

        _, _, _, J_inv = self.myArmUtilities.differential_kinematics(self.q_next)
        q_dot = J_inv @ v_cmd 
        self.q_next = self.q_next + q_dot * dt 

        # Utilizes the new, safe writing interface
        self.myArmUtilities.write_to_arm(self.q_next, self.gripCmd)
        
        self.last_time = current_time
        
        return self.trail_graph, self.leading_dot, self.live_drop_line, self.live_ground_shadow

    def run(self):
        #self.plot_kinematic_profiles()
        self.setup_3d_plot()

        with QArm(hardware=self.hardware, readMode=0) as myArm:
            np.set_printoptions(precision=2, suppress=True)
            self.myArmUtilities.attach_QArm(myArm)

            self.myArmUtilities.write_to_arm(np.array([0, 0, 0, 0]), self.gripCmd)
            time.sleep(2)

            start_pos = self.position_anchors[0]
            current_joints = self.myArmUtilities.read_from_arm()
            _, self.q_next = self.myArmUtilities.inverse_kinematics(start_pos, self.gamma, current_joints)
            
            print(f"Moving to start position: {start_pos}/{self.q_next}...")
            self.myArmUtilities.write_to_arm(self.q_next, self.gripCmd)
            time.sleep(2.0) 

            self.startTime = time.time()
            self.last_time = time.time()  
            self.last_drop_time = time.time()

            ani = animation.FuncAnimation(
                self.fig, self._animation_update, fargs=(myArm,), 
                frames=300, interval=20, blit=False
            )
            plt.show()

# ---------------------------------------------------------
# Joint Space Controller
# ---------------------------------------------------------
class JointSpaceController(BaseQArmController):
    def __init__(self, waypoints, hardware):
        super().__init__(waypoints)
        self.phi_targets = []
        self.joint_spline = None
        self.last_drop_time = 0
        self.DROP_INTERVAL = 0.5 
        self.hardware = hardware

    def _animation_update(self, frame, myArm):
        t_max = self.time_anchors[-1]
        t = np.clip(self.elapsed_time(), 0.0, t_max)
        current_time = time.time()
        
        if t >= t_max:
            return self.trail_graph, self.leading_dot, self.live_drop_line, self.live_ground_shadow
            
        # 1. Interpolate the joint angles
        current_phi_cmd = self.joint_spline(t)
        
        # 2. Command the arm safely
        self.myArmUtilities.write_to_arm(current_phi_cmd, self.gripCmd)
        
        # 3. Read actual joints dynamically
        current_joints = self.myArmUtilities.read_from_arm()
        
        # 4. Forward Kinematics for plotting
        state_vector = np.append(current_joints, self.gamma)
        location, _ = self.myArmUtilities.forward_kinematics(state_vector)
        x, y, z = location[0], location[1], location[2]
        
        self.update_plot_elements(x, y, z)
        
        if current_time - self.last_drop_time >= self.DROP_INTERVAL and t < t_max:
            self.ax.plot([x, x], [y, y], [z, 0], color='purple', linestyle=':', alpha=0.3, linewidth=1.2)
            self.ax.plot([x], [y], [0], color='purple', marker='+', alpha=0.2, markersize=4)
            self.last_drop_time = current_time
            
        return self.trail_graph, self.leading_dot, self.live_drop_line, self.live_ground_shadow

    def run(self):
        self.setup_3d_plot()

        with QArm(hardware=self.hardware, readMode=0) as myArm:
            np.set_printoptions(precision=2, suppress=True)
            self.myArmUtilities.attach_QArm(myArm)

            self.myArmUtilities.write_to_arm(np.array([0, 0, 0, 0]), self.gripCmd)
            time.sleep(2)
            
            print("Pre-calculating joint coordinates for waypoints...")
            current_joints = self.myArmUtilities.read_from_arm()
            
            for pt in self.position_anchors:
                _, q_target = self.myArmUtilities.inverse_kinematics(pt, self.gamma, current_joints)
                print(f"{q_target}")
                self.phi_targets.append(q_target)
                current_joints = q_target # Update starting point for next IK calculation
            
            self.phi_targets = np.array(self.phi_targets)
            self.joint_spline = CubicSpline(self.time_anchors, self.phi_targets, axis=0, bc_type='clamped')
                
            print(f"Moving to starting position {self.phi_targets[0]}...")
            self.myArmUtilities.write_to_arm(self.phi_targets[0], self.gripCmd)
            time.sleep(2.0) 

            self.startTime = time.time()
            self.last_drop_time = time.time()

            ani = animation.FuncAnimation(
                self.fig, self._animation_update, fargs=(myArm,), 
                frames=300, interval=20, blit=False
            )
            plt.show()