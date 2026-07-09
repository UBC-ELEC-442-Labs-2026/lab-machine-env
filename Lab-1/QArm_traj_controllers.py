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
        self.gamma = 0  # Still defined here if needed for internal FK state tracking
        
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

        J_inv = self.myArmUtilities.Inv_Jacobian(self.q_next)
        q_dot = J_inv @ v_cmd 
        self.q_next = self.q_next + q_dot * dt 

        # Utilizes the updated single parameter write_to_arm structure
        self.myArmUtilities.write_to_arm(self.q_next)
        
        self.last_time = current_time
        
        return self.trail_graph, self.leading_dot, self.live_drop_line, self.live_ground_shadow

    def run(self):
        self.setup_3d_plot()

        with QArm(hardware=self.hardware, readMode=0) as myArm:
            np.set_printoptions(precision=2, suppress=True)
            self.myArmUtilities.attach_QArm(myArm)

            self.myArmUtilities.write_to_arm(np.array([0.0, 0.0, 0.0, 0.0]))
            time.sleep(2)

            start_pos = self.position_anchors[0]
            current_joints = self.myArmUtilities.read_from_arm()
            _, self.q_next = self.myArmUtilities.inverse_kinematics(start_pos, self.gamma, current_joints)
            
            print(f"Moving to start position: {start_pos}/{self.q_next}...")
            self.myArmUtilities.write_to_arm(self.q_next)
            time.sleep(2.0) 

            self.startTime = time.time()
            self.last_time = time.time()  
            self.last_drop_time = time.time()

            ani = animation.FuncAnimation(
                self.fig, self._animation_update, fargs=(myArm,), 
                frames=300, interval=20, blit=False
            )
            plt.show()
            self.myArmUtilities.shutdown()

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
        
        # 2. Command the arm safely via single joint vector parameter signature
        self.myArmUtilities.write_to_arm(current_phi_cmd)
        
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

            self.myArmUtilities.write_to_arm(np.array([0.0, 0.0, 0.0, 0.0]))
            time.sleep(2)
            
            print("Pre-calculating joint coordinates for waypoints...")
            current_joints = self.myArmUtilities.read_from_arm()
            
            for pt in self.position_anchors:
                _, q_target = self.myArmUtilities.inverse_kinematics(pt, self.gamma, current_joints)
                print(f"{q_target}")
                self.phi_targets.append(q_target)
                current_joints = q_target 
            
            self.phi_targets = np.array(self.phi_targets)
            self.joint_spline = CubicSpline(self.time_anchors, self.phi_targets, axis=0, bc_type='clamped')
                
            print(f"Moving to starting position {self.phi_targets[0]}...")
            self.myArmUtilities.write_to_arm(self.phi_targets[0])
            time.sleep(2.0) 

            self.startTime = time.time()
            self.last_drop_time = time.time()

            ani = animation.FuncAnimation(
                self.fig, self._animation_update, fargs=(myArm,), 
                frames=300, interval=20, blit=False
            )
            plt.show()
            self.myArmUtilities.shutdown()


# ---------------------------------------------------------
# Letter Drawing Trajectory Controller
# ---------------------------------------------------------
class LetterTrajectoryController(BaseQArmController):
    FONT = {
        'A': [[(0, 0), (1, 0.5), (0, 1)], [(0.4, 0.2), (0.4, 0.8)]],
        'B': [[(0, 0), (1, 0), (1, 0.5), (0, 0.5)], [(0.5, 0.5), (0.5, 1), (0, 1), (0, 0)], [(0, 0), (0, 1)]],
        'C': [[(1, 1), (1, 0), (0, 0), (0, 1)]],
        'D': [[(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)]],
        'E': [[(0, 1), (0, 0), (1, 0)], [(0.5, 0), (0.5, 0.7)], [(1, 0), (1, 1)]],
        'F': [[(0, 0), (1, 0)], [(0.5, 0), (0.5, 0.7)], [(1, 0), (1, 1)]],
        'G': [[(0.5, 0.5), (0, 0.5), (0, 0), (1, 0), (1, 1), (0.5, 1)]],
        'H': [[(0, 0), (1, 0)], [(0.5, 0), (0.5, 1)], [(0, 1), (1, 1)]],
        'I': [[(0, 0.5), (1, 0.5)], [(1, 0), (1, 1)], [(0, 0), (0, 1)]],
        'J': [[(0.2, 0), (0, 0.3), (0, 0.7), (1, 0.7)], [(1, 0.3), (1, 1)]],
        'K': [[(0, 0), (1, 0)], [(0.5, 0), (0, 0.7)], [(0.5, 0), (1, 0.7)]],
        'L': [[(1, 0), (0, 0), (0, 1)]],
        'M': [[(0, 0), (1, 0), (0.5, 0.5), (1, 1), (0, 1)]],
        'N': [[(0, 0), (1, 0), (0, 1), (1, 1)]],
        'O': [[(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)]],
        'P': [[(0, 0), (1, 0), (1, 1), (0.5, 1), (0.5, 0)]],
        'Q': [[(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)], [(0.2, 0.2), (-0.2, -0.2)]],
        'R': [[(0, 0), (1, 0), (1, 1), (0.5, 1), (0.5, 0)], [(0.5, 0.5), (0, 1)]],
        'S': [[(0, 0), (0.2, 1), (0.5, 1), (0.8, 0), (1, 0)]],
        'T': [[(0, 0.5), (1, 0.5)], [(1, 0), (1, 1)]],
        'U': [[(1, 0), (0, 0), (0, 1), (1, 1)]],
        'V': [[(1, 0), (0, 0.5), (1, 1)]],
        'W': [[(1, 0), (0, 0.25), (0.5, 0.5), (0, 0.75), (1, 1)]],
        'X': [[(0, 0), (1, 1)], [(1, 0), (0, 1)]],
        'Y': [[(0.5, 0.5), (0, 0.5)], [(0.5, 0.5), (1, 0)], [(0.5, 0.5), (1, 1)]],
        'Z': [[(1, 0), (1, 1), (0, 0), (0, 1)]]
    }

    def __init__(self, initials_str, hardware):
        """
        initials_str: e.g., "EL" (exactly two alphabet characters)
        """
        self.hardware = hardware
        self.initials = initials_str.upper()[:2]
        
        # Geometrical placement adjustments to trace parallel to the ground
        self.z_draw = 0.1   # Drawing surface (safely 1cm above the 0.05 hard stop threshold)
        self.z_lift = 0.15   # Safe hover height between strokes
        self.scale = 0.08    # Total height bounding dimension for the letters
        self.x_fixed = 0.40  # Flat operational depth from the base center
        
        # Arrange letters left-to-right along the Y axis
        self.letter_configs = [
            {"letter": self.initials[0], "y_offset": -0.11},
            {"letter": self.initials[1], "y_offset": 0.03}
        ]
        
        raw_strokes = self._generate_all_strokes()
        self.spline_segments = self._build_piecewise_splines(raw_strokes)
        
        dummy_waypoints = [[0.5, 0, 0.3, 0], [0.5, 0, 0.3, 3]]
        super().__init__(dummy_waypoints)
        
        self.Kp = 1.8
        self.q_next = None
        self.last_drop_time = 0
        self.DROP_INTERVAL = 0.25

    def _generate_all_strokes(self):
        compiled_strokes = []
        for config in self.letter_configs:
            char = config["letter"]
            y_off = config["y_offset"]
            if char not in self.FONT:
                continue
                
            for stroke in self.FONT[char]:
                stroke_points = []
                for pt in stroke:
                    # Map font vertical scale directly to X (depth) to write flat on the table
                    target_x = self.x_fixed + (pt[0] * self.scale)
                    target_y = y_off + (pt[1] * self.scale)
                    target_z = self.z_draw
                    stroke_points.append([target_x, target_y, target_z])
                compiled_strokes.append(stroke_points)
        return compiled_strokes

    def _build_piecewise_splines(self, raw_strokes):
        segments = []
        current_time = 0.0
        
        if not raw_strokes:
            return segments
            
        last_pos = [self.x_fixed, raw_strokes[0][0][1], self.z_lift]
        
        for stroke in raw_strokes:
            # --- Segment A: Travel through the air ---
            start_air = last_pos
            end_air = [self.x_fixed, stroke[0][1], self.z_lift]
            
            dist_air = np.linalg.norm(np.array(end_air) - np.array(start_air))
            dur_air = max(0.8, dist_air * 5.0)
            
            wps_air = np.array([start_air, end_air])
            ts_air = np.array([current_time, current_time + dur_air])
            
            segments.append({
                "spline": CubicSpline(ts_air, wps_air, axis=0, bc_type='clamped'),
                "t_start": current_time,
                "t_end": current_time + dur_air
            })
            current_time += dur_air
            
            # --- Segment B: Lower pen to surface ---
            start_drop = end_air
            end_drop = stroke[0]
            dur_drop = 0.5  
            
            wps_drop = np.array([start_drop, end_drop])
            ts_drop = np.array([current_time, current_time + dur_drop])
            
            segments.append({
                "spline": CubicSpline(ts_drop, wps_drop, axis=0, bc_type='clamped'),
                "t_start": current_time,
                "t_end": current_time + dur_drop
            })
            current_time += dur_drop
            
            # --- Segment C: Sketch active stroke point-by-point with DWELL ---
            for i in range(len(stroke) - 1):
                pt_start = stroke[i]
                pt_end = stroke[i+1]
                
                # 1. Trace the straight line segment
                dist_line = np.linalg.norm(np.array(pt_end) - np.array(pt_start))
                dur_line = max(0.4, dist_line * 10.0) 
                
                wps_line = np.array([pt_start, pt_end])
                ts_line = np.array([current_time, current_time + dur_line])
                
                segments.append({
                    "spline": CubicSpline(ts_line, wps_line, axis=0, bc_type='clamped'),
                    "t_start": current_time,
                    "t_end": current_time + dur_line
                })
                current_time += dur_line
                
                # 2. Add a Dwell/Pause at the vertex to let the physical arm settle
                # (We don't need a pause on the very last point, because the pen lifts immediately after)
                if i < len(stroke) - 2:
                    dur_pause = 0.3  # 300ms pause. Increase this if corners are still slightly rounded!
                    wps_pause = np.array([pt_end, pt_end])
                    ts_pause = np.array([current_time, current_time + dur_pause])
                    
                    segments.append({
                        "spline": CubicSpline(ts_pause, wps_pause, axis=0, bc_type='clamped'),
                        "t_start": current_time,
                        "t_end": current_time + dur_pause
                    })
                    current_time += dur_pause
            
            # --- Segment D: Lift pen ---
            start_lift = stroke[-1]
            end_lift = [self.x_fixed, stroke[-1][1], self.z_lift]
            dur_lift = 0.5
            
            wps_lift = np.array([start_lift, end_lift])
            ts_lift = np.array([current_time, current_time + dur_lift])
            
            segments.append({
                "spline": CubicSpline(ts_lift, wps_lift, axis=0, bc_type='clamped'),
                "t_start": current_time,
                "t_end": current_time + dur_lift
            })
            current_time += dur_lift
            last_pos = end_lift
            
        return segments

    def _get_active_target(self, t):
        for seg in self.spline_segments:
            if seg["t_start"] <= t <= seg["t_end"]:
                pos = seg["spline"](t)
                vel = seg["spline"].derivative()(t)
                return pos, vel
        final_spline = self.spline_segments[-1]["spline"]
        t_final = self.spline_segments[-1]["t_end"]
        return final_spline(t_final), np.zeros(3)

    def _animation_update(self, frame, myArm):
        t_max = self.spline_segments[-1]["t_end"] if self.spline_segments else 0
        t = np.clip(self.elapsed_time(), 0.0, t_max)
        
        current_time = time.time()
        dt = current_time - self.last_time
        
        location, _ = self.myArmUtilities.forward_kinematics(np.append(self.q_next, self.gamma))
        x, y, z = location[0], location[1], location[2]
        
        self.update_plot_elements(x, y, z)
        
        if current_time - self.last_drop_time >= self.DROP_INTERVAL and t < t_max:
            dot_color = 'crimson' if z < (self.z_draw + 0.01) else 'gray'
            self.ax.scatter(x, y, z, color=dot_color, marker='.', s=15, alpha=0.6)
            self.last_drop_time = current_time

        if t >= t_max:
            return self.trail_graph, self.leading_dot, self.live_drop_line, self.live_ground_shadow

        target_pos, target_vel = self._get_active_target(t)
        
        v_cmd_xyz = target_vel + self.Kp * (target_pos - location)
        v_cmd = np.append(v_cmd_xyz, 0)
        
        J_inv = self.myArmUtilities.Inv_Jacobian(self.q_next)
        q_dot = J_inv @ v_cmd
        self.q_next = self.q_next + q_dot * dt
        
        # Adjusted parameter write call
        self.myArmUtilities.write_to_arm(self.q_next)
        self.last_time = current_time
        
        return self.trail_graph, self.leading_dot, self.live_drop_line, self.live_ground_shadow

    def run(self):
        self.setup_3d_plot()
        self.ax.set_xlim(0.1, 0.6)
        self.ax.set_ylim(-0.25, 0.25)
        self.ax.set_zlim(0.0, 0.4)
        self.ax.view_init(elev=35, azim=30)
        plt.title(f"QArm Script Autopilot Demo: Drawing '{self.initials}'")

        with QArm(hardware=self.hardware, readMode=0) as myArm:
            print("running")
            self.myArmUtilities.attach_QArm(myArm)
            print("running2")
            
            # --- Marker Calibration Stage ---
            prep_pos = np.array([0.40, 0.0, self.z_draw])
            current_joints = self.myArmUtilities.read_from_arm()
            
            _, prep_joints = self.myArmUtilities.inverse_kinematics(prep_pos, self.gamma, current_joints)
            
            print(f"\nHoming to marker insertion point: {prep_pos}...")
            self.myArmUtilities.write_to_arm(prep_joints)
            #self.myArmUtilities.open_gripper()
            
            # Allow a fluid 2.5 second non-blocking window for arrival
            arrival_start = time.time()
            while time.time() - arrival_start < 2.5:
                time.sleep(0.05) # Small micro-yields keep the background driver loop flowing
            
            print("\n" + "="*50)
            print("  READY FOR MARKER INSERTION!")
            print("  Place the marker inside the gripper claws now.")
            print("="*50)
            
            # Non-blocking interactive countdown
            for countdown in range(4, 0, -1):
                print(f"  Locking gripper in {countdown} seconds...")
                step_start = time.time()
                while time.time() - step_start < 1.0:
                    time.sleep(0.05) # Prevents the main thread from locking up the I/O driver
                
            print("\n  Securing marker...")
            self.myArmUtilities.close_gripper()
            
            # Yield gracefully to let the stall/current monitoring lock onto the pen
            latch_start = time.time()
            while time.time() - latch_start < 2.0:
                time.sleep(0.05)
                
            print("  Marker clamped. Transitioning to trajectory start...\n")
            # ---------------------------------
            
            # Setup initial stroke coordinate path properties cleanly
            first_pos = self.spline_segments[0]["spline"](0.0)
            current_joints = self.myArmUtilities.read_from_arm()
            _, self.q_next = self.myArmUtilities.inverse_kinematics(first_pos, self.gamma, current_joints)
            
            self.myArmUtilities.write_to_arm(self.q_next)
            
            settle_start = time.time()
            while time.time() - settle_start < 2.0:
                time.sleep(0.05)
            
            self.startTime = time.time()
            self.last_time = time.time()
            self.last_drop_time = time.time()
            
            ani = animation.FuncAnimation(
                self.fig, self._animation_update, fargs=(myArm,),
                frames=450, interval=20, blit=False
            )
            plt.show()
            self.myArmUtilities.shutdown()