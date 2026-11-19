from pal.products.qarm import QArm
from hal.products.qarm import QArmUtilities
import numpy as np
import os
import sys
import threading
import time

_this_directory = os.path.dirname(os.path.abspath(__file__))
if _this_directory not in sys.path:
    sys.path.append(_this_directory)
import QArm_calibration as calib

class QArm_Lab_interface(QArmUtilities):
    def __init__(self, QArm_obj=None, calibration=False, hold_on_attach=False):
        """
        calibration : bool, default False
            On hardware, write_to_arm adjusts each commanded tip position by a calibrated amount (QArm_calibration.py).
            The floor check then checks the tip against QArm_calibration.MIN_TIP_Z0, including in simulation.
        hold_on_attach : bool, default False
            When the arm is attached, keep it where it is instead of sending it to all-zeros.
            Use travel_to() for the first move.
        """
        super().__init__()  # Initialize parent QArmUtilities class

        self.calibration = calibration
        self.hold_on_attach = hold_on_attach
        
        if QArm_obj is None:
            self.QArm_attached = False
            self.myArm = None
        else:
            self.QArm_attached = True
            self.myArm = QArm_obj

        # Joint limits
        self.joint_mins = np.radians(np.array([-170.0, -85.0, -95.0, -160.0]))
        self.joint_maxs = np.radians(np.array([ 170.0,  85.0,  75.0,  160.0]))

        # Workspace limits
        self.z_min = 0.05

        self.gripper_state = "OPEN"
        self.cache_arm_joints = [0.0, 0.0, 0.0, 0.0]
        self.cache_gripper_pos = 0.0 
        self.actual_arm_joints = [0.0, 0.0, 0.0, 0.0]
        self.gripper_lock = threading.Lock()
        self.gripper_stall_counter = 0

        self.running = True
        self.update_thread = threading.Thread(target=self._update_arm_position, daemon=True)
        

    def attach_QArm(self, QArm_obj):
        self.QArm_attached = True
        self.myArm = QArm_obj

        if self.hold_on_attach:
            # Command the pose the arm is already in, so it doesn't jump to all-zeros
            self.myArm.read_std()
            measured = np.array(self.myArm.measJointPosition[:4], dtype=np.float64)
            if np.all(np.isfinite(measured)):   # skip if the reading is garbage
                self.cache_arm_joints = measured.copy()
                self.actual_arm_joints = measured.copy()

        self.update_thread.start()

    def check_joint_limits(self, phi):
        for i in range(4):
            if phi[i] < self.joint_mins[i] or phi[i] > self.joint_maxs[i]:
                print(f"Phi: {phi}")
                return False
        return True

    def _check_joint_movement(self, phi):
        # 1. Check joint limits
        for i in range(4):
            if phi[i] < self.joint_mins[i] or phi[i] > self.joint_maxs[i]:
                print(f"Phi: {phi}")
                return False

        # 2. Check workspace limits (append 0 for gamma to satisfy 5-element array requirement)
        state_vector = np.append(phi, 0)
        location, _ = self.forward_kinematics(state_vector)
        
        floor = calib.MIN_TIP_Z0 if self.calibration else self.z_min
        if location[2] < floor:
            print(f"Tip height {location[2]:.3f} m is below the floor limit {floor:.3f} m (tip at {np.round(location, 3)})")
            return False
            
        return True

    def _calibrated_command(self, phi):
        """
        The angles to send so the real tip lands where phi puts it (calibration on, hardware only).
        Falls back to phi unchanged if the corrected position can't be reached.
        """
        # No correction in simulation, or when calibration is off
        hardware = getattr(self.myArm, "hardware", 0)   # 0 if myArm has no "hardware" attribute
        if not self.calibration or not hardware:
            return phi

        # Where phi puts the tip, and how far it is from the base axis
        p, _ = self.forward_kinematics(np.append(phi, 0))
        r = np.hypot(p[0], p[1])                                   # sqrt(x^2 + y^2)
        r = np.clip(r, calib.REACH_RANGE[0], calib.REACH_RANGE[1])  # stay inside the measured range

        # Raise the tip by the calibrated amount, then find angles for that point
        lift = calib.HEIGHT_OFFSET_C0 + calib.HEIGHT_OFFSET_C1 * r
        target = p + np.array([0.0, 0.0, lift])
        corrected = self._ik_near(target, phi[3], phi)

        if corrected is None:   # can't reach the raised point
            return phi
        return corrected

    def _ik_near(self, p, wrist, phi_near, max_jump=0.5):
        """
        IK solution for tip position p that is valid and closest to phi_near.
        Returns None if no solution reaches p, stays in the joint limits and is within max_jump rad.
        """
        candidates, _ = self.inverse_kinematics(p, wrist, phi_near)   # 4 solutions, one per column
        phi_near = np.asarray(phi_near, dtype=np.float64)

        best = None
        best_dist = np.inf
        for i in range(4):
            q = np.array(candidates[:, i], dtype=np.float64)
            q[3] = wrist

            # Skip solutions that are invalid or outside the joint limits
            if not np.all(np.isfinite(q)) or not self._within_joint_limits(q):
                continue

            # Skip solutions that don't actually reach p (within 1 mm)
            reached, _ = self.forward_kinematics(np.append(q, 0))
            if np.linalg.norm(reached - p) > 1e-3:
                continue

            # Keep the one closest to phi_near (base, shoulder, elbow only)
            dist = np.linalg.norm(q[:3] - phi_near[:3])
            if dist < best_dist:
                best = q
                best_dist = dist

        if best is None or best_dist > max_jump:
            return None
        return best

    def _within_joint_limits(self, phi):
        """check_joint_limits without the printout."""
        return bool(np.all(phi >= self.joint_mins) and np.all(phi <= self.joint_maxs))

    def write_to_arm(self, phi):
        """
        Returns true if write successful, false otherwise.
        Halts the program if movement is unsafe.
        """
        phi = phi.copy()  # Future phi changes don't affect output until next write_to_arm call

        if not self.QArm_attached:
            print("No QArm attached")
            return False

        # Move only if safe
        if self._check_joint_movement(phi):
            #self.myArm.read_write_std(phiCMD=phi, gprCMD=self.cache_gripper_pos, baseLED=(0, 1, 0))
            self.cache_arm_joints = self._calibrated_command(phi)
            return True
        
        print("Unsafe joint movement detected! Halting to prevent damage.")
        self.shutdown()
        sys.exit() # Hard stop if invalid movement detected
        return False
    
    def _update_arm_position(self):
        '''
        Updates the arm position based on the cache
        '''
        TARGET_FREQ = 25.0
        RATE = 1.0 / TARGET_FREQ
        next_tick = time.perf_counter()

        while(self.running):
            self.myArm.read_std() # update sensor buffers
            #print(f"{self.cache_arm_joints}")
            grip_pos = self.myArm.measJointPosition[4]
            grip_vel = self.myArm.measJointSpeed[4]
            grip_current = self.myArm.measJointCurrent[4]

            self.actual_arm_joints = self.myArm.measJointPosition[:4]

            with self.gripper_lock:
                self._monitor_gripper(grip_pos, grip_vel, grip_current)
                self.myArm.read_write_std(phiCMD=self.cache_arm_joints, gprCMD=self.cache_gripper_pos, baseLED=(0, 1, 0))

            next_tick += RATE
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_tick = time.perf_counter()

    def _monitor_gripper(self, pos, vel, current):
        gripper_current_max = 0.35
        gripper_current_target = 0.3 # target current when holding
        gripper_current_baseline = 0.1
        gripper_vel_baseline = 0.05

        if self.gripper_state == "CLOSING":
            # Watch for stalls
            if current > gripper_current_max:
                if self.gripper_stall_counter >= 15:
                    self.gripper_state = "CLOSED"
                    self.cache_gripper_pos = pos
                    self.gripper_stall_counter = 0
                else:
                    self.gripper_stall_counter += 1
            else:
                self.cache_gripper_pos = 0.9
                self.gripper_stall_counter = 0

        elif self.gripper_state == "OPEN":
            self.cache_gripper_pos = 0.1

        elif self.gripper_state == "CLOSED":
            # Keep the current at about 0.3A
            if current > gripper_current_target:
                self.cache_gripper_pos -= 0.01
            elif current < gripper_current_baseline:
                self.cache_gripper_pos += 0.01


    def read_from_arm(self):
        if not self.QArm_attached:
            print("No QArm attached")
            return None
        
        return self.actual_arm_joints.copy()  # Return a copy to avoid external modification
    
    def Jacobian(self, phi):
        J, _, _, _ = self.differential_kinematics(phi)
        return J
    
    def Inv_Jacobian(self, phi):
        _, _, _, J_inv = self.differential_kinematics(phi)
        return J_inv
    
    def close_gripper(self):
        '''
        Safely close the gripper by monitoring the servo current, stops when resistance detected
        '''
        
        if not self.QArm_attached:
            print("No QArm attached")
            return None
        
        with self.gripper_lock:
            self.gripper_state = "CLOSING"
            self.gripper_stall_counter = 0

    def open_gripper(self):
        if not self.QArm_attached:
            print("No QArm attached")
            return None

        with self.gripper_lock:
            self.gripper_state = "OPEN"
            self.gripper_stall_counter = 0

    def travel_to(self, phi_target, safe_z=0.35, tuck_r=0.25, speed=0.6):
        """
        Move to phi_target without sweeping through things on the table: lift the tip straight up
        to safe_z, pull it back to tuck_r from the base axis, turn the base, reach out above the
        target at safe_z, then come straight down. Blocks until the arm is there.

        phi_target : joint angles [base, shoulder, elbow, wrist] (rad)
        safe_z     : travel height of the tip (m); lowered automatically where the arm can't reach it
        tuck_r     : how far from the base axis the tip is while the base turns (m)
        speed      : largest joint speed during the move (rad/s)
        """
        if not self.QArm_attached:
            print("No QArm attached")
            return False

        phi_target = np.array(phi_target, dtype=np.float64)
        phi = np.array(self.read_from_arm(), dtype=np.float64)   # where the arm is now
        wrist_now = phi[3]
        wrist_end = phi_target[3]

        # Tip positions now and at the target
        p_now, _ = self.forward_kinematics(np.append(phi, 0))
        p_end, _ = self.forward_kinematics(np.append(phi_target, 0))
        angle_now = np.arctan2(p_now[1], p_now[0])   # direction the arm points now
        angle_end = phi_target[0]
        r_now = np.hypot(p_now[0], p_now[1])
        r_end = np.hypot(p_end[0], p_end[1])

        def above(angle, r, z_min):
            """Highest reachable point at or below safe_z (but not below z_min), at base angle, reach r."""
            # Try heights from the top down, 1 cm at a time
            for z in np.arange(max(safe_z, z_min), z_min - 1e-9, -0.01):
                p = np.array([r * np.cos(angle), r * np.sin(angle), z])
                if self._ik_near(p, wrist_now, phi, max_jump=np.inf) is not None:
                    return p
            return None

        # Work out the stops before moving
        lifted = above(angle_now, r_now, p_now[2])
        tucked = above(angle_now, min(tuck_r, r_now), 0.0)
        over_target = above(angle_end, r_end, p_end[2])

        # 1. Lift straight up
        q = phi
        if lifted is not None:
            q = self._line_move(q, lifted, q[3], speed)

        # 2. Pull back toward the base
        if tucked is not None:
            q = self._line_move(q, tucked, q[3], speed)

        # 3. Turn the base (and the wrist) toward the target
        q_turned = q.copy()
        if tucked is not None:
            q_turned[0] = angle_end
        q_turned[3] = wrist_end
        self._joint_move(q, q_turned, speed)
        q = q_turned

        # 4. Reach out above the target
        if over_target is not None:
            q = self._line_move(q, over_target, q[3], speed)

        # 5. Come straight down onto the target
        self._joint_move(q, phi_target, speed, line_to=p_end)
        time.sleep(0.5)
        return True

    def _joint_move(self, q_from, q_to, speed, line_to=None):
        """Straight line in joint space (or, with line_to, a straight tip line) at up to speed rad/s."""
        # Optionally move the tip in a straight line first
        if line_to is not None:
            q_last = self._line_move(q_from, line_to, q_to[3], speed)
            if np.allclose(q_last, q_to, atol=1e-3):   # already there
                return
            q_from = q_last

        # Split the move into small steps so no joint goes faster than speed
        dt = 0.04
        biggest_change = np.max(np.abs(q_to - q_from))
        n = max(1, int(np.ceil(biggest_change / (speed * dt))))

        for k in range(1, n + 1):
            self.write_to_arm(q_from + (q_to - q_from) * k / n)
            time.sleep(dt)

    def _line_move(self, q_from, p_to, wrist, speed):
        """Move the tip in a straight line to p_to. Returns the joint angles it ended at."""
        p_from, _ = self.forward_kinematics(np.append(q_from, 0))

        # Points every 7.5 mm along the line
        n = max(1, int(np.ceil(np.linalg.norm(p_to - p_from) / 0.0075)))
        dt = 0.04
        q = q_from

        for k in range(1, n + 1):
            p_next = p_from + (p_to - p_from) * k / n
            q_next = self._ik_near(p_next, wrist, q, max_jump=0.3)

            # Can't follow the line here: finish with a joint move instead
            if q_next is None:
                q_end = self._ik_near(p_to, wrist, q, max_jump=np.inf)
                if q_end is not None:
                    self._joint_move(q, q_end, speed)
                    return q_end
                return q

            # Move to the next point in small enough steps
            steps = max(1, int(np.ceil(np.max(np.abs(q_next - q)) / (speed * dt))))
            for j in range(1, steps + 1):
                self.write_to_arm(q + (q_next - q) * j / steps)
                time.sleep(dt)
            q = q_next

        return q

    def shutdown(self):
        """Cleanly stop the background thread loop"""
        self.running = False
        self.update_thread.join()
        

        
        


