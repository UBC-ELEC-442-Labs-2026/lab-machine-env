from pal.products.qarm import QArm
from hal.products.qarm import QArmUtilities
import numpy as np
import sys
import threading
import time

class QArm_Lab_interface(QArmUtilities):
    def __init__(self, QArm_obj=None):
        super().__init__()  # Initialize parent QArmUtilities class
        
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
        self.gripper_lock = threading.Lock()
        self.gripper_stall_counter = 0

        self.running = True
        self.update_thread = threading.Thread(target=self._update_arm_position, daemon=True)
        

    def attach_QArm(self, QArm_obj):
        self.QArm_attached = True
        self.myArm = QArm_obj
        self.update_thread.start()

    def _check_joint_movement(self, phi):
        # 1. Check joint limits
        for i in range(4):
            if phi[i] < self.joint_mins[i] or phi[i] > self.joint_maxs[i]:
                print(f"Phi: {phi}")
                return False

        # 2. Check workspace limits (append 0 for gamma to satisfy 5-element array requirement)
        state_vector = np.append(phi, 0)
        location, _ = self.forward_kinematics(state_vector)
        
        if location[2] < self.z_min:
            print(f"Position: {location}")
            return False
            
        return True

    def write_to_arm(self, phi):
        """
        Returns true if write successful, false otherwise.
        Halts the program if movement is unsafe.
        """

        if not self.QArm_attached:
            print("No QArm attached")
            return False

        # Move only if safe
        if self._check_joint_movement(phi):
            #self.myArm.read_write_std(phiCMD=phi, gprCMD=self.cache_gripper_pos, baseLED=(0, 1, 0))
            self.cache_arm_joints = phi
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
            pos = self.myArm.measJointPosition[4]
            vel = self.myArm.measJointSpeed[4]
            current = self.myArm.measJointCurrent[4]

            with self.gripper_lock:
                self._monitor_gripper(pos, vel, current)
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

        self.myArm.read_std()
        return self.myArm.measJointPosition[:4]
    
    def Jacobian(self, phi):
        J, _, _, _ = self.differential_kinematics(phi)
        return J
    
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

    def shutdown(self):
        """Cleanly stop the background thread loop"""
        self.running = False
        self.update_thread.join()
        

        
        


