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

    def attach_QArm(self, QArm_obj):
        self.QArm_attached = True
        self.myArm = QArm_obj

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

    def write_to_arm(self, phi, gripper = 1):
        """
        Returns true if write successful, false otherwise.
        Halts the program if movement is unsafe.
        """
        if not self.QArm_attached:
            print("No QArm attached")
            return False

        # Move only if safe
        if self._check_joint_movement(phi):
            self.myArm.read_write_std(phiCMD=phi, gprCMD=gripper, baseLED=(0, 1, 0))
            return True
        
        print("Unsafe joint movement detected! Halting to prevent damage.")
        sys.exit() 
        return False

    def read_from_arm(self):
        if not self.QArm_attached:
            print("No QArm attached")
            return None

        self.myArm.read_std()
        return self.myArm.measJointPosition[:4]
    
    def Jacobian(self, phi):
        J, _, _, _ = self.differential_kinematics(phi)
        return J


