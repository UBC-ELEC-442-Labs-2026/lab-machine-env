import time
import numpy as np
import pygame  # Used for key constants and font rendering
from pal.products.qarm import QArm
from hal.products.qarm import QArmKeyboardNavigator, QArmUtilities
from pal.utilities.keyboard import PygameKeyboard 

if __name__ == "__main__":

    mode = "-1"
    while(int(mode) != 0 and int(mode) != 1):
        mode = input("Enter 1 for real hardware, 0 for simulation: ")

    # 1. Initialize the Pygame Keyboard
    kbd = PygameKeyboard()
    
    # Initialize the Pygame Font module so we can render text on the window
    pygame.font.init()
    font = pygame.font.SysFont("Consolas", 14)
    
    # Resize the window slightly to fit both joint angles and Cartesian coordinates
    screen = pygame.display.set_mode((650, 160))

    # Instantiate Quanser's utility class for Forward Kinematics
    arm_utils = QArmUtilities()

    # --- JOINT LIMIT DEFINITIONS (From Specification Sheet) ---
    # Converted from degrees to radians for mathematical comparisons
    JOINT_MINS = np.radians([-170.0, -85.0, -95.0, -160.0])
    JOINT_MAXS = np.radians([ 170.0,  85.0,  75.0,  160.0])

    # --- THE MONKEY-PATCH FIX + LIVE WINDOW RENDERING ---
    original_read = kbd.read
    
    # References so the patch can see the current manipulator state
    latest_phi = np.array([0.0, 0.0, 0.0, 0.0])
    current_active_idx = None  # Tracks navigator.active_joint (0 to 3)
    gripper_closed = False
    space_was_pressed = False
    
    def patched_read():
        global gripper_closed, space_was_pressed
        original_read()  # Run the original pygame key state grabber
        
        # Grab the raw pygame key states again
        keys = pygame.key.get_pressed()
        
        # Dynamically inject the missing attributes expected by QArmKeyboardNavigator
        kbd.k_1 = keys[pygame.K_1] or keys[pygame.K_KP1]
        kbd.k_2 = keys[pygame.K_2] or keys[pygame.K_KP2]
        kbd.k_3 = keys[pygame.K_3] or keys[pygame.K_KP3]
        kbd.k_4 = keys[pygame.K_4] or keys[pygame.K_KP4]

        # Spacebar Toggle Edge-Detection (changes state once per key down)
        if keys[pygame.K_SPACE]:
            if not space_was_pressed:
                gripper_closed = not gripper_closed
                space_was_pressed = True
        else:
            space_was_pressed = False

        # --- LIVE RENDER TO WINDOW ---
        angles_deg = latest_phi * 180.0 / np.pi
        
        # Use QArmUtilities to calculate location from the 4 joint angles + gamma=0
        fk_input = np.append(latest_phi, 0.0)
        ee_pos, _ = arm_utils.forward_kinematics(fk_input)
        
        # Clear the window with a clean dark-gray slate background
        screen.fill((30, 30, 30))
        
        # Render State Indicators
        grip_str = "CLOSED (1.0)" if gripper_closed else "OPEN (0.0)"
        grip_color = (240, 128, 128) if gripper_closed else (135, 206, 250)
        
        # Draw explicit UI text hints
        text_lines = [
            "Keys [1]-[4]: Select Joint  |  [UP]/[DOWN]: Move  |  [SPACE]: Toggle Gripper",
            f"Base (J1)    : {angles_deg[0]:6.1f}°",
            f"Shoulder (J2): {angles_deg[1]:6.1f}°",
            f"Elbow (J3)   : {angles_deg[2]:6.1f}°",
            f"Wrist (J4)   : {angles_deg[3]:6.1f}°",
            f"Gripper State: {grip_str}",
            f"EE Position  : X: {ee_pos[0]:.3f}m | Y: {ee_pos[1]:.3f}m | Z: {ee_pos[2]:.3f}m"
        ]
        
        for idx, text_line in enumerate(text_lines):
            if idx == 0:
                color = (200, 200, 200)
                display_string = text_line
            elif 1 <= idx <= 4:
                joint_idx = idx - 1
                if joint_idx == current_active_idx:
                    # Change color to orange/red if the active joint hits its physical limits
                    if latest_phi[joint_idx] <= JOINT_MINS[joint_idx] + 1e-4 or latest_phi[joint_idx] >= JOINT_MAXS[joint_idx] - 1e-4:
                        color = (255, 69, 0)  # Red-Orange warning color
                        display_string = f"-> {text_line} [LIMIT]"
                    else:
                        color = (50, 205, 50)  # Vibrant Lime Green for safe selection
                        display_string = f"-> {text_line}"
                else:
                    color = (150, 150, 150)
                    display_string = f"   {text_line}"
            elif idx == 5:
                color = grip_color
                display_string = f"   {text_line}"
            else:
                color = (238, 232, 170)  # Distinct Pale Goldenrod for XYZ coords
                display_string = f"   {text_line}"
                    
            text_surface = font.render(display_string, True, color)
            screen.blit(text_surface, (20, 12 + (idx * 19)))
            
        # Flip the display buffer to draw everything to the physical monitor window
        pygame.display.flip()

    # Swap out the read method with our patched version
    kbd.read = patched_read
    kbd.k_1 = kbd.k_2 = kbd.k_3 = kbd.k_4 = False
    # -----------------------------------------------------

    LOOP_RATE_HZ = 50.0
    TIMESTEP = 1.0 / LOOP_RATE_HZ
    
    ledCmd = np.array([0, 1, 0], dtype=np.float64)  

    print("Connecting to QArm... Ensure hardware/simulation is ready.")
    with QArm(hardware=int(mode), readMode=0) as myArm:
        np.set_printoptions(precision=2, suppress=True)

        initial_pose = myArm.measJointPosition[0:4]
        if np.allclose(initial_pose, 0.0):
            initial_pose = QArm.HOME_POSE

        # Set up the Navigator
        navigator = QArmKeyboardNavigator(keyboardDriver=kbd, initialPose=initial_pose)
        
        print("\n=== Joint Space Keyboard Controller Active ===")
        print("Telemetry, Gripper state, and EE Position redirected to the window display.")
        print("Safety limit protection filters active.")
        print("==============================================\n")

        try:
            while True:
                loop_start = time.time()

                # 1. Update active configurations
                kbd.read()

                print(f"{myArm.measJointCurrent}")

                # 2. Run control math logic and pipeline updates
                raw_target_phi = navigator.move_joints_with_keyboard(timestep=TIMESTEP)
                
                # --- APPLY JOINT LIMIT CLAMPING ---
                # Safely bind targets within the spec sheets' minimum and maximum limits
                target_phi = np.clip(raw_target_phi, JOINT_MINS, JOINT_MAXS)
                
                # Update the navigator's internal state tracker to match the clamped values
                # This ensures the keyboard response remains immediate when changing directions
                if hasattr(navigator, 'state_tracker') and hasattr(navigator.state_tracker, 'phi'):
                    navigator.state_tracker.phi[0:4] = target_phi
                
                # Push values out to global scope variables for the patch renderer to unpack
                latest_phi = target_phi
                current_active_idx = navigator.active_joint
                
                # Map boolean gripper state to standard hardware float command
                gripCmd = 1.0 if gripper_closed else 0.0
                
                # 3. Write standard output command configurations out to physical hardware channels
                myArm.read_write_std(phiCMD=target_phi, gprCMD=gripCmd, baseLED=ledCmd)

                # Consistent Pacing Maintenance
                elapsed = time.time() - loop_start
                if elapsed < TIMESTEP:
                    time.sleep(TIMESTEP - elapsed)

        except KeyboardInterrupt:
            print("\nShutting down control loop...")
        
        finally:
            kbd.terminate()
            myArm.terminate()
            print("Session ended successfully.")