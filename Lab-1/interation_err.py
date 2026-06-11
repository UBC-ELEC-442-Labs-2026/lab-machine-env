from pal.products.qarm import QArm
from hal.products.qarm import QArmUtilities
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

#region: Setup

# Circle Parameters
CIRCLE_CENTER = np.array([0.55, 0.0, 0.4]) # Centered at X=0.4, Y=0.0, Z=0.4
RADIUS = 0.15
OMEGA = 2.0 * np.pi / 3.0  # Angular velocity (1 full rotation every 3 seconds)
DRIFT_THRESHOLD = 0.4      # Stop when distance from center > 0.4m

# --- 3D ANIMATION SETUP ---
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(projection='3d')

# Trail and Leading Dot adjustments (Blue/RoyalBlue styling)
trail_graph, = ax.plot([], [], [], color='blue', alpha=0.5, linewidth=1.5, label="Drifting Trajectory Trail")
leading_dot, = ax.plot([], [], [], color='royalblue', marker='o', markersize=6, zorder=10)
animated_points = []

ax.view_init(elev=25, azim=45)
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(0, 1)

ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
ax.grid(False)

# Static origin crosshairs
ax.plot([-1, 1], [0, 0], [0, 0], color='gray', linewidth=1, linestyle='--') 
ax.plot([0, 0], [-1, 1], [0, 0], color='gray', linewidth=1, linestyle='--') 
ax.plot([0, 0], [0, 0], [0, 1], color='gray', linewidth=1, linestyle='--')  

ax.text(1.1, 0, 0, 'X', horizontalalignment='center', fontweight='bold')
ax.text(0, 1.1, 0, 'Y', horizontalalignment='center', fontweight='bold')
ax.text(0, 0, 1.1, 'Z', horizontalalignment='center', fontweight='bold')

# --- VISUAL REFERENCE: THE IDEAL TARGET CIRCLE ---
t_circle = np.linspace(0, 2*np.pi, 100)
ideal_y = CIRCLE_CENTER[1] + RADIUS * np.cos(t_circle)
ideal_z = CIRCLE_CENTER[2] + RADIUS * np.sin(t_circle)
ideal_x = np.full_like(t_circle, CIRCLE_CENTER[0])
ax.plot(ideal_x, ideal_y, ideal_z, color='forestgreen', linestyle='--', alpha=0.6, linewidth=1.5, label="Ideal Path")
ax.scatter(CIRCLE_CENTER[0], CIRCLE_CENTER[1], CIRCLE_CENTER[2], color='forestgreen', marker='X', s=50, label="Circle Center")

ax.legend(loc="upper left")

# Configuration Variables
q_next = None
gamma = 0
gripCmd = 0
ledCmd = np.array([0, 1, 0], dtype=np.float64)
myArmUtilities = QArmUtilities()

# Timing and Interval Parameters
startTime = 0
last_time = 0
ani = None # Initialized here so the update function can access it globally

def update(frame, myArm):
    global q_next, last_time, ani
    
    current_time = time.time()
    t = current_time - startTime
    dt = current_time - last_time
    last_time = current_time

    # 1. WRITE TARGET & REFRESH HARDWARE ENCODER DATA
    myArm.read_write_std(phiCMD=q_next, gprCMD=gripCmd, baseLED=ledCmd)
    
    # 2. CALCULATE ACTUAL POSITION VIA FORWARD KINEMATICS
    current_joints = myArm.measJointPosition[0:4]
    location, rotation = myArmUtilities.forward_kinematics(np.append(current_joints, gamma))
    x, y, z = location[0], location[1], location[2]
    
    # 3. CHECK FOR INTEGRATION ERROR BREAK CONDITION
    distance_from_center = np.linalg.norm(location - CIRCLE_CENTER)
    if distance_from_center > DRIFT_THRESHOLD:
        print(f"⚠️ Simulation Halted! Integration drift exceeded threshold: {distance_from_center:.3f}m")
        if ani is not None:
            ani.event_source.stop() # Freezes the animation but keeps the window open
        return trail_graph, leading_dot

    # 4. PATH TELEMETRY DRAWING
    animated_points.append(location)
    matrix = np.array(animated_points)
    trail_graph.set_data_3d(matrix[:, 0], matrix[:, 1], matrix[:, 2])
    leading_dot.set_data_3d([x], [y], [z])
    
    # 5. PURE OPEN-LOOP VELOCITY CALCULATIONS (No Kp Term!)
    # Desired circular trajectory components in the Y-Z plane
    v_y = -RADIUS * OMEGA * np.sin(OMEGA * t)
    v_z =  RADIUS * OMEGA * np.cos(OMEGA * t)
    v_cmd_xyz = np.array([0.0, v_y, v_z]) 
    
    v_cmd = np.append(v_cmd_xyz, 0) 

    # Differential Kinematics inversion
    _, _, _, J_inv = myArmUtilities.differential_kinematics(q_next)
    q_dot = J_inv @ v_cmd 

    # Pure numerical integration (Euler Step)
    q_next = q_next + q_dot * dt 
    
    return trail_graph, leading_dot

#endregion

with QArm(hardware=0, readMode=0) as myArm:
    np.set_printoptions(precision=2, suppress=True)

    # Move to the exact geometric starting position of the circle cycle (t=0)
    start_pos = np.array([CIRCLE_CENTER[0], CIRCLE_CENTER[1] + RADIUS, CIRCLE_CENTER[2]])
    
    print("Moving to circle perimeter starting point...")
    allPhi, q_next = myArmUtilities.inverse_kinematics(start_pos, gamma, myArm.measJointPosition[0:4])
    myArm.read_write_std(phiCMD=q_next, gprCMD=gripCmd, baseLED=ledCmd)
    time.sleep(2.0) 

    # Synchronize execution clocks
    startTime = time.time()
    last_time = time.time()  

    ani = animation.FuncAnimation(
        fig, 
        update, 
        fargs=(myArm,), 
        frames=1000, 
        interval=20, 
        blit=False
    )
    
    plt.show() # Code will pause here while the window is open
    myArm.terminate() # This will only execute after you manually close the graph window