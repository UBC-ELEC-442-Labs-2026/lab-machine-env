"""
Calibration constants for QArm_Lab_interface(calibration=True).

Measured in November 2026 on arm SN 63954, with the gripper empty and the arm at rest, by sweeping
the gripper tip out at constant heights of 5 to 15 cm and measuring it with callipers.
See the "QArm Calibration, Collision Model & RRT Tuning" doc for the method and the data.

Heights are in the arm's frame: z = 0 is the bottom of the arm's base, which sits on an 11 mm plate,
so the table is at z = -0.011 m.
"""

# On hardware, a commanded tip position is raised by HEIGHT_OFFSET_C0 + HEIGHT_OFFSET_C1 * r
# before it is sent, where r is the horizontal distance from the base axis (m).
HEIGHT_OFFSET_C0 = -0.0128   # m
HEIGHT_OFFSET_C1 = 0.1074    # m per m of reach
REACH_RANGE = (0.25, 0.75)   # m; r is clamped to this range (the sweep covered 0.25 to 0.70 m)

# Lowest allowed tip height when calibration is on (m, frame {0}: the bottom of the arm's base). 0.011 is 22 mm above the table,
# which leaves room for the calibration error (about 7 mm) and the bottom edge of the closed fingers.
MIN_TIP_Z0 = 0.011
TABLE_Z0 = -0.011
