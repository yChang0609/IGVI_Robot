#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# show_base_link.sh
#
# Starts robot_state_publisher (URDF model) + Foxglove bridge so you can open
# Foxglove Studio and see the 3D robot with coordinate axes.
#
# HOW TO USE
# ──────────
# 1. Run this script:
#       ./scripts/show_base_link.sh
#
# 2. Open Foxglove Studio → connect to ws://localhost:8765
#
# 3. Add a "3D" panel. In its settings:
#      • Add "URDF" display → URL: package://wildbot-car-description/urdf/kros_car.xacro
#        (or paste the URDF text from docker/compose/custom_configs/kros_car.xacro)
#      • Enable "TF" display → you will see red/green/blue axes at every frame,
#        including base_link (the robot body origin).
#
# 4. The base_link origin = the point where the three RGB axes cross at the
#    robot's body center. Compare this visually to where your Kinect is mounted.
#
# 5. Measure from base_link to where the Kinect's front-face centre is:
#      X = distance forward  (+ = forward, - = backward)
#      Y = distance sideways (+ = left,    - = right   )
#      Z = height            (+ = up,      - = down    )
#    Tilt: if the Kinect lens faces slightly downward, pitch is NEGATIVE.
#    (e.g. 10° tilt down = -0.175 rad).  Level and facing forward = 0.
#
# 6. Update docker/compose/compose.yaml with your measured values:
#       camera_mount_x:=<X>
#       camera_mount_y:=<Y>      # left of Gemini → positive number
#       camera_mount_z:=<Z>
#       camera_mount_pitch:=<pitch_rad>   # tilt down = negative value
#
# ─────────────────────────────────────────────────────────────────────────────
source "$(dirname "${BASH_SOURCE[0]}")/utils.sh"
main monitoring debug
