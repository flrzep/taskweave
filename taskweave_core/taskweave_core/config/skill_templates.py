"""skills/<skill>.yaml of the new skills in the new layout (skill-api-spec 2.2): what ``taskweave config
migrate-skills`` starts from before it lays the cell's own values (speeds, tolerances, ...) over them. They are the
files of docs/examples/skills/; test_migrate_skills keeps the two in step."""

MOVE_GRIPPER = """\
# skills/move_gripper.yaml (skill-api-spec 2.2). Replaces go_to_point.yaml and the tool-pose part of move_to_pose.yaml:
# the old flat keys were renamed (taskweave config migrate-skills): max_joint_speed -> speed.joint_rad_s,
# max_linear_speed -> speed.joint_linear_m_s, tcp_speed -> speed.tcp_m_s, tcp_turn -> speed.tcp_rad_s,
# position_tolerance / stall_timeout_s / state_timeout_s -> tolerances.*, lift_joint / lift_values -> torso_fallback,
# approach_m -> motion.approach_m.
# The first implementation whose requires fits the robot is served (the same class twice: two arms, then one arm).
implementations:
  - name: ik_planned            # IK for the target from where the robot stands, MoveIt-planned path
    type: script
    class: taskweave_skills.gripper_motion:MoveGripper
    requires: {capabilities: [arms.bimanual]}
  - name: ik_single
    type: script
    class: taskweave_skills.gripper_motion:MoveGripper
    requires: {capabilities: [arms.single]}
cell_frame: world               # TF frame of the cell (= cell/objects.yaml poses)
speed:                          # standard speeds; a step's motion.speed (fraction) or speed_abs scales them
  joint_rad_s: 1.2              # rad/s (used without a motion planner)
  joint_linear_m_s: 0.25        # m/s of prismatic joints
  tcp_m_s: 0.05                 # m/s on straight lines
  tcp_rad_s: 0.3                # rad/s on straight lines
accel: {joint_rad_s2: 3.0, tcp_m_s2: 0.2}   # a step's motion.accel (fraction) lengthens the shortest move
# defaults of the step's motion options; a robot pack's skill_overrides.yaml, the point's own approach_m / elbow and
# the step's motion override them, in that order
motion: {mode: free, planner: auto, approach_m: 0.10, torso: auto}
torso_fallback:                 # torso: auto: a target out of reach at this lift is tried at these heights (m), in order
  joint: body_joint
  values: [0.0, 0.2, 0.4, 0.6, 0.75]
tolerances:
  position: 0.03                # rad or m of a joint at the end of a move
  settle_s: 0.3
  stall_timeout_s: 5.0          # sim time
  state_timeout_s: 5.0          # no joint states this long (wall time) -> fail; rides out sim stalls (UI asset loads)
"""

GRIP = """\
# skills/grip.yaml (skill-api-spec 1.5). Replaces gripper.yaml: max_joint_speed -> speed.joint_rad_s,
# position_tolerance / stall_timeout_s / state_timeout_s -> tolerances.*.
implementations:
  - name: joint_targets
    type: script
    class: taskweave_skills.gripper_motion:Grip
    requires: {capabilities: [gripper.parallel]}
cell_frame: world               # TF frame of the cell (place_as)
speed: {joint_rad_s: 1.5}       # rad on the driving finger joint per second
accel: {joint_rad_s2: 3.0}
tolerances:
  position: 0.03
  stall_timeout_s: 1.0          # after the planned close: blocked (or creeping) this long -> holding
  state_timeout_s: 5.0          # no joint states this long (wall time) -> fail; rides out sim stalls (UI asset loads)
"""

MOVE_OBJECT = """\
# skills/move_object.yaml (skill-api-spec 1.2, 2.2): the new move_object (args: to / object / hands / motion). The old
# max_joint_speed / max_linear_speed / tcp_speed / tcp_turn were renamed as in move_gripper.yaml; the old elbow argument
# is the set_posture skill now.
implementations:
  - name: ik_held               # the hands that hold the object move it, IK from where they are
    type: script
    class: taskweave_skills.object_motion:MoveObject
    requires: {capabilities: [arms.bimanual]}
  - name: ik_held_single
    type: script
    class: taskweave_skills.object_motion:MoveObject
    requires: {capabilities: [arms.single]}
cell_frame: world               # TF frame of the cell (to.onto contacts are placed objects of cell/objects.yaml)
speed:                          # standard speeds; a step's motion.speed (fraction) or speed_abs scales them
  joint_rad_s: 1.2
  joint_linear_m_s: 0.25
  tcp_m_s: 0.05                 # m/s of the hands on straight lines (mode linear)
  tcp_rad_s: 0.3
accel: {joint_rad_s2: 3.0, tcp_m_s2: 0.2}
motion: {mode: linear, planner: auto}   # the step's motion overrides these
grip_drift_m: 0.02              # mode free with two or more hands: the hands may drift this far apart on the path
tolerances:
  position: 0.03
  settle_s: 0.3
  stall_timeout_s: 5.0
  state_timeout_s: 5.0
"""

SET_POSTURE = """\
# skills/set_posture.yaml (skill-api-spec 1.3). New; it is the old move_object elbow argument. The hands stay where
# they are, the elbows swing toward the given side along the arms' null space.
implementations:
  - name: null_space
    type: script
    class: taskweave_skills.object_motion:SetPosture
    requires: {capabilities: [arms.bimanual]}
  - name: null_space_single
    type: script
    class: taskweave_skills.object_motion:SetPosture
    requires: {capabilities: [arms.single]}
cell_frame: world
speed: {joint_rad_s: 1.2}       # rad/s of the swing; a step's motion.speed or speed_abs scales it
accel: {joint_rad_s2: 3.0}
elbow_tolerance_deg: 20.0       # a swing that would stop further than this from the side fails before moving
tolerances:
  position: 0.03
  settle_s: 0.3
  stall_timeout_s: 5.0
  state_timeout_s: 5.0
"""

MOVE_JOINTS = """\
# skills/move_joints.yaml (skill-api-spec 1.4). Replaces move_to_pose.yaml for joint poses: max_joint_speed ->
# speed.joint_rad_s, max_linear_speed -> speed.joint_linear_m_s, position_tolerance / stall_timeout_s /
# state_timeout_s -> tolerances.*.
implementations:
  - name: joint_targets         # joint-space moves; with a planner in the robot profile the path avoids collisions
    type: script
    class: taskweave_skills.joint_motion:MoveJoints
speed:                          # standard speeds; a step's motion.speed (fraction) or speed_abs scales them
  joint_rad_s: 1.2              # rad/s
  joint_linear_m_s: 0.25        # m/s for prismatic joints (profile joints.linear)
accel: {joint_rad_s2: 3.0}
motion: {planner: auto}         # the step's motion overrides these
tolerances:
  position: 0.03
  settle_s: 0.3
  stall_timeout_s: 5.0          # sim time
  state_timeout_s: 5.0          # no joint states this long (wall time) -> fail; rides out sim stalls (UI asset loads)
"""

TEMPLATES = {'move_gripper': MOVE_GRIPPER, 'grip': GRIP, 'move_object': MOVE_OBJECT, 'set_posture': SET_POSTURE,
             'move_joints': MOVE_JOINTS}
