## Multi-Color Box Sorting and Stacking System (V1 Architecture)

### 1. Goal

Build a demonstration-ready pipeline in simulation:

- Detect multiple colored boxes with RGB-D camera data.
- Pick boxes one by one with the manipulator.
- Sort by color and stack into neat piles.

### 2. Constraints and Design Choices

- The simulated camera pitch is fixed, so close-range target loss is expected.
- The manipulator has limited DOF for precise top-down grasping.
- We prioritize reliability and clear demo behavior over full dynamic planning.

Design strategy:

- Use color-based RGB-D detection for robust target acquisition.
- Use finite state machine (FSM) to keep behavior explainable and debuggable.
- Use open-loop short motion primitives for placement in predefined drop zones.

### 3. Node-Level Architecture

Single orchestrator node in V1:

- Node: stack_sort_pipeline.py
- Internal modules (class split inside one script):
  - Perception: color masks, contour extraction, center/depth estimation.
  - Task Planner: target selection and stack height bookkeeping.
  - Motion/Arm Controller: base velocity and manipulator command primitives.
  - Orchestrator FSM: end-to-end sequencing.

### 4. Data Flow

1. RGB image and depth image are subscribed.
2. Perception produces per-color detections: center, area, depth.
3. Planner chooses active target and computes drop-zone plan.
4. FSM executes pick and place with base and gripper commands.
5. Planner updates stack level for that color.

### 5. FSM (V1)

- SEARCH: rotate slowly and look for any valid box target.
- ALIGN: center selected target in camera view.
- APPROACH: move forward while keeping target centered.
- PICK_PREP: open gripper and set lift for pickup.
- PICK_PUSH: short blind push to put object inside gripper jaws.
- PICK_CLOSE: close gripper.
- PICK_LIFT: lift object.
- GO_DROP: move to color-specific drop zone.
- DROP: place object at stack height and open gripper.
- RETREAT: back off and return heading.
- FINISH: end after configured number of picks.

### 6. Iteration Roadmap

V1 (this commit):

- Multi-color scene assets and launch.
- Multi-color perception + sorting + stack bookkeeping.
- End-to-end FSM baseline.

V2 (implemented in current iteration):

- Add target-loss tolerance with bounded lost-cycle handling.
- Add retry and recovery policy for failed pick attempts.
- Add runtime metrics: attempts, retries, success/fail, cycle time.

V3:

- Replace open-loop drop motion with odometry-aware motion. (implemented)
- Add calibration and tuning profile auto-switch by scene profile.
- Add stack consistency scoring and run report export. (implemented with JSON export and anchor-error metrics)
