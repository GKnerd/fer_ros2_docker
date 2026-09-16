# Robotics Stack 

Date: 2026-08-24

## Hardware Components

- Franka Emika Robot (FER), `libfranka` version $0.9.2$
- Jetson Orin AGX 64GB - PC driving FER with Real Time Kernel and modified Network Interface Card (NIC) settings:
- 3D LiDAR: Seyond Robin W / Falcon K - Workstation Monitoring 
- Intel Realsense D455 - Flange mounted, used for object detection and manipulation of detected objects.
- HTC Vive Trackers - Low-cost solution for a MoCap, so that human joint poses can be captured.

## Software Architecture

Custom made ROS2 Jazzy packages for the FER in simulation and reality. The packages for the real robot are a port of `multipanda_ros2`. 
`ros2_control`is used as a baseline framework for control. The code is split into two stacks, simulation and reality

### Robot 

- **Task Planning**
    - Behavior Trees that combine *skills* to a full task.
    - Human defines the task a priori. Trees are not generated at runtime. 
- **Skill Server**
    - Defines the skills a robot can execute. *Goal-Terminated Skills* and *Condition-terminated Skills* are available.
    - Goal-terminated Skills: *Pick*, *Place*, *MoveToPose*, *GoHome*, and *ControlGripper*
    - Condition-terminated Skills: *TrackTarget* to a not fixed target location (**not yet implemented**), *ReleaseGraspedObject* via some kind of condition being met (**not yet implemented, exact condition not yet specified**). 
- **Path Planning**
    - *Goal-terminated Skills* used the MoveIt2 Framework to plan against a predefined goal and a known end state.
    - MoveIt Task Constructor (MTC) for the skills *Pick* and *Place*
    - MoveGroup Interface for: *MoveToPose*, *GoHome*, and *ControlGripper*
    - *Condition-terminated Skills* do not use MoveIt as the planning happens live.
- **Executor (not yet implemented)**
    - Handles arbitration so that skills don't fight for concurrent resources.
    - If MoveIt is used, utilizes the `execute()` functionality it provides.
    -  
- **Control**
    - Different deterministic Controllers are utilized.
    - Implemented: 
        - effort-based joint trajectory controller 
        - effort-based cartesian impedance controller (CRISP) (**not implemented**)
    - Planned: 
        - effort-based Model Predictive Controller (MPC) (**not implemented**)
- **Object Detection - Not implemented**
    - Camera-based observation of the robot's manipulation ws.
    - No continuous tracking of objects possible. Robot makes a screenshot of the current manipulable objects and executes a task on the frozen belief it has. 
    - It is planned to make a skill out of this, so the skill server can actively call, when the robot should refresh its belief, i.e. perform object detection again.
    - Should be its own docker container
    - Needs to be hand-eye calibrated (use a custom package for this)
    - Needs a way to detect objects and provide 3D position (minimum) or pose if possible (AnyPose is a good fit).

### Safety

- **Speed and Separation Monitoring (SSM)**
    - SSM is being implemented. 
    - 3D LiDAR is utilized for workspace monitoring. The generated point cloud is going to track the humans. This has not been implemented yet. Project of a colleague, doing it via SDFs.
    - HTC Vive Trackers are used to track the human joint poses and provide inputs for the safety module. 
    - The module is  not ready.
        - No defined interface as to what SSM expects exactly as a topic to function (**open**)
        - No velocity scaling implemented
        - Hardware Experiments on how to calibrate SSM for our own system have also not been performed. 
    - It sits beside the robotics stack and plugs in at the control level directly. (**open - we don't know if this is feasible**)
    - SSM will not use only the linear mode, but also integral form of the equation.

### Environments

- Simulation: MuJoCo using the `mujoco_ros2_control` package provided by PAL robotics
- Real World: Real FER



## Open Questions

- Unclear how we perform grasping of unknown objects.
- No clear plan on how to implement live tracking of the human hand.

## Known Limitations 
- Camera cannot capture the manipulable objects continuously. It plans once against a frozen scene for *Goal-terminated skills*.
- **Handover** violates SSM, Power-Force Limitation is more suitable safety mode Will be tackled later.
- Sim and Real controllers are named differently in the stack. Needs to be aligned. Unnecessary controllers should be removed.
- Using named states for grasping is problematic. The real FER uses an action server for the gripper, which should take `width` and a tolerance `epsilon` to its planned gripping, along with the needed force.
- The real franka provides Wrench Measurements directly through Franka State, these have to be routed to a WrenchStamped message, so that we can implement an external force observation mechanism. `set_load()` needs to be called, after attaching an object to it here to keep the dynamic model correct. For the sim we need to add a virtual force/torque sensor to have the same capability.
- Contracts at the boundaries of all interfaces are not defined yet. 
- fer_world_model package needs to be renamed  (probably scene_manager) and keep an update on the items in the world. 


## Tasks 

How I would implement the Behavior Trees. 

### Handover a fixed number of objects in the workspace to the human

1) MoveToPose/ GoHome --> Good pose to take a screenshot of the WS
2) ObjectDetection Skill --> Identify all objects and plan against the poses or 3D positions
3) Pick first object
4) Use continuous handover skill
5) Grasp release
Perform this action for N objects.
6) Pick next object 
6a) Pick fails: MoveToPose again and recapture the ws, check if object is still present or not. 
6b) Pick succesfull: Go to 4.

### Sorting Task - Objects sorted to different bins

1) MoveToPose/ GoHome --> Good pose to take a screenshot of the WS
2) ObjectDetection Skill --> Identify all objects and plan against the poses or 3D positions
Repeat N times
3) Pick object
3a) Pick fails: MoveToPose again and recapture the ws, check if object is still present or not. 
3b) Pick succesful: Go to 4.
4) Place object 
