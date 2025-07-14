#  Service Robot Behavior with SMACH & YOLOv4 (ROS)

This project implements a high-level robot behavior using **ROS**, **SMACH**, and **YOLOv4** for a simulated TurtleBot3 patrolling a smart home environment. The robot ensures house rules are respected during a party by monitoring room occupancy and identifying people, cats, and dogs via video input.

---

## Behavior Summary

The robot must enforce two rules:
1. **No people in room D (kitchen)**
2. **Cats and dogs cannot be in the same room**

The robot alternates between rooms A and B, navigates each for 30 seconds, and periodically checks room D for rule violations. The entire logic is built using a **SMACH state machine** with concurrent states, ROS services, YOLO detection, and actionlib feedback.

---

##  Architecture Overview

| Component | Role |
|----------|------|
| `move_robot_server.py` | ROS service that moves the robot to room A, B, or D |
| `Robot_Behavior_Server.py` | SMACH-based actionlib server controlling high-level logic |
| `Go_To_RoomState.py` | SMACH state to move robot via service call |
| `Navigate_In_Room_State.py` | Drives the robot randomly inside a room for 30 seconds |
| `Decide_Next_Room_State.py` | Chooses the next room to visit and manages patrol cycles |
| `Yolo_Detection_State.py` | Runs YOLO object detection in real time and publishes feedback if rules are broken |
| `yolo_detection.py` | Provides YOLO detection as a ROS service using pre-trained YOLOv4 |
| `setroom.py` | Utility script to quickly set robot start location for testing |

---

##  Core Technologies

- **ROS (Melodic/Noetic)** — Robot Operating System for messaging and control
- **SMACH** — Python library for finite state machines in ROS
- **YOLOv4** — Real-time object detection (dog, cat, person)
- **Actionlib** — Used to manage goal execution, feedback, and results
- **TTS (Text-to-Speech)** — Publishes voice alerts when rules are broken

---

##  Key Features

- ✅ **SMACH State Machine** with concurrent YOLO and Navigation states
- ✅ **YOLO detection service** with object filtering and coordinate extraction
- ✅ **ROS Actionlib server** with structured feedback (`robot position` and `rule ID`)
- ✅ **Speech publishing** for rule-breaking announcements
- ✅ **Launch file and parameterization** (`nchecks`, `vid_folder`)

---

## File Structure

robot-behavior-smach-yolo/
├── move_robot_server.py # Room navigation service (ROS)
├── move_robot_service_client.py # Service client (test script)
├── setroom.py # Set initial room position
├── Robot_Behavior_Server.py # Main actionlib + SMACH controller
├── Go_To_RoomState.py # SMACH state to invoke movement
├── Navigate_In_Room_State.py # State for patrolling inside a room
├── Decide_Next_Room_State.py # Chooses next room / terminates
├── Yolo_Detection_State.py # Concurrent SMACH state using YOLO
├── yolo_detection.py # Runs YOLOv4 ROS service
├── itr_cw.launch # Launch file to start all components


