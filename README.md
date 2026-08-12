# ⚙️ Metal Mavericks

## WRO Future Engineers 2026

Welcome to the engineering repository of **Metal Mavericks**, a student robotics team from India participating in **WRO Future Engineers 2026**.

Our objective is to develop an autonomous vehicle capable of sensing its surroundings, understanding the track, making real-time decisions, and controlling its movement without human intervention.

This repository documents the complete development of our robot — from mechanical construction and electronics to autonomous software, testing, CAD models, photographs, schematics, and competition performance.

---

## 🏴 Team

**Team Name:** Metal Mavericks
**Country:** India
**School / Organization:** ROBOFUN LAB

### Team Members

* **Arhaan Sharma**
* **Saranya Singhal**
* **Myraah M Sadani**
  
### Coach

**Vishal Kanjariya**

---

# 🤖 The Robot

Metal Mavericks is an autonomous self-driving robot developed specifically for the WRO Future Engineers challenge.

The robot combines:

* Raspberry Pi based computing
* Camera-based perception
* Computer vision
* Motor control
* Servo steering
* Track interpretation
* Obstacle recognition
* Autonomous navigation
* Real-time decision making

Rather than treating the robot as a collection of separate components, our development approach focuses on making the mechanical, electrical and software systems work together as one vehicle.

---

# 🧠 Autonomous System

The robot continuously follows this basic decision cycle:

```text
        CAMERA
           ↓
     IMAGE CAPTURE
           ↓
    VISUAL ANALYSIS
           ↓
   TRACK / OBJECT DATA
           ↓
    DECISION ENGINE
           ↓
    STEERING + DRIVE
           ↓
      ROBOT MOTION
           ↓
       NEXT FRAME
```

The system repeats this process while the robot is moving.

This allows the robot to react to changes in the environment instead of depending entirely on a pre-programmed movement sequence.

---

# 👁️ Perception

The camera acts as the primary source of environmental information.

The vision system processes the camera image to determine useful information such as:

* Track position
* Track direction
* Colored markers
* Obstacles
* Relative position of detected objects
* Safe driving area

The image-processing pipeline is designed to reduce unnecessary information and concentrate on the part of the image that is useful for driving.

Typical processing stages include:

```text
Camera Frame
     ↓
Image Preparation
     ↓
Region Selection
     ↓
Color / Feature Detection
     ↓
Object Filtering
     ↓
Position Estimation
     ↓
Driving Decision
```

---

# 🛞 Motion Control

The robot uses a dedicated drive system together with servo-based steering.

The software does not simply command the steering to a fixed angle.

Instead, the detected position of the track or target is converted into a steering correction.

Conceptually:

```text
Target Position
      ↓
Robot Position
      ↓
Position Error
      ↓
Steering Correction
      ↓
Servo Command
```

Small errors produce small corrections while larger errors require stronger steering.

This provides smoother movement and reduces unnecessary steering oscillations.

---

# 🔄 Driving Strategy

Metal Mavericks is designed around **continuous correction**.

The robot repeatedly asks:

1. Where am I?
2. Where should I be?
3. How far am I from the desired path?
4. Which direction should I steer?
5. How quickly should I move?

The answer to these questions changes as the robot moves around the track.

This makes the driving system adaptive rather than dependent on a fixed sequence of turns.

---

# 🚧 Obstacle Handling

Obstacle handling is treated as a separate decision layer within the autonomous system.

When an obstacle is detected, the robot evaluates its position relative to the vehicle and determines an appropriate path around it.

The general decision structure is:

```text
Obstacle Detected
       ↓
Determine Position
       ↓
Estimate Safe Side
       ↓
Modify Steering
       ↓
Pass Obstacle
       ↓
Return to Normal Driving
```

The robot then transitions back into normal autonomous driving once the obstacle is no longer influencing the vehicle.

---

# 🏁 Open Round

During the Open Round, the robot needs to understand the track and its directional indicators.

Metal Mavericks uses visual information from the camera to determine the required driving behavior.

The software separates:

* Track interpretation
* Direction recognition
* Steering
* Drive control
* Lap management

This separation allows individual parts of the system to be tested independently.

The Open Round software is maintained inside:

```text
src/
```

---

# 🚦 Obstacle Round

The Obstacle Round adds another layer of autonomous decision making.

The robot must combine normal track following with obstacle awareness.

Instead of completely replacing the steering system when an obstacle appears, the obstacle system modifies the normal driving decision.

The architecture can be represented as:

```text
                 CAMERA
                    ↓
          ┌─────────┴─────────┐
          ↓                   ↓
    TRACK ANALYSIS       OBSTACLE ANALYSIS
          ↓                   ↓
          └─────────┬─────────┘
                    ↓
              DECISION LOGIC
                    ↓
             STEERING OUTPUT
                    ↓
                ROBOT
```

This structure makes it possible to tune track following and obstacle behavior independently.

---

# ⚡ Electronics

The electronics system provides power and control to the robot's computing, drive and steering components.

The main controller is a:

**Raspberry Pi 5**

The electronics architecture includes the required motor-control, steering, power-regulation and sensing hardware.

Electrical documentation and schematics are maintained inside:

```text
schemes/
```

The objective is to keep the electrical system:

* Compact
* Reliable
* Easy to troubleshoot
* Securely connected
* Suitable for repeated competition testing

---

# 🔩 Mechanical Design

The robot's mechanical structure is designed around stability, predictable steering and repeatable movement.

Important design considerations include:

* Wheel placement
* Steering geometry
* Camera position
* Component mounting
* Centre of mass
* Ground clearance
* Motor accessibility
* Wiring accessibility

CAD and robot models are stored inside:

```text
models/
```

The mechanical design is continuously evaluated during physical testing.

---

# 💻 Software

The autonomous software is developed in Python.

Major software responsibilities include:

```text
Camera
  ↓
Vision
  ↓
Detection
  ↓
Navigation
  ↓
Steering
  ↓
Motor Control
```

The software is divided into functional components wherever possible so that individual systems can be tested without changing the entire program.

The competition source code is maintained inside:

```text
src/
```

---

# 🧪 Testing

Testing is an important part of the Metal Mavericks development process.

We do not consider the robot complete simply because the program runs.

Each major change is tested on the physical robot.

Testing focuses on:

* Steering response
* Camera positioning
* Vision reliability
* Detection accuracy
* Driving stability
* Obstacle behavior
* Motor response
* Battery performance
* Track consistency
* Recovery behavior

A simplified development cycle is:

```text
IDEA
 ↓
DESIGN
 ↓
BUILD
 ↓
PROGRAM
 ↓
TEST
 ↓
OBSERVE
 ↓
CHANGE
 ↓
TEST AGAIN
```

---

# 📸 Team & Robot Media

Team photographs are stored in:

```text
t-photos/
```

Robot photographs are stored in:

```text
v-photos/
```

Performance and demonstration videos are stored in:

```text
video/
```

These folders provide visual evidence of the team's development and competition preparation.

---

# 📁 Repository Structure

```text
METAL_MAVERICKS/
│
├── models/
│   └── Robot CAD files
│
├── schemes/
│   └── Electrical and system schematics
│
├── src/
│   └── Autonomous robot software
│
├── t-photos/
│   └── Team photographs
│
├── v-photos/
│   └── Robot photographs
│
├── video/
│   └── Robot performance videos
│
└── README.md
```

The folder structure follows the required project documentation format while the engineering content inside each section represents the independent development of Metal Mavericks.

---

# 🧩 Engineering Principles

Metal Mavericks follows several principles during development.

### 1. Measure Before Changing

When the robot behaves incorrectly, we first identify the actual cause instead of changing parameters randomly.

### 2. Keep Systems Independent

Vision, steering, drive control and obstacle handling should be testable individually.

### 3. Test on the Real Robot

A system that works on a computer is not automatically reliable on a moving robot.

### 4. Prefer Repeatable Behavior

Competition performance requires predictable behavior rather than occasional successful runs.

### 5. Improve Through Testing

Every failed test provides information that can be used to improve the next version.

---

# 🏆 WRO Future Engineers 2026

Metal Mavericks was developed as a complete autonomous robotics project for the **WRO Future Engineers 2026** challenge.

This repository contains the technical material behind the robot, including:

* Robot software
* CAD models
* Electrical schematics
* Team documentation
* Robot photographs
* Performance videos
* Engineering development material

The purpose of this repository is not only to show the finished robot, but also to document how the robot was designed, programmed, tested and improved.

---

# ⚙️ METAL MAVERICKS

**Sense → Decide → Move → Adapt**

Built for autonomous robotics.
Built through testing.
Built to compete.
