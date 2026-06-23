# Vision-guided-industrial-robot-for-pick-and-place-tasks
Visually Guided Robot (VGR) arm simulation for an industrial picking application. Designed for a 2nd-year mechatronics project at ENISo. Uses Webots and YOLO v8 to model a robot arm picking randomly placed 3D parts from a moving conveyor using a multi-sensor system (camera, IR, proximity) and custom control laws.
# 🤖 Vision-Guided Industrial Robot for Pick & Place Tasks

## 🎬 Project Demo

Below is a simulation demo of the vision-guided industrial robot performing object detection and pick-and-place tasks:

![Robot Demo](assets/Vidéo_simulation.gif)

---



---

## 📌 Overview

How does a robot know how to grasp a part that can land at any angle on a conveyor?

This project answers that question by building a full **perception–action pipeline** entirely in simulation. A camera feed from Webots is streamed over a TCP socket to a Python inference server running **YOLOv8 with Oriented Bounding Boxes (OBB)**. The detected orientation angle is normalized and sent back to a C-based Webots controller, which adjusts the gripper and executes the pick.

---

## 🏗️ System Architecture

```
Webots Camera
     │
     │  (TCP Socket — raw image frames)
     ▼
Python Inference Server
     │  YOLOv8 OBB detection
     │  Orientation angle extraction & normalization (0°–180°)
     │
     │  (TCP Socket — angle response)
     ▼
Webots Controller (C)
     │  Gripper alignment
     │  Pick-and-place execution
     ▼
Robotic Arm
```

---

## ✨ Features

- **Orientation-aware detection** — YOLOv8 OBB predicts both the bounding box and the rotation angle of each part, enabling the gripper to align correctly regardless of how the part lands.
- **Real-time client–server integration** — image frames and angle results are exchanged over TCP/IP, decoupling the vision pipeline from the robot controller.
- **Simulation-first approach** — the entire system runs inside Webots, making it safe to iterate without physical hardware.
- **Custom trained model** — a domain-specific OBB dataset was built, augmented with Roboflow, and fine-tuned in Google Colab.

---

## 🧠 Model Training

| Step | Tool |
|---|---|
| Dataset creation & annotation | Manual OBB labeling |
| Dataset management & augmentation | [Roboflow](https://roboflow.com) |
| Model fine-tuning | Google Colab (YOLOv8 OBB) |
| Inference weights | `best.pt` |

The model was trained to detect industrial parts and output an oriented bounding box, from which the rotation angle is extracted, normalized to the **[0°, 180°]** range, and forwarded to the robot controller.

---

## 🛠️ Tech Stack

| Layer | Technology |
|---|---|
| Simulation & robot control | [Webots](https://cyberbotics.com) |
| Robot controller language | C |
| Object detection | YOLOv8 OBB (`ultralytics`) |
| Vision pipeline & inference server | Python |
| Communication | TCP/IP Sockets |
| Dataset preparation | Roboflow |
| Training environment | Google Colab |

---

## 📂 Project Structure

```
├── webots/
│   ├── worlds/          # Webots world files
│   ├── controllers/
│   │   └── arm_controller/  # C controller — TCP client, gripper logic
│   └── protos/          # Robot & conveyor proto files
│
├── vision/
│   ├── server.py        # Python TCP server — receives frames, runs YOLO, returns angle
│   ├── best.pt          # YOLOv8 OBB trained weights
│   └── utils.py         # Angle normalization & preprocessing helpers
│
├── dataset/
│   └── README.md        # Dataset structure & Roboflow export instructions
│
└── README.md
```

> **Note:** Adjust this structure to match your actual layout.

---

## 🚀 Getting Started

### Prerequisites

- [Webots](https://cyberbotics.com/#download) R2023b or later
- Python 3.9+
- A C compiler (GCC or the one bundled with Webots)

### Installation

```bash
# Clone the repository
git clone https://github.com/your-username/your-repo-name.git
cd your-repo-name

# Install Python dependencies
pip install ultralytics opencv-python numpy
```

### Running the System

**1. Start the Python inference server**
```bash
python vision/server.py
```
The server listens on a local TCP port and waits for image frames.

**2. Launch Webots and open the world**

Open `webots/worlds/conveyor_picking.wbt` (or your world filename).

**3. Run the simulation**

Press Play in Webots. The C controller will connect to the server, stream camera frames, receive orientation angles, and drive the robotic arm.

---

## 📊 Results

- Accurate orientation estimation across the full **0°–180°** range
- Stable real-time communication between the Webots controller and the Python server
- Successful pick sequences for parts arriving at arbitrary angles on the conveyor

---

## 🧩 Skills Developed

- Vision-based robotic manipulation with YOLOv8 OBB
- Perception–control integration across heterogeneous languages (C ↔ Python)
- TCP/IP client–server architecture for real-time robotics
- Deep learning model training, augmentation, and deployment
- Robotic simulation with Webots (sensors, actuators, world design)

---


---

## 🙏 Acknowledgements

- [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics) for the OBB detection framework
- [Roboflow](https://roboflow.com) for dataset management and augmentation
- [Cyberbotics Webots](https://cyberbotics.com) for the open-source robotics simulator
- Google Colab for the training environment

