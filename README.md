# Project README

## Prerequisites

### System Requirements
- **Ubuntu 18.04**
- **ROS Noetic** (full installation), including:
  - Gazebo  
  - RViz

### Hand Gesture Module Dependencies  
> It is recommended to create a **new virtual environment** before installing these packages.

Install the following Python packages:

- `mediapipe` **0.8.1 or later**
- `opencv-python` **3.4.2 or later**
- `tensorflow` **2.3.0 or later**
- `tf-nightly` **2.5.0.dev or later**
- `scikit-learn` **0.23.2 or later**
- `matplotlib` **3.3.2 or later**

---

## How to Run

1. **Make sure all required packages and libraries are installed** (system, ROS, and Python dependencies).
2. **Run the startup script(Or manually run the code inside that file)**:
   ```bash
   ./run.sh
