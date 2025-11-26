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

## Project Video

https://github.com/user-attachments/assets/ab1a83ec-530b-4827-90ac-fee754aa0577

1. The terminal in the top left contains all the tabs running the commands required to run the entire projects, They are started using the ./run.sh command.
2. The gazebo window on the top right is the simulator where the robotic arm control is executed and controlled.
3. The window on the bottom left is part of th ehand gesture module and it detects and send the hand gesture.
4. The window on the bottom right runs RViz and it is responsible for the motion plannig of the robotic arm.


## How to Run

1. **Make sure all required packages and libraries are installed** (system, ROS, and Python dependencies).
2. **Run the startup script(Or manually run the code inside that file)**:
   ```bash
   ./run.sh
