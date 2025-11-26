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

1. The terminal in the top-left shows all the tabs running the commands required for the project. These processes are started using the ./run.sh command.
2. The Gazebo window in the top-right is the simulation environment where the robotic arm is executed and controlled.
3. The window in the bottom-left is part of the hand-gesture module, and it detects and sends the recognized hand gestures.
4. The window on the bottom-right runs RViz with the MoveIt plugin. MoveIt handles the motion planning for the robotic arm, while RViz is used to visualize the robot and the planned trajectories.


## How to Run

1. **Make sure all required packages and libraries are installed** (system, ROS, and Python dependencies).
2. **Run the startup script(Or manually run the code inside that file)**:
   ```bash
   ./run.sh
