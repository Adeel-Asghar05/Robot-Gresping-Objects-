  Robotic Manipulation with PyBullet

    This project simulates a robotic arm with a gripper in PyBullet. The robot uses a camera to detect objects, estimate grasp positions, and pick-and-place         them onto a target location.
    The simulation integrates inverse kinematics, keyboard teleoperation, and object grasping strategies.

🚀 Features

    Loads a custom robot (final.urdf) and workspace (waste.urdf).

    waste.ipynb as table where objects are placed and trash bins besides of robot base 

    Uses camera simulation (fixed_camera.py, robot_view.py) for RGB-D sensing.

    Converts depth maps to 3D point clouds for object pose estimation.

    Supports grasp pose generation (object_pose.py).

    Keyboard control for manual teleoperation of joints.

    Automatic pick-and-place routine triggered by pressing P.

    Gripper control for opening and closing.

📦 Requirements

    Install dependencies:

    pip install pybullet numpy


Ensure the following project files exist in the repository:

    final.urdf → Robot model

    waste.urdf → Table/workspace model

    obj4.urdf → Object models (placed on table)

    fixed_camera.py → Camera sensor utility

    robot_view.py → Camera controller

    object_pose.py → Functions for object pose + grasp detection

▶️ Usage

    Run the simulation in notebook:

    python Robot.ipynb

    Where robot.ipynb contains the provided code.

🎮 Controls

    Key	Action
    ⬆️ / ⬇️	Move joint 1
    ⬅️ / ➡️	Move joint 0
    1 / 2	Move joint 2
    3 / 4	Move joint 3
    5 / 6	Move joint 4
    Q / A	Gripper joint (open/close)
    R / F	Secondary gripper joint
    P	Trigger pick-and-place routine
⚙️ Workflow

    Camera Update:
    Captures RGB-D frames every 5 seconds.

    Object Detection:

    Converts depth → point cloud.

    Segments objects in view.

    Estimates grasp poses (object_pose_and_grasp).

    Pick-and-Place:

    Robot moves to grasp position.

    Closes gripper → picks object.

    Moves to predefined placement location.

    Opens gripper → releases object.

    Returns to rest pose.
  
📂 Project Structure

    ├── main.py                # Simulation entry script
    ├── final.urdf-------> meshes path 3D/meshes/meshname.stl   # Robot model
    ├── waste.urdf             # Table/workspace
    ├── obj4.urdf              # Sample object
    ├── fixed_camera.py        # Camera utilities
    ├── robot_view.py          # Camera control
    ├── object_pose.py         # Object detection + grasping
    └── README.md              # Project documentation
