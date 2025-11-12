# English | [中文](README_cn.md)

# Deployment of Training Results

### 1. Deployment Environment Configuration

- Install ROS Noetic: We recommend setting up an algorithm development environment based on ROS Noetic on the Ubuntu 20.04 operating system. ROS provides a series of tools and libraries that greatly facilitate the development, testing, and deployment of robot algorithms. These resources offer users a rich and complete algorithm development environment.

  - For ROS Noetic installation, please refer to the documentation: https://wiki.ros.org/noetic/Installation/Ubuntu, and select "ros-noetic-desktop-full" for installation.

  - After installing ROS Noetic, enter the following Shell commands in the Bash terminal to install the libraries required for the development environment:

    ```
    sudo apt-get update
    sudo apt install ros-noetic-urdf \
                     ros-noetic-kdl-parser \
                     ros-noetic-urdf-parser-plugin \
                     ros-noetic-hardware-interface \
                     ros-noetic-controller-manager \
                     ros-noetic-controller-interface \
                     ros-noetic-robot-state-* \
                     ros-noetic-joint-state-* \
                     ros-noetic-controller-manager-msgs \
                     ros-noetic-control-msgs \
                     ros-noetic-ros-control \
                     ros-noetic-gazebo-* \
                     ros-noetic-rqt-gui \
                     ros-noetic-rqt-controller-manager \
                     ros-noetic-plotjuggler* \
                     ros-noetic-joy-teleop ros-noetic-joy \
                     cmake build-essential libpcl-dev libeigen3-dev libopencv-dev libmatio-dev \
                     python3-pip libboost-all-dev libtbb-dev liburdfdom-dev liborocos-kdl-dev -y
    ```

- Install onnxruntime dependencies. Download link: https://github.com/microsoft/onnxruntime/releases/tag/v1.10.0. Please select the appropriate version according to your operating system and platform. For Ubuntu 20.04 x86_64, follow these steps for installation:

  ```
  wget https://github.com/microsoft/onnxruntime/releases/download/v1.10.0/onnxruntime-linux-x64-1.10.0.tgz
  
  tar xvf onnxruntime-linux-x64-1.10.0.tgz
  
  sudo cp -a onnxruntime-linux-x64-1.10.0/include/* /usr/include
  sudo cp -a onnxruntime-linux-x64-1.10.0/lib/* /usr/lib
  ```

### 2. Create a Workspace

 You can create an RL deployment development workspace by following these steps:

- Open a Bash terminal.

- Create a new directory to store the workspace. For example, create a directory named "limx_ws" in the user's home directory:
  ```Bash
  mkdir -p ~/limx_ws
  ```
  
- Download the MuJoCo simulator:
  ```Bash
  cd ~/limx_ws

  # Option 1: HTTPS
  git clone --recurse https://github.com/limxdynamics/humanoid-mujoco-sim.git
  
  # Option 2: SSH
  git clone --recurse git@github.com:limxdynamics/humanoid-mujoco-sim.git
  ```
  
- Download the motion control algorithm:
  ```Bash
  cd ~/limx_ws
  
  # Option 1: HTTPS
  git clone --recurse https://github.com/limxdynamics/humanoid-rl-deploy-ros.git

  # Option 2: SSH
  git clone --recurse git@github.com:limxdynamics/humanoid-rl-deploy-ros.git
  ```
  
- Set the robot model: If it is not set yet, follow these steps:
  - List available robot types with the Shell command  `tree -L 1 -P "humanoid_controllers" humanoid-rl-deploy-ros/robot_controllers/config/humanoid_controllers`:
    
    ```
    cd ~/limx_ws/humanoid-rl-deploy-ros/src
    tree -L 1 -P "humanoid_controllers" humanoid-rl-deploy-ros/robot_controllers/config/humanoid_controllers
    humanoid-rl-deploy-ros/robot_controllers/config/humanoid_controllers
    ├── HU_D03_03
    └── HU_D04_01
    ```
    
  - Taking `HU_D04_01` (replace with your actual robot type) as an example, set the robot model type:
    
    ```
    echo 'export ROBOT_TYPE=HU_D04_01' >> ~/.bashrc && source ~/.bashrc
    ```

### 3. Simulation Debugging

- Run the MuJoCo simulator (Python 3.8 or higher is recommended):

  - Open a Bash terminal.

  - Install the motion control development library:
    - For Linux x86_64 environment:
    
      ```bash
      cd ~/limx_ws
      pip install humanoid-mujoco-sim/limxsdk-lowlevel/python3/amd64/limxsdk-*-py3-none-any.whl
      ```
    
    - For Linux aarch64 environment:
    
      ```bash
      cd ~/limx_ws
      pip install humanoid-mujoco-sim/limxsdksdk-lowlevel/python3/aarch64/limxsdk-*-py3-none-any.whl
      ```
    
  - Run the MuJoCo simulator:
    
    ```bash
    cd ~/limx_ws
    python humanoid-mujoco-sim/simulator.py
    ```

- Run the algorithm:

  - Open a Bash terminal.

  - Navigate to your workspace and complete the compilation:
    
    ```bash
    # If you have Conda installed, temporarily disable the Conda environment
    # because Conda may interfere with ROS environment settings
    conda deactivate
    
    # Set up the ROS compilation environment
    source /opt/ros/noetic/setup.bash
    
    # Compile the algorithm code
    cd ~/limx_ws/humanoid-rl-deploy-ros
    catkin_make install
    ```
    
  - Run the algorithm:
    
    ```bash
    # If you have Conda installed, temporarily disable the Conda environment
    # because Conda may interfere with ROS environment settings
    conda deactivate
    
    # Set up the ROS compilation environment
    source /opt/ros/noetic/setup.bash
    
    # Run the algorithm
    cd ~/limx_ws/humanoid-rl-deploy-ros
    source install/setup.bash
    roslaunch robot_hw humanoid_hw_sim.launch
    ```
    
    ![](doc/simulator.gif)

- Virtual Joystick: You can use a virtual joystick to operate the robot during simulation. Here are the specific steps to use the virtual joystick:

  - Open a Bash terminal.

  - Run the virtual joystick:

    ```
    ~/limx_ws/humanoid-mujoco-sim/robot-joystick/robot-joystick
    ```
    
    ![](doc/robot-joystick.png)


  - At this point, you can use the virtual joystick to control the robot.
  
    | **Button** | **Mode**         | **Description**                                                    |
    | -------- | ---------------- | ----------------------------------------------------------- |
    | L1+Y     | Switch to Stand Mode  | If the robot cannot stand, click "Reset" in the MuJoCo interface to reset it. |
    | L1+B     | Switch to Greeting Mode |                                                             |

### 4. Real Robot Debugging

- Set your computer's IP: Ensure your computer is connected to the robot via the external network port. Set your computer's IP address to `10.192.1.200` and verify connectivity using the Shell command `ping 10.192.1.2`. Configure your development computer's IP as shown in the following figure:

  ![img](doc/ip.png)

- Navigate to your workspace, find the `humanoid_hw.launch` startup file, and modify the robot's IP address to 10.192.1.2 as shown in the following figure:

  ![img](doc/humanoid_hw.png)

- Compile after making the modifications:

    ```bash
    # If you have Conda installed, temporarily disable the Conda environment
    # because Conda may interfere with ROS environment settings
    conda deactivate
    
    # Set up the ROS compilation environment
    source /opt/ros/noetic/setup.bash
    
    # Compile the algorithm code
    cd ~/limx_ws/humanoid-rl-deploy-ros
    catkin_make install
    ```

- Robot preparation:

    - Hang the robot using the hooks on its left and right shoulders.
    - After pressing the power button to turn it on, press the `right joystick` button on the remote control to start the robot's motors.
    - Press the remote control buttons `R1 + DOWN` to switch to developer mode. In this mode, users can develop their own motion control algorithms. (This mode setting will remain effective after the next startup. To exit developer mode, press `R1 + LEFT`.)

- Deploy and run on the real robot. In the Bash terminal, simply use the following Shell command to start the control algorithm:

  ```bash
  # If you have Conda installed, temporarily disable the Conda environment
  # because Conda may interfere with ROS environment settings
  conda deactivate
  
  # Set up the ROS compilation environment
  source /opt/ros/noetic/setup.bash
  
  # Run the algorithm
  cd ~/limx_ws/humanoid-rl-deploy-ros
  source install/setup.bash
  roslaunch robot_hw humanoid_hw.launch
  ```

- At this point, you can press the remote control buttons `L1 + Y` to make the robot enter the standing mode.

- Press `L1 + B` on the remote control to make the robot greet.
