# Installation Guide för ROS2 API

## ROS2 Installation

ROS2 Python-paket (`rclpy`, `std_msgs`) installeras vanligtvis som en del av ROS2-systemet, inte via pip.

### Alternativ 1: ROS2 System Installation (Rekommenderas)

1. **Installera ROS2** (Humble eller Jazzy):
   ```bash
   # För Ubuntu 22.04 (Humble)
   sudo apt update
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   sudo add-apt-repository "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

2. **Aktivera ROS2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. **Installera Python dependencies**:
   ```bash
   cd Ros2
   pip install -r requirements.txt
   ```

### Alternativ 2: Python Virtual Environment med ROS2

Om du använder ett virtual environment:

1. **Aktivera ROS2** (måste göras innan du aktiverar venv):
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Aktivera ditt virtual environment**:
   ```bash
   source /path/to/venv/bin/activate
   ```

3. **Installera dependencies**:
   ```bash
   cd Ros2
   pip install -r requirements.txt
   ```

### Alternativ 3: Conda Environment (Om du använder Conda)

```bash
conda create -n ros2_env python=3.10
conda activate ros2_env
conda install -c conda-forge rclpy std-msgs
pip install numpy
```

## Verifiera Installation

Testa att ROS2 fungerar:

```bash
python3 -c "import rclpy; from std_msgs.msg import String; print('ROS2 OK!')"
```

Om detta fungerar är ROS2 korrekt installerat.

## Felsökning

### Problem: `ModuleNotFoundError: No module named 'rclpy'`

**Lösning**: Du måste aktivera ROS2 environment först:
```bash
source /opt/ros/humble/setup.bash  # eller din ROS2-distribution
```

### Problem: ROS2 fungerar inte i virtual environment

**Lösning**: Aktivera ROS2 FÖRE du aktiverar venv:
```bash
source /opt/ros/humble/setup.bash
source /path/to/venv/bin/activate
```

### Problem: Kan inte hitta `ros2` command

**Lösning**: Installera ROS2 desktop package:
```bash
sudo apt install ros-humble-desktop
```

## Kör Testprogram

Efter installation:

```bash
cd Ros2/api
python3 test_ros2_api.py
```
