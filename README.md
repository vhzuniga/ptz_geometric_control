# PTZ Geometric Control

This project implements a **geometry-based control system** for a Pan-Tilt-Zoom (PTZ) camera using ROS Noetic. Instead of relying on feature detection or computer vision models, this system uses **geometric modeling** and **manual reference points** to compute optimal zoom levels and camera corrections with high reproducibility.

---

## 📷 Hardware Used

- **Camera**: Datavideo BC-50 (20x optical zoom)
- **Pan-Tilt Base**: IQR-PTU2 (2 DoF)

---

## 📦 Features

- Manual selection of a target point via RViz.
- Real-time computation of pan/tilt angles based on TF transformations.
- Estimation of object size in pixels from real-world dimensions and distance.
- Calculation of **optimal zoom level** using experimental FOV and scale tables.
- Angular correction based on **pre-calibrated tables** for each zoom level.
- Error reporting (pixel offset and margin analysis).
- Optional visual debug with OpenCV (optional use).

---

## 🧩 System Requirements

- **ROS Noetic** (Ubuntu 20.04)
- Python 3
- A real hardware PTZ system using:
  - Datavideo BC-50 camera
  - IQR-PTU2 pan-tilt base
- Packages:
  - `rospy`, `sensor_msgs`, `geometry_msgs`, `tf2_ros`, `visualization_msgs`, etc.

---

## 🚀 Installation & Setup

```bash
# 1. Create and initialize your workspace
mkdir -p ~/ptz_ws/src
cd ~/ptz_ws/src

# 2. Clone the repository
git clone https://github.com/vhzuniga/ptz_geometric_control.git

# 3. Build the workspace
cd ~/ptz_ws
catkin_make

# 4. Source the workspace
echo "source ~/ptz_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 5. Set up udev rules (for USB connection to PTZ device)
roscd pan_tilt_bringup/config
sudo cp ./56-pan-tilt.rules /etc/udev/rules.d/

# 6. Launch the visualization and hardware
roslaunch pan_tilt_bringup panTilt_view.launch
