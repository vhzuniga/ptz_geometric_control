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


---

## 🚀 Installation & Setup


# 1. Install system dependencies

```bash
sudo apt-get update
```
```bash
sudo apt-get install ros-noetic-joy
```
```bash
sudo apt-get install ros-noetic-serial
```
# 2. Create and initialize your workspace
```bash
mkdir -p ~/ptz_ws/src
```
```bash
cd ~/ptz_ws/src
```
# 3. Clone the repository
```bash
git clone https://github.com/vhzuniga/ptz_geometric_control.git
```
# 4. Build the workspace
```bash
cd ~/ptz_ws
```
```bash
catkin_make
```
# 5. Source the workspace
```bash
echo "source ~/ptz_ws/devel/setup.bash" >> ~/.bashrc
```
```bash
source ~/.bashrc
```
# 6. Set up udev rules 
```bash
roscd pan_tilt_bringup/config
```
```bash
sudo cp ./56-pan-tilt.rules /etc/udev/rules.d/
```
# 7. Launch the visualization and hardware
```bash
roslaunch pan_tilt_bringup panTilt_view.launch
```
---




