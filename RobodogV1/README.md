# Robodog URDF → MJCF Conversion Pipeline

This repository provides a **Xacro-based pipeline** to transform a **raw SolidWorks-generated URDF** into a clean, Mujoco-compatible **MJCF** file. The process adds **controllers, sensors, transmissions**, and other simulation-ready elements.

---

## 📌 Features

* ✅ **Automated URDF enhancement** via Xacro:

  * Adds ROS 2 controllers
  * Adds joint transmissions
  * Adds IMU and other sensor plugins
  * Appends a configurable ground plane
* ✅ **Ready for Mujoco** — MJCF output loads after minimal manual edits
* ✅ **Maintains ROS 2 compatibility** for Gazebo simulation
* ✅ Modular Xacro design — easy to extend with new hardware and plugins

---

## 📂 Repository Structure

```
urdf/
├── ROBODOG URDF.urdf         # Raw URDF from SolidWorks
├── robodog.urdf.xacro        # Main Xacro entry point
├── robodog_transmission.xacro
├── robodog_ros2control.xacro
├── imu.urdf.xacro
├── lidar.urdf.xacro
├── ground.urdf.xacro         # Ground plane definition
├── meshes/                   # STL mesh files
└── *.mjcf / *.urdf           # Generated files
```

---

## ⚙️ Installation

```bash
sudo apt install ros-${ROS_DISTRO}-xacro
pip install urdf2mjcf
```

---

## 🚀 Usage

### 1️⃣ Configure Xacro Before Conversion

Edit `robodog.urdf.xacro` to:

1. **Verify mesh paths** — ensure they are correct for your system 
2. **Match joint names** with those in the SolidWorks URDF:

```xml
<xacro:property name="legs"     value="${['front_right','front_left','back_right','back_left']}"/>
<xacro:property name="segments" value="${['_1','_2','_3']}"/>
```

3. **Add extra sensors if needed**:

```xml
<xacro:include filename="$(find robodog)/urdf/imu.urdf.xacro"/>
<xacro:imu_sensor xyz="0 0 0" rpy="0 0 0" parent="base_link"/>
```

4. **Adjust ground plane** settings as required:

```xml
<xacro:ground_plane size_x="100" size_y="100" size_z="0.2" z_offset="-0.1"/>
```
5. **Note :** 
You have to comment the ground if you are going to spawn the robot in Gazebo 
---

### 2️⃣ Generate an Enhanced URDF

```bash
cd urdf
ros2 run xacro xacro robodog.urdf.xacro -o robodog_enhanced.urdf
```

---

### 3️⃣ Convert URDF → MJCF

```bash
urdf2mjcf robodog_enhanced.urdf
```

---

### 4️⃣ Minimal Manual MJCF Edits

In `robodog.mjcf`:

1. **Ensure mesh paths** are correct.(remove any `package://` prefixes if necessary).
2. **Add a material name if missing**:

```xml
<material name="mat01" rgba="0.89804 0.91765 0.92941 1" />
```

3. **Assign materials to geoms** where missing:

```xml
<geom name="base_link_visual" material="mat01" ... />
```

4. **Add a ground plane** before `</worldbody>`:

```xml
<geom name="floor" type="plane" pos="0 0 0" size="50 50 1"
      material="mat01" rgba="0.8 0.8 0.8 1"
      contype="1" conaffinity="1" friction="1 0.005 0.0001" />
```

---

### 5️⃣ Load into Mujoco

Open Mujoco and load `robodog.mjcf` — your robot should appear with correct materials and a ground plane.

---

## 🔄 Workflow Overview

```
[ SolidWorks URDF ]
        ↓
[ Edit Xacro (joints, sensors, paths) ]
        ↓
[ Enhanced URDF ]
        ↓
[ urdf2mjcf Conversion ]
        ↓
[ Minimal MJCF Fixes ]
        ↓
[ Mujoco Simulation ]
```

