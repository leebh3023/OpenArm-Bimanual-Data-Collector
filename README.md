# OpenArm Bimanual Data Collector

## 🏫 Institution Information
- **KONKUK Univ. RVLAB**
- **Author**: Lee Byeong-hyeon
- **Email**: [leebh3023@gmail.com](mailto:leebh3023@gmail.com)

## 📝 Project Overview
This project is a dedicated GUI system for real-time control and training data collection of the OpenArm Bimanual robot.
- **Real-time Monitoring**: Supports dual-arm joint state monitoring and multi-camera streaming.
- **Data Logging**: Saves data in **HDF5 (.h5)** format, optimized for training VLA models (ACT, Diffusion Policy, etc.), integrating joint states and image data.
- **Stable Control**: Independent, high-performance control loop based on CAN communication.

## 🚀 How to Run

### 1. Prerequisites
Using a virtual environment is strongly recommended to prevent polluting the local system environment.

### 2. Running the GUI (Automated)
Execute the provided `start_gui.sh` script to automatically handle virtual environment creation, dependency installation, and program launch.
```bash
cd openarm_gui
./start_gui.sh
```

### 3. Detailed Guide
For more detailed technical specifications and usage, please refer to [doc/Walkthrough.md](openarm_gui/doc/Walkthrough.md).
