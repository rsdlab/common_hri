# Common HRI Components (common_hri)

**English** | [日本語](README.ja.md)

## Overview

- A repository for managing HRI (Human-Robot Interaction) components based on RoIS 2.0.
- This repository provides HRI functions shared across robots, while robot-specific operations are separated into execution nodes deployed on each robot.
- `common_hri_ros1` supports ROS 1, and `common_hri_ros2` supports ROS 2.
- Used in conjunction with the [RT System Integration Framework (RTSI-FW)](https://github.com/rsdlab/RTSI_FW).

## Development Environment

- Programming language: Python
- ROS: ROS 1 Noetic / ROS 2 Humble
- Supported OS: Ubuntu 20.04 / 22.04

## Repository Structure

```text
common_hri/
├── README.md
├── README.ja.md
├── common_hri_ros1/
│   ├── config/
│   ├── include/
│   ├── launch/
│   ├── rtc/
│   ├── script/
│   ├── yaml/
│   ├── CMakeLists.txt
│   ├── hri.xml
│   ├── mgr.py
│   ├── package.xml
│   ├── robot.yaml
│   └── setting.yaml
└── common_hri_ros2/
```

| Directory / File | Description |
| --- | --- |
| `common_hri_ros1/` | Common HRI package for ROS 1 |
| `common_hri_ros2/` | Common HRI package for ROS 2 |
| `config/` | Robot-specific configuration files |
| `script/` | Implementations of HRI components and MainEngine |
| `yaml/` | Configuration files used to collect HRI components |
| `launch/` | Files for launching nodes |
| `robot.yaml` | Configuration for the selected robot; copy the corresponding file from `config/` for use |
| `hri.xml` | HRI package definition; copy the corresponding file from `config/` for use |

## System Architecture

RTSI-FW system architecture using this package:

```text
RTSI-FW / Service Application
             ↓
         MainEngine
             ↓
     Common HRI Components
   (common_hri_ros1 / ros2)
             ↓
  Robot-Specific Execution Nodes
    (e.g., airec_hri_action)
             ↓
       Robot / Simulator
```

Common HRI components receive requests from upper layers and execute robot operations through robot-specific Action nodes. This separates differences in robot-specific implementations into the Action layer.

## Robot Configuration

Robot-specific configuration files are managed in the `config/` directory of each package.
The `robot.yaml` file specifies the HRI components to use and robot-dependent service names, among other settings.
This package includes example configuration files intended for AIREC, HSR, and SEED-Noid.

```text
config/
├── airec/
│   ├── robot.yaml
│   └── hri.xml
├── hsr/
│   ├── robot.yaml
│   └── hri.xml
└── seed/
    ├── robot.yaml
    └── hri.xml
```

## Notes

- Use the ROS 1 and ROS 2 packages in their respective ROS workspaces.
- Person-related components are referenced in some configuration files but are not included in the current public release.
- Some components contain functions whose implementation or operation has not yet been fully verified.

## Related Repositories

- [RTSI-FW](https://github.com/rsdlab/RTSI_FW)
- [rois_env](https://github.com/rsdlab/rois_env/tree/ros1-ros2)
- [Wasanbon](https://github.com/rsdlab/wasanbon/tree/ros1-ros2)
- [Wasanbon Web Framework](https://github.com/rsdlab/wasanbon_webframework)
