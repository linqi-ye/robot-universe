# 天工通用人形机器人 URDF & CAD 文件

本仓库提供了 **天工通用人形机器人** 的 **URDF 机械描述文件**、**STL 网格文件** 和 **STEP 结构图纸**，包括 **Lite 版本** 和 **Pro 版本**，适用于 **ROS** 及 **Gazebo** 仿真环境，供研究、开发和仿真使用。

TienKung_URDF （项目根目录）
```
  ├── 📜 README.md        # 项目总览
  ├── 📂 lite/                     # Lite 版本
  │   ├── 📂 urdf/                 # URDF 文件
  │   │   ├── humanoid_publish.urdf
  │   │   ├── 00-leg-assy-urdf-0402.csv
  │   │   ├── Humanoid.proto
  │   ├── 📂 meshes/               # STL 网格文件
  │   │   ├── shoulder_pitch_r_link.STL
  │   │   ├── ...
  │   ├── TG10-00 机器人总装体.STEP                # 结构图纸（STEP 格式）
  │   ├── README.md                # Lite 版本说明
```

```
 │
 ├── 📂 pro/                      # Pro 版本
 │   ├── 📂 urdf/                 # URDF 文件
 │   │   ├── humanoid_publish.urdf
 │   ├── 📂 meshes/               # STL 网格文件
 │   │   ├── 📂 机身Urdf
 │   │   ├── shoulder_pitch_r_link.STL
 │   │   ├── ...
 │   ├── TG11-00 机器人总装体.STEP                 # 结构图纸（STEP 格式）
 │   ├── README.md                # Pro 版本说明
```

---

## 🔹 **URDF 说明**
**URDF（Unified Robot Description Format）** 是 ROS 机器人描述文件，本仓库中的 URDF 文件定义了 **天工 Lite 机器人的**：
- **机械结构**
- **关节限位**
- **质量分布**
- **惯性参数**

该文件适用于 **ROS** 及 **Gazebo 仿真**，可用于运动规划与控制算法研究。

---

## 🔹 **STEP 结构图纸**
本仓库包含 Lite 版机器人的 **lite.STEP** 文件，提供完整的 **三维 CAD 设计**，适用于开发、调试、二次开发和硬件维护。包括：
- **机器人主要部件结构**
- **关节和连杆连接方式**
- **详细的装配信息**

适用于开发、调试、二次开发和硬件维护。

天工Lite结构图纸下载： [Lite结构图纸](https://x-humanoid.com/download/TG10-00%20%E6%9C%BA%E5%99%A8%E4%BA%BA%E6%80%BB%E8%A3%85%E4%BD%93.STEP)

天工Pro结构图纸下载：[Pro结构图纸](https://x-humanoid.com/download/TG11-00%20%E6%9C%BA%E5%99%A8%E4%BA%BA%E6%80%BB%E8%A3%85%E4%BD%93.STEP)

---
