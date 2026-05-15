# bxi_revo2_example

Revo2 灵巧手BXI can接口 ROS 2 示例程序，基于 bc-stark-sdk 实现手指位置控制、速度控制、力反馈等演示功能。

**更多请关注官方例程** - https://github.com/BrainCoTech/brainco-hand-sdk.git

我们仅移植底层通信接口去调用官方的sdk

所有功能都包含在官方sdk中dist/include/stark-sdk.h

## 依赖

- ROS 2 (测试环境: Humble)
- CMake >= 3.8
- C++17
- wget, unzip

### ROS 2 软件包依赖

- `rclcpp`
- `std_msgs`
- `communication` bxi_ros2_pkg

## ⚠️ 编译前必须先下载 SDK

**在编译之前，务必先运行 `download-lib.sh` 下载 bc-stark-sdk 库文件。** 该脚本会从远程下载编译所需的 `.so` 动态库和头文件到 `dist/` 目录，CMakeLists.txt 会链接这些文件。如果不执行此步骤，编译将失败。

```bash
# 下载 bc-stark-sdk (必须在编译前执行)
./download-lib.sh
```

`download-lib.sh` 会根据当前操作系统和架构自动选择对应的库文件：
- **Linux x86_64**: `linux.zip`
- **Linux aarch64**: `linux-arm64.zip`
- **macOS**: `mac.zip`
- **Windows**: `win.zip`

下载完成后会在项目根目录生成 `VERSION` 文件记录 SDK 版本，`dist/` 目录会包含所需的头文件和库文件（这些目录均已被 `.gitignore` 忽略，不会提交到仓库）。

## 编译

```bash
# 1. 下载 SDK（必须！）
./download-lib.sh

# 2. 确认 ROS 2 环境已加载
source /opt/ros/humble/setup.bash

# 3. 导入 communication 等自定义消息包（如有单独编译的自定义包，需先 source 其 install 目录）
# source /path/to/your/workspace/install/setup.bash

# 4. 使用 colcon 编译
colcon build --packages-select bxi_revo2_example
```

## 运行

```bash
# 加载工作空间环境
source install/setup.bash

# 运行示例节点
ros2 run bxi_revo2_example bxi_revo2_example
```

## 演示内容

示例程序包含以下功能演示：

1. **基本位置控制** — 握拳、张开、单指移动
2. **速度控制** — 手指速度模式控制
3. **力反馈控制** — 带力阈值的抓取控制
4. **设备信息查询** — 读取硬件类型、序列号、固件版本

## 项目结构

```
bxi_revo2_example/
├── CMakeLists.txt          # CMake 构建配置
├── download-lib.sh          # SDK 下载脚本（编译前必须执行）
├── package.xml              # ROS 2 包描述
├── VERSION                  # SDK 版本记录 (由 download-lib.sh 生成)
├── src/
│   └── bxi_revo2_example.cpp  # 主程序
├── include/
│   └── bxi_can_node.hpp       # CAN 节点头文件
└── dist/                    # SDK 下载目录 (由 download-lib.sh 生成，被 .gitignore 忽略)
    ├── include/
    │   ├── stark-sdk.h
    │   └── zlgcan/
    └── shared/linux/
        └── libbc_stark_sdk.so
```
