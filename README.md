# bxi_revo2_example

Revo2 灵巧手BXI can接口 ROS 2 示例程序，基于 bc-stark-sdk 实现手指位置控制、速度控制、力反馈等演示功能。

**更多请关注官方例程** - https://github.com/BrainCoTech/brainco-hand-sdk.git

我们仅移植底层通信接口去调用官方的sdk

所有功能都包含在官方sdk中dist/include/stark-sdk.h

## 依赖

- ROS 2 (测试环境: Humble)
- CMake >= 3.8
- C++17
- Python 3
- wget, unzip

### ROS 2 软件包依赖

- `rclcpp`
- `rclpy`
- `std_msgs`
- `communication` bxi_ros2_pkg

### Python SDK 依赖

运行 Python 示例前必须安装指定版本的 Python SDK：

```bash
pip install bc-stark-sdk==1.5.1 --index-url https://pypi.org/simple/
```

Python 示例通过 `bc_stark_sdk` 调用官方 SDK，只移植最底层 BxiPci CAN/CANFD 通信桥接逻辑。

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

### C++ 示例

```bash
# 加载工作空间环境
source install/setup.bash

# 运行 C++ 示例节点
ros2 run bxi_revo2_example bxi_revo2_example
```

### Python 示例

```bash
# 加载工作空间环境
source install/setup.bash

# 安装 Python SDK（运行 Python 示例前必须执行）
pip install bc-stark-sdk==1.5.1 --index-url https://pypi.org/simple/

# 运行 Python 示例：master=1，左手 bus=5/slave=0x7e，右手 bus=6/slave=0x7f
ros2 run bxi_revo2_example bxi_revo2_example.py
```

## 在自己的 ROS 2 Node class 中使用

`bxi_can_node.hpp` 和 `bxi_can_node.py` 会在 `init_bxipci_device()` 内部启动 BxiPci ROS bridge。用户自己的节点只需要正常加入自己的 executor，不需要手动把 bridge 节点加入 executor。

### C++ Node class 示例

C++ SDK 接口是同步调用，适合直接放在 timer、service 或 action 的回调里。因为底层 bridge 需要同时收发 ROS 消息，应用层也建议使用 `MultiThreadedExecutor`。

```cpp
#include "bxi_can_node.hpp"
#include "stark-sdk.h"

#include <chrono>
#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

using namespace std::chrono_literals;

class HandNode : public rclcpp::Node {
public:
    HandNode() : rclcpp::Node("hand_node")
    {
        ctx_.master_id = 1;
        ctx_.hw_type_override = STARK_HARDWARE_TYPE_REVO2_BASIC;

        const uint32_t bus = 5;
        const uint8_t slave_id = 126;
        const bool is_canfd = true;
        if (!init_bxipci_device(&ctx_, bus, slave_id, is_canfd)) {
            throw std::runtime_error("init_bxipci_device failed");
        }

        timer_ = create_wall_timer(1s, [this]() { read_status(); });
    }

private:
    void read_status()
    {
        CMotorStatusData *status =
            stark_get_motor_status(ctx_.handle, ctx_.slave_id);
        if (!status) {
            RCLCPP_WARN(get_logger(), "get_motor_status failed");
            return;
        }

        RCLCPP_INFO(get_logger(), "thumb position: %u", status->positions[0]);
        free_motor_status_data(status);
    }

    BxiDeviceContext ctx_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<HandNode>();
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
```

### Python Node class 示例

Python SDK 接口需要 `await`。在 ROS callback 里不要直接 `asyncio.run()`；推荐给 SDK 单独开一个 asyncio loop，ROS timer/subscription 回调里用 `asyncio.run_coroutine_threadsafe()` 投递任务。

```python
#!/usr/bin/env python3

import asyncio
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from bxi_can_node import (
    BxiDeviceContext,
    cleanup_bxipci_device,
    init_bxipci_device,
    libstark,
    stop_bxipci_runtime,
)


class HandNode(Node):
    def __init__(self, ctx: BxiDeviceContext, loop: asyncio.AbstractEventLoop):
        super().__init__("hand_node_py")
        self.ctx = ctx
        self.loop = loop
        self.pending_status = None
        self.callback_group = ReentrantCallbackGroup()
        self.create_timer(1.0, self.on_timer, callback_group=self.callback_group)

    def on_timer(self):
        if self.pending_status is not None and not self.pending_status.done():
            return

        self.pending_status = asyncio.run_coroutine_threadsafe(
            self.read_status(), self.loop
        )

    async def read_status(self):
        try:
            status = await self.ctx.handle.get_motor_status(self.ctx.slave_id)
        except Exception as exc:
            self.get_logger().warn(f"get_motor_status failed: {exc}")
            return

        self.get_logger().info(f"thumb position: {status.positions[0]}")


def main():
    rclpy.init(args=None)

    loop = asyncio.new_event_loop()
    loop_thread = threading.Thread(target=loop.run_forever, daemon=True)
    loop_thread.start()

    ctx = asyncio.run_coroutine_threadsafe(
        init_bxipci_device(
            5,
            126,
            master_id=1,
            is_canfd=True,
            hw_type=libstark.StarkHardwareType.Revo2Basic,
        ),
        loop,
    ).result()

    node = HandNode(ctx, loop)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        asyncio.run_coroutine_threadsafe(cleanup_bxipci_device(ctx), loop).result()
        stop_bxipci_runtime()
        loop.call_soon_threadsafe(loop.stop)
        loop_thread.join()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

## 演示内容

`bxi_revo2_example.cpp` 和 `bxi_revo2_example.py` 包含同一组 demo：

1. **基本位置控制** — 握拳、张开、单指移动
2. **速度/电流/PWM 控制** — 单指和多指速度、电流、PWM 控制
3. **Revo2 高级控制** — 单位模式、位置+时间、位置+速度、多指联动
4. **动作序列** — Open、Fist、Pinch、Point 等内置手势
5. **设备信息和配置查询** — 通信参数、系统参数、手指参数

## 项目结构

```
bxi_revo2_example/
├── CMakeLists.txt          # CMake 构建配置
├── download-lib.sh          # SDK 下载脚本（编译前必须执行）
├── package.xml              # ROS 2 包描述
├── VERSION                  # SDK 版本记录 (由 download-lib.sh 生成)
├── src/
│   ├── bxi_can_node.hpp       # C++ BxiPci CAN/CANFD ROS bridge
│   ├── bxi_can_node.py        # Python BxiPci CAN/CANFD ROS bridge
│   ├── bxi_revo2_example.cpp  # C++ 示例
│   └── bxi_revo2_example.py   # Python 示例
└── dist/                    # SDK 下载目录 (由 download-lib.sh 生成，被 .gitignore 忽略)
    ├── include/
    │   ├── stark-sdk.h
    │   └── zlgcan/
    └── shared/linux/
        └── libbc_stark_sdk.so
```
