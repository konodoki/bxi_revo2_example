#include <cstdio>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include "stark-sdk.h"
#include "stark_common.h"
#include "can_common.h"
using namespace std::chrono_literals;
DeviceContext ctx;
class bxi_revo2_demo : public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr timer_;

public:
    bxi_revo2_demo()
        : rclcpp::Node("bxi_revo2_demo")
    {
        timer_ = this->create_wall_timer(3s, [&]() {
            static bool stage = false;
            if (stage) {
                uint16_t positions_fist[] = { 500, 500, 1000, 1000, 1000, 1000 };
                stark_set_finger_positions(ctx.handle, 126, positions_fist, 6);
            } else {
                uint16_t positions_open[] = { 0, 0, 0, 0, 0, 0 };
                stark_set_finger_positions(ctx.handle, 126, positions_open, 6);
            }
            stage = !stage;
        });
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::memset(&ctx, 0, sizeof(ctx));
    ctx.hw_type_override = STARK_HARDWARE_TYPE_REVO2_BASIC;
    init_bxipci_device(&ctx, 126, true);

    auto demo_node = std::make_shared<bxi_revo2_demo>();
    rclcpp::executors::MultiThreadedExecutor executer; //注意一定要是多线程执行
    executer.add_node(bxi_revo2_can_);
    executer.add_node(demo_node);
    executer.spin();

    cleanup_device_context(&ctx);
    rclcpp::shutdown();
    return 0;
}
