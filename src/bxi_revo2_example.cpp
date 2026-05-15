#include <cstdio>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include "bxi_can_node.hpp"
#include "stark-sdk.h"
#include <thread>
using namespace std::chrono_literals;

static void print_device_info(DeviceHandler *handle, uint8_t slave_id)
{
    CDeviceInfo *info = stark_get_device_info(handle, slave_id);
    if (!info) {
        printf("[WARN] Failed to get device info\n");
        return;
    }

    printf("Device Info:\n");
    printf("  Hardware Type: %d\n", info->hardware_type);
    printf("  Serial Number: %s\n", info->serial_number ? info->serial_number : "");
    printf("  Firmware Version: %s\n", info->firmware_version ? info->firmware_version : "");
    free_device_info(info);
}

class bxi_revo2_demo : public rclcpp::Node {
private:
    BxiDeviceContext& left_ctx_;
    BxiDeviceContext& right_ctx_;
    rclcpp::TimerBase::SharedPtr info_timer_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    explicit bxi_revo2_demo(BxiDeviceContext& left_ctx_,BxiDeviceContext& right_ctx_)
        : rclcpp::Node("bxi_revo2_demo")
        , left_ctx_(left_ctx_)
        , right_ctx_(right_ctx_)
    {
        info_timer_ = this->create_wall_timer(3s, [&]() {
            info_timer_->cancel();
            print_device_info(left_ctx_.handle, left_ctx_.slave_id);
            print_device_info(right_ctx_.handle, right_ctx_.slave_id);
            start_motion_timer();
        });
    }

private:
    void start_motion_timer()
    {
        timer_ = this->create_wall_timer(3s, [this]() {
            static bool stage = false;
            if (stage) {
                uint16_t positions_fist[] = { 500, 500, 1000, 1000, 1000, 1000 };
                stark_set_finger_positions(left_ctx_.handle, left_ctx_.slave_id, positions_fist, 6);
                stark_set_finger_positions(right_ctx_.handle, right_ctx_.slave_id, positions_fist, 6);
            } else {
                uint16_t positions_open[] = { 0, 0, 0, 0, 0, 0 };
                stark_set_finger_positions(left_ctx_.handle, left_ctx_.slave_id, positions_open, 6);
                stark_set_finger_positions(right_ctx_.handle, right_ctx_.slave_id, positions_open, 6);
            }
            stage = !stage;
        });
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    BxiDeviceContext left_ctx_;
    BxiDeviceContext right_ctx_;
    left_ctx_.hw_type_override = STARK_HARDWARE_TYPE_REVO2_BASIC;
    right_ctx_.hw_type_override = STARK_HARDWARE_TYPE_REVO2_BASIC;
    if (!init_bxipci_device(&left_ctx_, 5, 126, true)) {
        rclcpp::shutdown();
        return 1;
    }
    if (!init_bxipci_device(&right_ctx_, 6, 127, true)) {
        rclcpp::shutdown();
        return 1;
    }
    auto demo_node = std::make_shared<bxi_revo2_demo>(left_ctx_,right_ctx_);
    std::thread([&](){
        rclcpp::spin(left_ctx_.bridge);
    }).detach();
    rclcpp::spin(demo_node);
    rclcpp::shutdown();
    return 0;
}
