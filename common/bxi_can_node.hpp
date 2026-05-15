#include "bxi_pci_drv.h"
#include "rclcpp/rclcpp.hpp"
#include "communication/msg/canfd_packet.hpp"
#include <algorithm>
#include <communication/msg/detail/canfd_packet__struct.hpp>
#include <functional>
#include <memory>
#include <rclcpp/node.hpp>
#include "linux/can.h"
#include <rclcpp/publisher.hpp>
class bxi_revo2_can_node : public rclcpp::Node {
private:
    rclcpp::Subscription<communication::msg::CANFDPacket>::SharedPtr canfd_sub_;
    rclcpp::Publisher<communication::msg::CANFDPacket>::SharedPtr canfd_pub_;
    canfd_rx_call can_rx;

public:
    bxi_revo2_can_node(canfd_rx_call can_rx)
        : rclcpp::Node("bxi_revo2_can_node")
        , can_rx(can_rx)
    {
        canfd_sub_ = this->create_subscription<communication::msg::CANFDPacket>(
            "canfd_packet/rx", 10,
            [this](communication::msg::CANFDPacket::SharedPtr msg) { // 改为
                                                                     // [this]
                // 检查 msg 是否为空
                if (!msg) {
                    RCLCPP_ERROR(this->get_logger(), "Received null message");
                    return;
                }
                // 检查 msg->frame.data 是否有效
                if (msg->frame.data.empty()) {
                    RCLCPP_WARN(this->get_logger(),
                                "Received empty CAN FD packet");
                    return;
                }

                canfd_packet packet;
                memset(&packet, 0, sizeof(packet)); // 初始化整个结构体

                packet.bus = msg->bus;
                packet.frame.can_id = msg->frame.can_id;
                packet.frame.flags = msg->frame.flags;
                packet.frame.len = msg->frame.len;

                // 确保不会超过数组边界
                if (msg->frame.len > 64) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Frame length %d exceeds 64", msg->frame.len);
                    return;
                }

                std::copy(msg->frame.data.begin(),
                          msg->frame.data.begin() + msg->frame.len,
                          packet.frame.data);

                // 检查 can_rx 函数指针是否有效
                if (this->can_rx) {
                    this->can_rx((void *)0, &packet);
                } else {
                    RCLCPP_ERROR(this->get_logger(),
                                 "can_rx function pointer is null!");
                }

                printf("[CANFD_RX] BUS:%d ID:0x%08X Len:%u Data:",msg->bus, msg->frame.can_id,
                       msg->frame.len);
                for (size_t i = 0; i < msg->frame.len && i < 64; i++) {
                    printf("%02X ", msg->frame.data[i]);
                }
                printf("\n");
            });
        canfd_pub_ = this->create_publisher<communication::msg::CANFDPacket>(
            "canfd_packet/tx", 10);
    }
    int canfd_send_packet(canfd_packet *msg)
    {
        communication::msg::CANFDPacket packet;
        packet.bus = msg->bus;
        packet.frame.can_id = msg->frame.can_id;
        packet.frame.flags = msg->frame.flags;
        std::copy(msg->frame.data, msg->frame.data + msg->frame.len,
                  packet.frame.data.begin());
        packet.frame.len = msg->frame.len;
        canfd_pub_->publish(packet);
        printf("[CANFD_TX] BUS:%d ID:0x%08X Len:%u Data:", msg->bus, msg->frame.can_id,
               msg->frame.len);
        for (uintptr_t i = 0; i < msg->frame.len && i < 64; i++) {
            printf("%02X ", msg->frame.data[i]);
        }
        printf("\n");
        return 1;
    }
};