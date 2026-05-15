#pragma once

#include "bxi_pci_drv.h"
#include "communication/msg/canfd_packet.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

class BxiRosCanBridge : public rclcpp::Node {
public:
    explicit BxiRosCanBridge(std::size_t max_rx_frames = 1000)
        : rclcpp::Node("bxi_revo2_can_node")
        , max_rx_frames_(max_rx_frames)
    {
        canfd_sub_ = this->create_subscription<communication::msg::CANFDPacket>(
            "canfd_packet/rx", rclcpp::QoS(100),
            [this](communication::msg::CANFDPacket::SharedPtr msg) {
                if (!msg) {
                    RCLCPP_ERROR(this->get_logger(), "Received null CANFD packet");
                    return;
                }

                canfd_packet packet;
                if (!to_canfd_packet(*msg, packet)) {
                    return;
                }

                push_rx_packet(packet);
                RCLCPP_DEBUG(this->get_logger(), "CANFD RX bus=%u id=0x%08X len=%u",
                             packet.bus, packet.frame.can_id, packet.frame.len);
            });

        canfd_pub_ = this->create_publisher<communication::msg::CANFDPacket>(
            "canfd_packet/tx", rclcpp::QoS(100));
    }

    int send_packet(const canfd_packet& msg)
    {
        if (msg.frame.len > kMaxCanFdDataLen) {
            RCLCPP_ERROR(this->get_logger(), "TX frame length %u exceeds %zu",
                         msg.frame.len, kMaxCanFdDataLen);
            return -1;
        }

        communication::msg::CANFDPacket packet;
        packet.bus = msg.bus;
        packet.frame.can_id = msg.frame.can_id;
        packet.frame.flags = msg.frame.flags;
        packet.frame.len = msg.frame.len;
        resize_if_supported(packet.frame.data, packet.frame.len, 0);

        if (packet.frame.data.size() < packet.frame.len) {
            RCLCPP_ERROR(this->get_logger(), "TX message data size %zu is smaller than len %u",
                         packet.frame.data.size(), packet.frame.len);
            return -1;
        }

        std::copy_n(msg.frame.data, packet.frame.len, packet.frame.data.begin());
        canfd_pub_->publish(packet);
        RCLCPP_DEBUG(this->get_logger(), "CANFD TX bus=%u id=0x%08X len=%u",
                     msg.bus, msg.frame.can_id, msg.frame.len);
        return 1;
    }

    int canfd_send_packet(canfd_packet *msg)
    {
        if (!msg) {
            RCLCPP_ERROR(this->get_logger(), "TX packet is null");
            return -1;
        }
        return send_packet(*msg);
    }

    void push_rx_packet(const canfd_packet& msg)
    {
        RxFrame frame;
        frame.can_id = msg.frame.can_id & CAN_EFF_MASK;
        frame.len = static_cast<uint8_t>(
            std::min<std::size_t>(msg.frame.len, kMaxCanFdDataLen));
        std::copy_n(msg.frame.data, frame.len, frame.data);

        {
            std::lock_guard<std::mutex> lock(rx_mutex_);
            if (rx_frames_.size() >= max_rx_frames_) {
                rx_frames_.pop_front();
                dropped_rx_frames_++;
            }
            rx_frames_.push_back(frame);
        }
        rx_cv_.notify_all();
    }

    int receive_can(uint32_t expected_can_id,
                    uint8_t expected_frames,
                    uint32_t *can_id_out,
                    uint8_t *data_out,
                    uintptr_t *data_len_out,
                    int wait_time_ms)
    {
        if (!can_id_out || !data_out || !data_len_out) {
            return -1;
        }

        int total_dlc = 0;
        int received_count = 0;
        const bool is_dfu_mode = (expected_can_id == 0);
        const uint8_t cmd = (expected_can_id >> 3) & 0x0F;
        const bool is_multi_frame_cmd = (cmd == 0x0B || cmd == 0x0D);
        const int max_attempts = is_dfu_mode ? 200 : (expected_frames > 1 || is_multi_frame_cmd ? 5 : 2);

        auto matches = [expected_can_id, is_dfu_mode](const RxFrame& frame) {
            return is_dfu_mode || expected_can_id == 0 || frame.can_id == expected_can_id;
        };

        for (int attempt = 0; attempt < max_attempts; attempt++) {
            RxFrame frame;
            if (!wait_for_frame(matches, frame, std::chrono::milliseconds(wait_time_ms))) {
                continue;
            }

            const uint32_t can_id = frame.can_id;
            const int can_dlc = frame.len;

            if (is_multi_frame_cmd && can_dlc > 0) {
                const uint8_t frame_header = frame.data[0];

                if (cmd == 0x0B && can_dlc >= 2) {
                    const uint8_t len_and_flag = frame.data[1];
                    const bool is_last = (len_and_flag & 0x80) != 0;
                    append_frame_data(frame, data_out, total_dlc);
                    received_count++;

                    if (is_last) {
                        *can_id_out = can_id;
                        *data_len_out = total_dlc;
                        return 0;
                    }
                    continue;
                }

                if (cmd == 0x0D) {
                    const uint8_t total = (frame_header >> 4) & 0x0F;
                    const uint8_t seq = frame_header & 0x0F;

                    if (total > 0 && seq > 0) {
                        append_frame_data(frame, data_out, total_dlc);
                        received_count++;

                        if (received_count >= total) {
                            *can_id_out = can_id;
                            *data_len_out = total_dlc;
                            return 0;
                        }
                        continue;
                    }
                }
            }

            append_frame_data(frame, data_out, total_dlc);
            *can_id_out = can_id;
            *data_len_out = total_dlc;
            return 0;
        }

        if (total_dlc > 0) {
            *can_id_out = expected_can_id;
            *data_len_out = total_dlc;
            return 0;
        }

        return -1;
    }

    int receive_canfd(uint32_t expected_can_id,
                      uint32_t *canfd_id_out,
                      uint8_t *data_out,
                      uintptr_t *data_len_out,
                      int wait_time_ms)
    {
        if (!canfd_id_out || !data_out || !data_len_out) {
            return -1;
        }

        const uint8_t expected_slave_id = (expected_can_id >> 16) & 0xFF;
        const uint8_t expected_master_id = (expected_can_id >> 8) & 0xFF;

        auto matches = [expected_slave_id, expected_master_id](const RxFrame& frame) {
            const uint8_t resp_slave_id = (frame.can_id >> 16) & 0xFF;
            const uint8_t resp_master_id = (frame.can_id >> 8) & 0xFF;
            return resp_slave_id == expected_slave_id && resp_master_id == expected_master_id;
        };

        for (int attempt = 0; attempt < 2; attempt++) {
            RxFrame frame;
            if (!wait_for_frame(matches, frame, std::chrono::milliseconds(wait_time_ms))) {
                continue;
            }

            *canfd_id_out = frame.can_id;
            *data_len_out = frame.len;
            std::copy_n(frame.data, frame.len, data_out);
            return 0;
        }

        return -1;
    }

    void clear_rx_queue()
    {
        std::lock_guard<std::mutex> lock(rx_mutex_);
        rx_frames_.clear();
    }

private:
    static constexpr std::size_t kMaxCanFdDataLen = 64;

    struct RxFrame {
        uint32_t can_id = 0;
        uint8_t data[kMaxCanFdDataLen] = {};
        uint8_t len = 0;
    };

    template <typename Container>
    static auto resize_if_supported(Container& data, std::size_t len, int)
        -> decltype(data.resize(len), void())
    {
        data.resize(len);
    }

    template <typename Container>
    static void resize_if_supported(Container&, std::size_t, long)
    {
    }

    bool to_canfd_packet(const communication::msg::CANFDPacket& msg, canfd_packet& packet)
    {
        if (msg.frame.len > kMaxCanFdDataLen) {
            RCLCPP_ERROR(this->get_logger(), "RX frame length %u exceeds %zu",
                         msg.frame.len, kMaxCanFdDataLen);
            return false;
        }

        if (msg.frame.data.size() < msg.frame.len) {
            RCLCPP_ERROR(this->get_logger(), "RX message data size %zu is smaller than len %u",
                         msg.frame.data.size(), msg.frame.len);
            return false;
        }

        std::memset(&packet, 0, sizeof(packet));
        packet.bus = msg.bus;
        packet.frame.can_id = msg.frame.can_id;
        packet.frame.flags = msg.frame.flags;
        packet.frame.len = msg.frame.len;
        std::copy_n(msg.frame.data.begin(), packet.frame.len, packet.frame.data);
        return true;
    }

    void append_frame_data(const RxFrame& frame, uint8_t *data_out, int& total_dlc)
    {
        std::copy_n(frame.data, frame.len, data_out + total_dlc);
        total_dlc += frame.len;
    }

    bool wait_for_frame(const std::function<bool(const RxFrame&)>& matches,
                        RxFrame& frame_out,
                        std::chrono::milliseconds timeout)
    {
        std::unique_lock<std::mutex> lock(rx_mutex_);
        const auto deadline = std::chrono::steady_clock::now() + timeout;

        while (true) {
            for (auto it = rx_frames_.begin(); it != rx_frames_.end(); ++it) {
                if (matches(*it)) {
                    frame_out = *it;
                    rx_frames_.erase(it);
                    return true;
                }
            }

            if (rx_cv_.wait_until(lock, deadline) == std::cv_status::timeout) {
                return false;
            }
        }
    }

    rclcpp::Subscription<communication::msg::CANFDPacket>::SharedPtr canfd_sub_;
    rclcpp::Publisher<communication::msg::CANFDPacket>::SharedPtr canfd_pub_;
    std::size_t max_rx_frames_;
    std::deque<RxFrame> rx_frames_;
    std::mutex rx_mutex_;
    std::condition_variable rx_cv_;
    std::size_t dropped_rx_frames_ = 0;
};

using bxi_revo2_can_node = BxiRosCanBridge;
