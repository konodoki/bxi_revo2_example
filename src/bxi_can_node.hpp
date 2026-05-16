#pragma once
#include "linux/can.h"
#include "communication/msg/canfd_packet.hpp"
#include "stark-sdk.h"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

inline constexpr uint32_t BXI_PCI_DEFAULT_BUS_INDEX = 5;
inline constexpr uint8_t BXI_PCI_DEFAULT_MASTER_ID = 1;
inline constexpr int BXI_PCI_RX_WAIT_TIME_MS = 250;
inline constexpr std::size_t BXI_PCI_RX_BUFFER_SIZE = 1000;
typedef struct
{
    unsigned int bus;
    struct canfd_frame frame;
}canfd_packet __attribute__((__aligned__(8)));

typedef int (*canfd_rx_call)(void *arg, canfd_packet *msg);

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

    ~BxiRosCanBridge() override
    {
        clear_rx_queue();
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

    void set_default_bus(uint32_t bus)
    {
        std::lock_guard<std::mutex> lock(bus_mutex_);
        default_bus_ = bus;
    }

    void register_slave_bus(uint8_t slave_id, uint32_t bus)
    {
        std::lock_guard<std::mutex> lock(bus_mutex_);
        slave_bus_[slave_id] = bus;
    }

    uint32_t bus_for_slave(uint8_t slave_id)
    {
        std::lock_guard<std::mutex> lock(bus_mutex_);
        auto it = slave_bus_.find(slave_id);
        return (it == slave_bus_.end()) ? default_bus_ : it->second;
    }

    void push_rx_packet(const canfd_packet& msg)
    {
        RxFrame frame;
        frame.bus = msg.bus;
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

    int receive_can(uint32_t expected_bus,
                    uint32_t expected_can_id,
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

        auto matches = [expected_bus, expected_can_id, is_dfu_mode](const RxFrame& frame) {
            if (frame.bus != expected_bus) {
                return false;
            }
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

    int receive_canfd(uint32_t expected_bus,
                      uint32_t expected_can_id,
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

        auto matches = [expected_bus, expected_slave_id, expected_master_id](const RxFrame& frame) {
            if (frame.bus != expected_bus) {
                return false;
            }
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
        uint32_t bus = 0;
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
    std::mutex bus_mutex_;
    uint32_t default_bus_ = BXI_PCI_DEFAULT_BUS_INDEX;
    std::unordered_map<uint8_t, uint32_t> slave_bus_;
    std::size_t dropped_rx_frames_ = 0;
};

using bxi_revo2_can_node = BxiRosCanBridge;

class BxiPciRosRuntime {
public:
    static BxiPciRosRuntime& instance()
    {
        static BxiPciRosRuntime runtime;
        return runtime;
    }

    BxiPciRosRuntime(const BxiPciRosRuntime&) = delete;
    BxiPciRosRuntime& operator=(const BxiPciRosRuntime&) = delete;

    ~BxiPciRosRuntime()
    {
        stop();
    }

    std::shared_ptr<bxi_revo2_can_node> ensure_bridge(uint32_t default_bus)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!rclcpp::ok()) {
            printf("[ERROR] rclcpp::init() must be called before init_bxipci_device()\n");
            return nullptr;
        }

        if (!bridge_) {
            bridge_ = std::make_shared<bxi_revo2_can_node>(BXI_PCI_RX_BUFFER_SIZE);
            executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
                rclcpp::ExecutorOptions(), 2);
            executor_->add_node(bridge_);
            spin_thread_ = std::thread([this]() {
                executor_->spin();
            });
            printf("[BxiPci] ROS CAN bridge initialized successfully\n");
        }

        bridge_->set_default_bus(default_bus);
        return bridge_;
    }

    std::shared_ptr<bxi_revo2_can_node> current_bridge()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return bridge_;
    }

    void stop()
    {
        std::thread spin_thread;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (executor_) {
                executor_->cancel();
            }
            spin_thread = std::move(spin_thread_);
        }

        if (spin_thread.joinable()) {
            spin_thread.join();
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (executor_ && bridge_) {
            try {
                executor_->remove_node(bridge_);
            } catch (...) {
            }
        }
        executor_.reset();
        bridge_.reset();
    }

private:
    BxiPciRosRuntime() = default;

    std::mutex mutex_;
    std::shared_ptr<bxi_revo2_can_node> bridge_;
    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread spin_thread_;
};

struct BxiDeviceContext {
    DeviceHandler *handle = nullptr;
    uint32_t bus = BXI_PCI_DEFAULT_BUS_INDEX;
    uint8_t master_id = BXI_PCI_DEFAULT_MASTER_ID;
    uint8_t slave_id = 0;
    bool is_canfd = true;
    StarkHardwareType hw_type_override = static_cast<StarkHardwareType>(0);

    BxiDeviceContext() = default;
    BxiDeviceContext(const BxiDeviceContext&) = delete;
    BxiDeviceContext& operator=(const BxiDeviceContext&) = delete;

    ~BxiDeviceContext()
    {
        close();
    }

    void close()
    {
        if (handle) {
            close_device_handler(handle, stark_get_protocol_type(handle));
            handle = nullptr;
        }
    }
};

inline std::shared_ptr<bxi_revo2_can_node> ensure_bxipci_bridge(uint32_t default_bus = BXI_PCI_DEFAULT_BUS_INDEX)
{
    return BxiPciRosRuntime::instance().ensure_bridge(default_bus);
}

inline void setup_bxipci_can_callbacks()
{
    set_can_tx_callback([](uint8_t slave_id,
                           uint32_t can_id,
                           const uint8_t *data,
                           uintptr_t data_len) -> int {
        auto bridge = BxiPciRosRuntime::instance().current_bridge();
        if (!bridge) return -1;

        canfd_packet packet;
        std::memset(&packet, 0, sizeof(packet));
        packet.bus = bridge->bus_for_slave(slave_id);
        packet.frame.can_id = can_id;
        packet.frame.len = (data_len > 8) ? 8 : static_cast<uint8_t>(data_len);
        std::memcpy(packet.frame.data, data, packet.frame.len);

        const int ret = bridge->send_packet(packet);
        return (ret > 0) ? 0 : -1;
    });

    set_can_rx_callback([](uint8_t slave_id,
                           uint32_t expected_can_id,
                           uint8_t expected_frames,
                           uint32_t *can_id_out,
                           uint8_t *data_out,
                           uintptr_t *data_len_out) -> int {
        auto bridge = BxiPciRosRuntime::instance().current_bridge();
        if (!bridge) return -1;

        return bridge->receive_can(bridge->bus_for_slave(slave_id),
                                   expected_can_id,
                                   expected_frames,
                                   can_id_out,
                                   data_out,
                                   data_len_out,
                                   BXI_PCI_RX_WAIT_TIME_MS);
    });
}

inline void setup_bxipci_canfd_callbacks()
{
    set_can_tx_callback([](uint8_t slave_id,
                           uint32_t canfd_id,
                           const uint8_t *data,
                           uintptr_t data_len) -> int {
        auto bridge = BxiPciRosRuntime::instance().current_bridge();
        if (!bridge) return -1;

        canfd_packet packet;
        std::memset(&packet, 0, sizeof(packet));
        packet.bus = bridge->bus_for_slave(slave_id);
        packet.frame.can_id = (canfd_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
        packet.frame.len = (data_len > 64) ? 64 : static_cast<uint8_t>(data_len);
        packet.frame.flags = CANFD_BRS | CANFD_FDF;
        std::memcpy(packet.frame.data, data, packet.frame.len);

        const int ret = bridge->send_packet(packet);
        return (ret > 0) ? 0 : -1;
    });

    set_can_rx_callback([](uint8_t slave_id,
                           uint32_t expected_can_id,
                           uint8_t expected_frames,
                           uint32_t *canfd_id_out,
                           uint8_t *data_out,
                           uintptr_t *data_len_out) -> int {
        (void)expected_frames;
        auto bridge = BxiPciRosRuntime::instance().current_bridge();
        if (!bridge) return -1;

        return bridge->receive_canfd(bridge->bus_for_slave(slave_id),
                                     expected_can_id,
                                     canfd_id_out,
                                     data_out,
                                     data_len_out,
                                     BXI_PCI_RX_WAIT_TIME_MS);
    });
}

inline bool init_bxipci_device(BxiDeviceContext* ctx,
                               uint32_t bus,
                               uint8_t slave_id,
                               bool is_canfd)
{
    if (!ctx) {
        return false;
    }

    printf("\n[Init] Mode: BxiPci %s\n", is_canfd ? "(CANFD)" : "(CAN 2.0)");
    printf("  Bus: %u, Master ID: %u, Slave ID: %u\n",
           bus,
           ctx->master_id,
           slave_id);

    auto bridge = ensure_bxipci_bridge(bus);
    if (!bridge) {
        printf("[ERROR] Failed to initialize BxiPci ROS CAN bridge\n");
        return false;
    }

    bridge->register_slave_bus(slave_id, bus);
    if (is_canfd) {
        setup_bxipci_canfd_callbacks();
    } else {
        setup_bxipci_can_callbacks();
    }

    StarkProtocolType protocol = is_canfd ? STARK_PROTOCOL_TYPE_CAN_FD : STARK_PROTOCOL_TYPE_CAN;
    uint32_t arb_baudrate = 1000000;
    uint32_t data_baudrate = is_canfd ? 5000000 : 1000000;

    if (ctx->hw_type_override != 0) {
        printf("  Hardware type override: %d\n", ctx->hw_type_override);
        ctx->handle = init_device_handler_can_with_hw_type(protocol,
                                                           ctx->master_id,
                                                           slave_id,
                                                           arb_baudrate,
                                                           data_baudrate,
                                                           ctx->hw_type_override);
    } else {
        ctx->handle = init_device_handler_can(protocol,
                                              ctx->master_id,
                                              arb_baudrate,
                                              data_baudrate);
    }

    if (ctx->handle == NULL) {
        printf("[ERROR] Failed to create device handler\n");
        return false;
    }

    ctx->bus = bus;
    ctx->slave_id = slave_id;
    ctx->is_canfd = is_canfd;
    return true;
}

inline bool init_bxipci_device(BxiDeviceContext* ctx,
                               uint32_t bus,
                               uint8_t slave_id,
                               bool is_canfd,
                               uint8_t master_id)
{
    if (!ctx) {
        return false;
    }
    ctx->master_id = master_id;
    return init_bxipci_device(ctx, bus, slave_id, is_canfd);
}

inline bool init_bxipci_device(BxiDeviceContext* ctx, uint8_t slave_id, bool is_canfd)
{
    return init_bxipci_device(ctx, BXI_PCI_DEFAULT_BUS_INDEX, slave_id, is_canfd);
}

inline bool init_bxipci_device(BxiDeviceContext* ctx,
                               uint8_t slave_id,
                               bool is_canfd,
                               uint8_t master_id)
{
    return init_bxipci_device(ctx,
                              BXI_PCI_DEFAULT_BUS_INDEX,
                              slave_id,
                              is_canfd,
                              master_id);
}
