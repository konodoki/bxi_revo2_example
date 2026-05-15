/**
 * @file can_common.cpp
 * @brief BxiPci-only CAN/CANFD bridge for the Stark SDK callbacks.
 */

#include "can_common.h"
#include "stark-sdk.h"

#include <memory>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

std::shared_ptr<bxi_revo2_can_node> bxi_revo2_can_;

static bool g_bxipci_initialized = false;

static bool ensure_bxipci_bridge(void)
{
  if (g_bxipci_initialized) {
    return bxi_revo2_can_ != nullptr;
  }

  bxi_revo2_can_ = std::make_shared<bxi_revo2_can_node>(RX_BUFF_SIZE);
  g_bxipci_initialized = true;
  printf("[BxiPci] ROS CAN bridge initialized successfully\n");
  return true;
}

bool init_can_device(void)
{
  return ensure_bxipci_bridge();
}

bool init_canfd_device(void)
{
  return ensure_bxipci_bridge();
}

bool start_can_channel(void)
{
  return ensure_bxipci_bridge();
}

bool start_canfd_channel(void)
{
  return ensure_bxipci_bridge();
}

void setup_can_callbacks(void)
{
  set_can_tx_callback([](uint8_t slave_id,
                         uint32_t can_id,
                         const uint8_t *data,
                         uintptr_t data_len) -> int {
    (void)slave_id;
    if (!bxi_revo2_can_) return -1;

    canfd_packet packet;
    memset(&packet, 0, sizeof(packet));
    packet.bus = BXI_PCI_BUS_INDEX;
    packet.frame.can_id = can_id;
    packet.frame.len = (data_len > 8) ? 8 : static_cast<uint8_t>(data_len);
    memcpy(packet.frame.data, data, packet.frame.len);

    const int ret = bxi_revo2_can_->send_packet(packet);
    return (ret > 0) ? 0 : -1;
  });

  set_can_rx_callback([](uint8_t slave_id,
                         uint32_t expected_can_id,
                         uint8_t expected_frames,
                         uint32_t *can_id_out,
                         uint8_t *data_out,
                         uintptr_t *data_len_out) -> int {
    (void)slave_id;
    if (!bxi_revo2_can_) return -1;

    return bxi_revo2_can_->receive_can(expected_can_id,
                                       expected_frames,
                                       can_id_out,
                                       data_out,
                                       data_len_out,
                                       RX_WAIT_TIME);
  });
}

void setup_canfd_callbacks(void)
{
  set_can_tx_callback([](uint8_t slave_id,
                         uint32_t canfd_id,
                         const uint8_t *data,
                         uintptr_t data_len) -> int {
    (void)slave_id;
    if (!bxi_revo2_can_) return -1;

    canfd_packet packet;
    memset(&packet, 0, sizeof(packet));
    packet.bus = BXI_PCI_BUS_INDEX;
    packet.frame.can_id = (canfd_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
    packet.frame.len = (data_len > 64) ? 64 : static_cast<uint8_t>(data_len);
    packet.frame.flags = CANFD_BRS | CANFD_FDF;
    memcpy(packet.frame.data, data, packet.frame.len);

    const int ret = bxi_revo2_can_->send_packet(packet);
    return (ret > 0) ? 0 : -1;
  });

  set_can_rx_callback([](uint8_t slave_id,
                         uint32_t expected_can_id,
                         uint8_t expected_frames,
                         uint32_t *canfd_id_out,
                         uint8_t *data_out,
                         uintptr_t *data_len_out) -> int {
    (void)slave_id;
    (void)expected_frames;
    if (!bxi_revo2_can_) return -1;

    return bxi_revo2_can_->receive_canfd(expected_can_id,
                                         canfd_id_out,
                                         data_out,
                                         data_len_out,
                                         RX_WAIT_TIME);
  });
}

bool setup_can(void)
{
  printf("Setting up BxiPci CAN...\n");

  if (!init_can_device()) {
    printf("[ERROR] Failed to initialize BxiPci CAN bridge\n");
    return false;
  }

  setup_can_callbacks();
  printf("BxiPci CAN setup completed successfully\n");
  return true;
}

bool setup_canfd(void)
{
  printf("Setting up BxiPci CANFD...\n");

  if (!init_canfd_device()) {
    printf("[ERROR] Failed to initialize BxiPci CANFD bridge\n");
    return false;
  }

  setup_canfd_callbacks();
  printf("BxiPci CANFD setup completed successfully\n");
  return true;
}

void cleanup_can_resources(void)
{
  if (bxi_revo2_can_) {
    bxi_revo2_can_->clear_rx_queue();
    bxi_revo2_can_.reset();
  }

  g_bxipci_initialized = false;
  printf("[BxiPci] Resources cleaned up\n");
}
