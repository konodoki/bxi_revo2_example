/**
 * @file can_common.h
 * @brief BxiPci-only CAN/CANFD bridge setup for the Stark SDK callbacks.
 */

#ifndef CAN_COMMON_H
#define CAN_COMMON_H

#include "bxi_can_node.hpp"

#include <memory>
#include <stdbool.h>
#include <stdint.h>

extern std::shared_ptr<bxi_revo2_can_node> bxi_revo2_can_;

#ifdef __cplusplus
extern "C" {
#endif

#define BXI_PCI_BUS_INDEX 5
#define RX_WAIT_TIME 100
#define RX_BUFF_SIZE 1000

bool init_can_device(void);
bool init_canfd_device(void);
bool start_can_channel(void);
bool start_canfd_channel(void);

void setup_can_callbacks(void);
void setup_canfd_callbacks(void);

bool setup_can(void);
bool setup_canfd(void);
void cleanup_can_resources(void);

#ifdef __cplusplus
}
#endif

#endif // CAN_COMMON_H
