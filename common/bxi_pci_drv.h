#ifndef _BXI_PCI_DRV_H
#define _BXI_PCI_DRV_H

#include <linux/can.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define CANFD_DEVICE_NUM 5

typedef struct
{
    unsigned int bus;
    struct canfd_frame frame;
}canfd_packet __attribute__((__aligned__(8)));

typedef int (*canfd_rx_call)(void *arg, canfd_packet *msg);
typedef int (*canfd_event_call)(void *arg, int event); //TODO:event call(e.g.:tx failed)
#ifdef __cplusplus
}
#endif

#endif