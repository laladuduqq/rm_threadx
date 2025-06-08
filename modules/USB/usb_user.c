#include "usb_user.h"
#include "cdc_acm.h"
#include "offline.h"

static vcom_receive_t vcom_receive;

void usb_callback(uint8_t* Buf, uint32_t Len)
{
    if (Len ==sizeof(struct Recv_s) && Buf[0]==0xFF && Buf[sizeof(struct Recv_s)-1] == 0x0D)
    {
        offline_device_update(vcom_receive.offline_index);
        memcpy(&vcom_receive.recv,Buf,sizeof(struct Recv_s));
    }
}

void usb_user_init(void){
    OfflineDeviceInit_t offline_init = {
        .name = "minipc",
        .timeout_ms = 100,
        .level = OFFLINE_LEVEL_HIGH,
        .beep_times = 9,
        .enable = OFFLINE_DISABLE,
    };
    vcom_receive.offline_index = offline_device_register(&offline_init);
    // 注册回调函数
    USB_RegisterCallback(usb_callback);
}

void usb_user_send(const struct Sentry_Send_s *send)
{
    uint8_t data[sizeof(struct Sentry_Send_s)]={0};
    memcpy(data,send,sizeof(struct Sentry_Send_s));
    cdc_acm_send(0,data, sizeof(struct Sentry_Send_s));
}

vcom_receive_t* usb_user_get_vcom_receive(void)
{
    return &vcom_receive;
}
