#ifndef CDC_ACM_H
#define CDC_ACM_H

#include <stdint.h>

typedef void (*USB_RxCpltCallback_t)(uint8_t* Buf, uint32_t Len);

void cdc_acm_send(uint8_t busid,uint8_t* buffer,uint8_t size);
void USB_RegisterCallback(USB_RxCpltCallback_t callback);

#endif // CDC_ACM_H
