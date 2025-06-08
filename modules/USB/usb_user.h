#ifndef USB_USER_H
#define USB_USER_H

#include <stdint.h>

#pragma pack(1)
struct Sentry_Send_s
{
  uint8_t header;
  uint16_t projectile_allowance_17mm;  //剩余发弹量
  uint8_t power_management_shooter_output; // 功率管理 shooter 输出
  uint16_t current_hp_percent; // 机器人当前血量百分比
  uint16_t outpost_HP;     //前哨站血量
  uint16_t base_HP;        //基地血量
  uint8_t game_progess;
  uint16_t game_time;
  uint8_t mode;
  float roll;
  float pitch;
  float yaw;
  uint8_t end;
};

struct Recv_s
{
  uint8_t header; 
  uint8_t fire_advice;
  float pitch;
  float yaw;
  float distance;
  uint8_t nav_state;
  float vx;
  float vy;
  float wz;
  uint8_t tail;
};

#pragma pack() 

typedef struct{
  struct Recv_s recv;
  uint8_t offline_index;
}vcom_receive_t;

void usb_user_init(void);
void usb_user_send(const struct Sentry_Send_s *send);
vcom_receive_t* usb_user_get_vcom_receive(void);

#endif // USB_USER_H
