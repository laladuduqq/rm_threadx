# usb

这里是usb模块，使用的是cherryusb软件包

官方文档：[cherryusb](https://cherryusb.readthedocs.io/zh-cn/latest/)

bilibili教程：[移植和问题解答](https://www.bilibili.com/cheese/play/ep1548199)

## 使用说明

目前已经针对c板进行适配，使用者只需要关注usb_user.c中的回调函数即可，如果数据量较大，可以修改成事件通知，线程处理方法，修改请参考bsp_uart

首先收发结构体：根据需要决定内容，保证在pragma pack范围内

```c
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
```

2.回调函数 : 这里只需要按照需要修改即可

```c
void usb_callback(uint8_t* Buf, uint32_t Len)
{
    if (Len ==sizeof(struct Recv_s) && Buf[0]==0xFF && Buf[sizeof(struct Recv_s)-1] == 0x0D)
    {
        offline_device_update(vcom_receive.offline_index);
        memcpy(&vcom_receive.recv,Buf,sizeof(struct Recv_s));
    }
}
```



## 发送函数：

```c
static struct Sentry_Send_s sentry_send;

usb_user_send(&sentry_send);
```

## 获取数据：

```c
static vcom_receive_t* vcom_receive;
vcom_receive = usb_user_get_vcom_receive();//这返回的是地址，所以可以在init中调用即可
```


