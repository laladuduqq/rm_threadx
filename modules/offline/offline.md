# offline

这里是离线检测模块

## 使用示例

```c
static uint8_t offline_index;

OfflineDeviceInit_t offline_init = {
        .name = "name", // 不要超过32字节
        .timeout_ms = 100, //检测时间
        .level = OFFLINE_LEVEL_HIGH, //检测等级
        .beep_times = 0, //beep次数
        .enable = OFFLINE_ENABLE, //是否开启离线检测
};
offline_index = offline_device_register(&offline_init);


//更新
offline_device_update(offline_index);
```


