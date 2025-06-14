# IMU

算法介绍：

[# RoboMaster机器人姿态解算方案开源](https://zhuanlan.zhihu.com/p/540676773)

参考代码：

https://github.com/WangHongxi2001/RoboMaster-C-Board-INS-Example

[modules/imu/ins_task.c · HNUYueLuRM/basic_framework - Gitee.com](https://gitee.com/hnuyuelurm/basic_framework/blob/master/modules/imu/ins_task.c)



使用示例：

这是获取值

```c
const IMU_DATA_T* imu = INS_GetData();
float yaw = *(imu->Yaw);           // 使用解引用获取值
float pitch = *(imu->Pitch);
float roll = *(imu->Roll);
float yawTotal = *(imu->YawTotalAngle);
const float gyro = *imu->gyro[2];  // 获取陀螺仪数据指针
```

指针操作：

```c
const IMU_DATA_T* imu = INS_GetData();

struct {
    const float *other_angle_feedback_ptr;  
    const float *other_speed_feedback_ptr; 
} control = {
    .other_angle_feedback_ptr = imu->YawTotalAngle,
    .other_speed_feedback_ptr = &((*imu->gyro)[2]),
};
```






