# systemwatch

这里是线程监控模块，主要任务是监控各个注册的线程是否正常运行。目前策略是如果检测到有线程发生阻塞则直接reset

## 模块细节说明

1. > 模块注册函数介绍
   >
   > 主要功能是初始化任务列表，然后创建systemwatch线程。
   >
   > (注意threadx使用的是静态内存分配，所以需要引入字节池来分配线程所需栈空间）
   >
   > >注意分配多少就用多少，严禁过大或过小

   ```c
   void SystemWatch_Init(TX_BYTE_POOL *pool)
   {
       // 初始化任务列表
       memset(taskList, 0, sizeof(taskList));
       taskCount = 0;
       watch_task_last_active = tx_time_get();
   
   
       #if SystemWatch_Enable == 1
       // 用内存池分配监控线程栈
       CHAR *watch_thread_stack;
       if (tx_byte_allocate(pool, (VOID **)&watch_thread_stack, 1024, TX_NO_WAIT) != TX_SUCCESS) {
           log_e("Failed to allocate stack for WatchTask!");
           return;
       }
   
       static TX_THREAD watch_thread;
       UINT status = tx_thread_create(&watch_thread, "WatchTask", SystemWatch_Task, 0,
                                      watch_thread_stack, 1024,
                                      3, 3, TX_NO_TIME_SLICE, TX_AUTO_START);
   
       if(status != TX_SUCCESS) {
           log_e("Failed to create SystemWatch task!");
           return;
       }
       watchTaskHandle = &watch_thread;
   
       SystemWatch_RegisterTask(watchTaskHandle, "WatchTask");
   
       #endif
   
       log_i("SystemWatch initialized, watch task created.");
   }
   ```
   
2. > systemwatch任务说明：
   >
   > > 任务：检查线程状态，喂狗。如果检测到某个线程阻塞，则先进入临界区，关闭所有中断，打印对应线程数据，然后退出临界区，再reset重启

   ```c
   static void SystemWatch_Task(ULONG thread_input)
   {
       #if SystemWatch_Iwdg_Enable == 1
       __HAL_DBGMCU_FREEZE_IWDG();
       MX_IWDG_Init();
       #endif
       (void)thread_input;
       while(1) {
           HAL_IWDG_Refresh(&hiwdg);
           taskList[0].dt = DWT_GetDeltaT(&taskList[0].dt_cnt); //自身更新
           for(uint8_t i = 0; i < taskCount; i++) {
               if(taskList[i].isActive) {
                   // 检查任务执行间隔是否过长
                   if(taskList[i].dt > TASK_BLOCK_TIMEOUT) {
                       // ThreadX临界区
                       UINT old_posture = tx_interrupt_control(TX_INT_DISABLE);
                       log_e("\r\n**** Task Blocked Detected! System State Dump ****");
                       DWT_Delay(0.005);
                       log_e("Time: %.3f s", DWT_GetTimeline_s());
                       DWT_Delay(0.005);
                       log_e("----------------------------------------");
                       DWT_Delay(0.005);
                       log_e("Blocked Task Information:");
                       DWT_Delay(0.005);
                       PrintTaskInfo(&taskList[i]);
                       DWT_Delay(0.5);
                       tx_interrupt_control(old_posture);
   
                       #if SystemWatch_Reset_Enable == 1
                       HAL_NVIC_SystemReset(); 
                       #endif
                   }
               }
           }
           tx_thread_sleep(10);
       }
   }
   ```

3. > 使用示例：
   >
   > ```c
   > SystemWatch_Init(TX_BYTE_POOL *pool);
   > //给需要的线程注册
   > TX_THREAD shootTask_thread;
   > SystemWatch_RegisterTask(&shootTask_thread,"shoot task");
   > //更新线程存活时间
   > SystemWatch_ReportTaskAlive(&shootTask_thread);//注意，需要放入对应的任务的while循环里
   > ```

   