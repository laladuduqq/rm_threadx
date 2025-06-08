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
   
       log_i("SystemWatch initialized, watch task created.");
   }
   ```

2. 