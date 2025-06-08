#include "robot_config.h"
#include "tx_api.h"

//内存分配
extern TX_BYTE_POOL tx_app_byte_pool;
// 内存分配函数
void* threadx_malloc(size_t size) {
    void *ptr = NULL;
    if (tx_byte_allocate(&tx_app_byte_pool, &ptr, size, TX_NO_WAIT) == TX_SUCCESS) {
        return ptr;
    } else {
        return NULL;
    }
}

// 内存释放函数
void threadx_free(void *ptr) {
    if (ptr != NULL) {
        tx_byte_release(ptr);
    }
}
