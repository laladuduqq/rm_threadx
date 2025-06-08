#include "robot_config.h"
#include "elog.h"
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

//调试参数
void Print_BytePool_Info(TX_BYTE_POOL *pool, const char *name)
{
    ULONG total = pool->tx_byte_pool_size;
    ULONG available = pool->tx_byte_pool_available;
    ULONG used = total - available;
    UINT fragments = pool->tx_byte_pool_fragments;

    log_i("%s: total=%lu bytes, available=%lu bytes, used=%lu bytes, fragments=%u\n",
        name, total, available, used, fragments);
}
