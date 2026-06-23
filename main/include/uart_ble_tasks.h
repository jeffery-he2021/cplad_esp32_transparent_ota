#ifndef __UART_TASKS_H
#define __UART_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

#define ITEM_BUF_SIZE      256 // 项目中缓存区大小
#define QUEUE_LENGTH       1       // 队列最大元素个数         
// 队列数据结构：保存 实际长度 + 数据
typedef struct {
    size_t len;
    uint8_t data[ITEM_BUF_SIZE];
} queue_item_t;

extern QueueHandle_t q_uart2ble;
extern QueueHandle_t q_ble2uart;
extern bool notify_state_transparent;
extern SemaphoreHandle_t sem_transparent_notify;
// Function prototypes for UART tasks
void uart_task_init(void);

#ifdef __cplusplus
}
#endif

#endif // __UART_TASKS_H