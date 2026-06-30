/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "uart_ble_tasks.h"

#include "host/ble_hs.h"
SemaphoreHandle_t sem_transparent_notify;
bool notify_state_transparent = false;
extern uint8_t transparent_payload[20];
extern uint16_t ble_conn_handle;
extern uint16_t transparent_notify_handle;

static const int RX_BUF_SIZE = 256;//1024;

//create queue
QueueHandle_t q_uart2ble;
QueueHandle_t q_ble2uart;
QueueHandle_t q_uart2uart;
int32_t bleWrTimeStart = 0;
int32_t bleWrTimeEnd = 0;


#define TXD_PIN (GPIO_NUM_4)//(GPIO_NUM_20)//
#define RXD_PIN (GPIO_NUM_5)//(GPIO_NUM_19)//

void uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //create queue
    q_uart2ble = xQueueCreate(QUEUE_LENGTH, sizeof(queue_item_t));
    q_ble2uart = xQueueCreate(QUEUE_LENGTH, sizeof(queue_item_t));
    q_uart2uart = xQueueCreate(QUEUE_LENGTH, sizeof(queue_item_t));
    assert(q_uart2ble != NULL);
    assert(q_ble2uart != NULL); 
}

int sendData(const char* logName, const char* data, const int len)
{
    // const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}
int recvData(const char* logName, char* data, const int len)
{
    const int rxBytes = uart_read_bytes(UART_NUM_1, data, len, 10 / portTICK_PERIOD_MS);
    if (rxBytes > 0) {
        data[rxBytes] = 0;
        ESP_LOGI(logName, "Read %d bytes: '%s'", rxBytes, data);
        ESP_LOG_BUFFER_HEXDUMP(logName, data, rxBytes, ESP_LOG_INFO);
    }
    return rxBytes;
}

int32_t start_time;
int32_t end_time;
//uart_to_ble_task: 从队列取数据 -> 发送BLE通知
static void uart_to_ble_task(void *arg)
{
    queue_item_t uart2ble_item;
    int rc;
    struct os_mbuf *om;
    while(1) {
        uart2ble_item.len = recvData("Jeffery_uart_to_ble_task", (char*)uart2ble_item.data, ITEM_BUF_SIZE);
        if(uart2ble_item.len > 0) {
            bleWrTimeEnd = esp_log_timestamp();
            ESP_LOGI("Jeffery_uart_to_ble_task", "Received Data FROM UART Time %ld us", (int32_t)(bleWrTimeEnd - bleWrTimeStart)); 
            end_time = esp_log_timestamp();    
            ESP_LOGI("Jeffery_uart_to_ble_task", "Received Data Time %ld us", (int32_t)(end_time - start_time)); 
            if(notify_state_transparent == false) {
                ESP_LOGI("Jeffery", "Notify is not enabled");
                continue;
            }
            //检查连接句柄
            if(ble_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
                ESP_LOGE("Jeffery", "No Connection");
                continue;
            }
            //发送数据
            ESP_LOGI("Jeffery_UART_to_BLE", "Send %d bytes", uart2ble_item.len);
            om = ble_hs_mbuf_from_flat(uart2ble_item.data, uart2ble_item.len);
            rc = ble_gatts_notify_custom(ble_conn_handle, transparent_notify_handle, om);
            if (rc != 0) {
                ESP_LOGE("Jeffery_Error", "Error while sending notification; rc = %d", rc);
                /* Most probably error is because we ran out of mbufs (rc = 6),
                    * increase the mbuf count/size from menuconfig. Though
                    * inserting delay is not good solution let us keep it
                    * simple for time being so that the mbufs get freed up
                    * (?), of course assumption is we ran out of mbufs */
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            bleWrTimeEnd = esp_log_timestamp();
            ESP_LOGI("Jeffery_uart_to_ble_task", "SEND Data to BLE NOTIFY Time %ld us", (int32_t)(bleWrTimeEnd - bleWrTimeStart));
            ESP_LOGI("Jeffery_uart_to_ble_task", "bleWrTimeEnd Time %ld us", (int32_t)(bleWrTimeEnd));
        }
    }
}
//发送数据到UART
static void ble_to_uart_task(void *arg)
{
    queue_item_t ble2uart_item;
    while(1) {
        //等待队列中有数据
        if(xQueueReceive(q_ble2uart, &ble2uart_item, portMAX_DELAY) == pdTRUE) {
            bleWrTimeEnd = esp_log_timestamp();
            ESP_LOGI("Jeffery_BLE_to_UART", "Received Data FROM BLE Time %ld us", (int32_t)(bleWrTimeEnd - bleWrTimeStart)); 
            // //发送数据
            start_time = esp_log_timestamp(); 
            ESP_LOGI("Jeffery_BLE_to_UART", "Receive %d bytes", ble2uart_item.len);
            sendData("Jeffery_BLE_to_UART", (const char*)ble2uart_item.data, ble2uart_item.len);
        }
        // vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
void uart_task_init(void)
{
    uart_init();
    xTaskCreate(uart_to_ble_task, "uart_to_ble_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(ble_to_uart_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
}
