/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
// #include "common.h"
//test
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOSConfig.h"
/* BLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "gatts_sens.h"
#include "../src/ble_hs_hci_priv.h"

#include "gatt_svc.h"
#include "uart_ble_tasks.h"

#include "ble_ota.h"   // OTA 组件头文件
/* Library function declarations */
void ble_store_config_init(void);

#define NOTIFY_THROUGHPUT_PAYLOAD 500
#define MIN_REQUIRED_MBUF         2 /* Assuming payload of 500Bytes and each mbuf can take 292Bytes.  */
#define PREFERRED_MTU_VALUE       512
#define LL_PACKET_TIME            2120
#define LL_PACKET_LENGTH          251
#define MTU_DEF                   512

static const char *TAG = "ESP_BLE_OTA";
static const char *tag = "bleprph_throughput";
static const char *device_name = "copeland_ble";
static SemaphoreHandle_t notify_sem;
static bool notify_state;
static int notify_test_time = 60;
 uint16_t ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
 extern bool notify_state_transparent;
// static uint16_t conn_handle;
/* Dummy variable */
static uint8_t dummy;
static uint8_t gatts_addr_type;

static int gatts_gap_event(struct ble_gap_event *event, void *arg);

/**
 * Utility function to log an array of bytes.
 */

void
print_bytes(const uint8_t *bytes, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        ESP_LOGI(tag, "%s0x%02x", i != 0 ? ":" : "", bytes[i]);
    }
}

void
print_addr(const void *addr)
{
    const uint8_t *u8p;

    u8p = addr;
    ESP_LOGI(tag, "%02x:%02x:%02x:%02x:%02x:%02x",
             u8p[5], u8p[4], u8p[3], u8p[2], u8p[1], u8p[0]);
}

static void
bleprph_print_conn_desc(struct ble_gap_conn_desc *desc)
{
    ESP_LOGI(tag, "handle=%d our_ota_addr_type=%d our_ota_addr=",
                desc->conn_handle, desc->our_ota_addr.type);
    print_addr(desc->our_ota_addr.val);
    ESP_LOGI(tag, " our_id_addr_type=%d our_id_addr=",
                desc->our_id_addr.type);
    print_addr(desc->our_id_addr.val);
    ESP_LOGI(tag, " peer_ota_addr_type=%d peer_ota_addr=",
                desc->peer_ota_addr.type);
    print_addr(desc->peer_ota_addr.val);
    ESP_LOGI(tag, " peer_id_addr_type=%d peer_id_addr=",
                desc->peer_id_addr.type);
    print_addr(desc->peer_id_addr.val);
    ESP_LOGI(tag, " conn_itvl=%d conn_latency=%d supervision_timeout=%d "
                "encrypted=%d authenticated=%d bonded=%d",
                desc->conn_itvl, desc->conn_latency,
                desc->supervision_timeout,
                desc->sec_state.encrypted,
                desc->sec_state.authenticated,
                desc->sec_state.bonded);
}

/*
 * Enables advertising with parameters:
 *     o General discoverable mode
 *     o Undirected connectable mode
 */
static void
gatts_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    /*
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info)
     *     o Advertising tx power
     *     o Device name
     */
    memset(&fields, 0, sizeof(fields));

    /*
     * Advertise two flags:
     *      o Discoverability in forthcoming advertisement (general)
     *      o BLE-only (BR/EDR unsupported)
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /*
     * Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(tag, "Error setting advertisement data; rc=%d", rc);
        return;
    }

    /* Begin advertising */
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(gatts_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gatts_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(tag, "Error enabling advertisement; rc=%d", rc);
        return;
    }
}

/* This function sends notifications to the client */
static void
notify_task(void *arg)
{
    static uint8_t payload[NOTIFY_THROUGHPUT_PAYLOAD] = {0};/* Data payload */
    int rc, notify_count = 0;
    int64_t start_time, end_time, notify_time = 0;
    struct os_mbuf *om;

    payload[0] = dummy; /* storing dummy data */
    payload[1] = rand();
    payload[99] = rand();

    while (!notify_state) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    while (1) {
        switch (notify_test_time) {

        case 0:
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            break;
        default:
            start_time = esp_timer_get_time();

            if (!notify_state) {
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                break;
            }

            while (notify_time < (notify_test_time * 1000)) {
                /* We are anyway using counting semaphore for sending
                 * notifications. So hopefully not much waiting period will be
                 * introduced before sending a new notification. Revisit this
                 * counter if need to do away with semaphore waiting. XXX */
                xSemaphoreTake(notify_sem, portMAX_DELAY);

                if (dummy == 200) {
                    dummy = 0;
                }
                dummy++;

                /* Check if the MBUFs are available */
                if (os_msys_num_free() >= MIN_REQUIRED_MBUF) {
                    do {
                        om = ble_hs_mbuf_from_flat(payload, sizeof(payload));
                        if (om == NULL) {
                            /* Memory not available for mbuf */
                            ESP_LOGE(tag, "No MBUFs available from pool, retry..");
                            vTaskDelay(100 / portTICK_PERIOD_MS);
                        }
                    } while (om == NULL);

                    rc = ble_gatts_notify_custom(ble_conn_handle, notify_handle, om);
                    if (rc != 0) {
                        ESP_LOGE(tag, "Error while sending notification; rc = %d", rc);
                        notify_count -= 1;
                        xSemaphoreGive(notify_sem);
                        /* Most probably error is because we ran out of mbufs (rc = 6),
                         * increase the mbuf count/size from menuconfig. Though
                         * inserting delay is not good solution let us keep it
                         * simple for time being so that the mbufs get freed up
                         * (?), of course assumption is we ran out of mbufs */
                        vTaskDelay(10 / portTICK_PERIOD_MS);
                    }
                } else {
                    ESP_LOGE(tag, "Not enough OS_MBUFs available; reduce notify count ");
                    xSemaphoreGive(notify_sem);
                    notify_count -= 1;
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                }

                end_time = esp_timer_get_time();
                notify_time = (end_time - start_time) / 1000 ;
                notify_count += 1;
            }

            printf("\n*********************************\n");
            ESP_LOGI(tag, "Notify throughput = %d bps, count = %d",
                     (notify_count * NOTIFY_THROUGHPUT_PAYLOAD * 8) / notify_test_time, notify_count);
            printf("\n*********************************\n");
            ESP_LOGI(tag, " Notification test complete for stipulated time of %d sec", notify_test_time);
            notify_test_time = 0;
            notify_count = 0;

            break;
        }
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}
//jeffery 9/9/2025
uint8_t transparent_payload[20] = {0};
// SemaphoreHandle_t sem_transparent_notify;
SemaphoreHandle_t sem_notify_uart;
// bool notify_state_transparent = false;

static void
transparent_notify_task(void *arg){
    int rc;
    struct os_mbuf *om;
    while (1)
    {
        /* code */
        if(xSemaphoreTake(sem_transparent_notify, portMAX_DELAY) == true){
            ESP_LOGI("Jeffery", "Take Semaphore");  
            om = ble_hs_mbuf_from_flat(transparent_payload, sizeof(transparent_payload));
            rc = ble_gatts_notify_custom(ble_conn_handle, transparent_notify_handle, om);
            if (rc != 0) {
                ESP_LOGE("Jeffery_Error", "Error while sending notification; rc = %d", rc);
                // xSemaphoreGive(sem_transparent_notify);
                /* Most probably error is because we ran out of mbufs (rc = 6),
                    * increase the mbuf count/size from menuconfig. Though
                    * inserting delay is not good solution let us keep it
                    * simple for time being so that the mbufs get freed up
                    * (?), of course assumption is we ran out of mbufs */
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
extern int32_t bleWrTimeEnd;
//OTA
extern void esp_ble_ota_fill_handle_table(void);
extern esp_ble_ota_char_t find_ota_char_and_desr_by_handle(uint16_t handle);
extern esp_ble_ota_notification_check_t ota_notification;
extern uint16_t connection_handle;
static int
gatts_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        ESP_LOGI(TAG, "connection %s; status=%d ",
                 event->connect.status == 0 ? "established" : "failed",
                 event->connect.status);
        if (event->connect.status == 0) {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            //don't print conn desc here now
            // esp_ble_ota_print_conn_desc(&desc);
            connection_handle = event->connect.conn_handle;           
        } //link layout has performed advising, so do not restart advertising here
        //OTA
        esp_ble_ota_fill_handle_table();
        return 0;
    case BLE_GAP_EVENT_LINK_ESTAB:
        /* A new connection was established or a connection attempt failed */
        ESP_LOGI(tag, "connection %s; status = %d ",
                 event->link_estab.status == 0 ? "established" : "failed",
                 event->link_estab.status);
        rc = ble_att_set_preferred_mtu(PREFERRED_MTU_VALUE);
        if (rc != 0) {
            ESP_LOGE(tag, "Failed to set preferred MTU; rc = %d", rc);
        }

        if (event->link_estab.status != 0) {
            /* Connection failed; resume advertising */
            gatts_advertise();
        }       

        rc = ble_hs_hci_util_set_data_len(event->link_estab.conn_handle,
                                          LL_PACKET_LENGTH,
                                          LL_PACKET_TIME);
        if (rc != 0) {
            ESP_LOGE(tag, "Set packet length failed");
        }

        ble_conn_handle = event->link_estab.conn_handle;
        
        //jeffery add 9/19/2025
            printf("Link established, conn_handle=%d\n", event->link_estab.conn_handle);

            struct ble_gap_upd_params params = {0};
            params.itvl_min = 6;   // 7.5 ms
            params.itvl_max = 12;  // 15 ms
            params.latency  = 0;
            params.supervision_timeout = 400; // 4s

            int rc = ble_gap_update_params(event->link_estab.conn_handle, &params);
            if (rc != 0) {
                printf("ble_gap_update_params failed rc=%d\n", rc);
            } else {
                printf("Connection param update requested\n");
            }

        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(tag, "disconnect; reason = %d", event->disconnect.reason);
        ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        notify_state_transparent = false;
        /* Connection terminated; resume advertising */
        gatts_advertise();
        break;

    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        ESP_LOGI(tag, "connection updated; status=%d ",
                 event->conn_update.status);
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        assert(rc == 0);
        bleprph_print_conn_desc(&desc);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(tag, "adv complete ");
        gatts_advertise();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(tag, "subscribe event; cur_notify=%d; value handle; "
                 "val_handle = %d",
                 event->subscribe.cur_notify, event->subscribe.attr_handle);
        if (event->subscribe.attr_handle == notify_handle) {
            notify_state = event->subscribe.cur_notify;
            if (arg != NULL) {
                ESP_LOGI(tag, "notify test time = %d", *(int *)arg);
                notify_test_time = *((int *)arg);
            }
            xSemaphoreGive(notify_sem);
        } else if (event->subscribe.attr_handle != notify_handle) {
            // notify_state = event->subscribe.cur_notify;
        }
        if(event->subscribe.attr_handle == transparent_notify_handle) {
            ESP_LOGI("Jeffery", "cur_notify=%d; "
                 "val_handle = %d", event->subscribe.cur_notify, event->subscribe.attr_handle);
            if(event->subscribe.cur_notify == 1) {
                notify_state_transparent = true;
            } else {
                notify_state_transparent = false;
            }
            // notify_state = event->subscribe.cur_notify;
        } else {
            ESP_LOGI("Jeffery", "Notify Handle is not matched");
        }
        //OTA
        esp_ble_ota_char_t ota_char;
        ota_char = find_ota_char_and_desr_by_handle(event->subscribe.attr_handle);
        ESP_LOGI(TAG, "client subscribe ble_gap_event, ota_char: %d", ota_char);

        ESP_LOGI(TAG, "subscribe event; conn_handle=%d attr_handle=%d "
                 "reason=%d prevn=%d curn=%d previ=%d curi=%d\n",
                 event->subscribe.conn_handle,
                 event->subscribe.attr_handle,
                 event->subscribe.reason,
                 event->subscribe.prev_notify,
                 event->subscribe.cur_notify,
                 event->subscribe.prev_indicate,
                 event->subscribe.cur_indicate);

        switch (ota_char) {
        case RECV_FW_CHAR:
            ota_notification.recv_fw_ntf_enable = true;
            break;
        case OTA_STATUS_CHAR:
            ota_notification.process_bar_ntf_enable = true;
            break;
        case CMD_CHAR:
            ota_notification.command_ntf_enable = true;
            break;
        case CUS_CHAR:
            ota_notification.customer_ntf_enable = true;
            break;
        case INVALID_CHAR:
            break;
        }
        break;

    case BLE_GAP_EVENT_NOTIFY_TX:
        ESP_LOGD(tag, "BLE_GAP_EVENT_NOTIFY_TX success !!");
        if ((event->notify_tx.status == 0) ||
                (event->notify_tx.status == BLE_HS_EDONE)) {
            /* Send new notification i.e. give Semaphore. By definition,
             * sending new notifications should not be based on successful
             * notifications sent, but let us adopt this method to avoid too
             * many `BLE_HS_ENOMEM` errors because of continuous transfer of
             * notifications.XXX */
            xSemaphoreGive(notify_sem);
            //jeffery 8/27/2025
            ESP_LOGI("Jeffery", "BLE_GAP_EVENT_NOTIFY_TX notify tx status = %d", event->notify_tx.status);
            //jeffery 8/27/2025 End
            bleWrTimeEnd = esp_log_timestamp();
            ESP_LOGI("Jeffery", "BLE_GAP_EVENT_NOTIFY_TX Time %ld us", (int32_t)(bleWrTimeEnd));
        } else {
            ESP_LOGE(tag, "BLE_GAP_EVENT_NOTIFY_TX notify tx status = %d", event->notify_tx.status);
        }
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(tag, "mtu update event; conn_handle = %d mtu = %d ",
                 event->mtu.conn_handle,
                 event->mtu.value);
        break;
    }
    return 0;
}

static void
gatts_on_sync(void)
{
    int rc;
    uint8_t addr_val[6] = {0};

    rc = ble_hs_id_infer_auto(0, &gatts_addr_type);
    assert(rc == 0);
    rc = ble_hs_id_copy_addr(gatts_addr_type, addr_val, NULL);
    assert(rc == 0);
    ESP_LOGI(tag, "Device Address: ");
    print_addr(addr_val);
    /* Begin advertising */
    gatts_advertise();
}

static void
gatts_on_reset(int reason)
{
    ESP_LOGE(tag, "Resetting state; reason=%d", reason);
}

void gatts_host_task(void *param)
{
    ESP_LOGI(tag, "BLE Host Task Started");
    /* Create a counting semaphore for Notification. Can be used to track
     * successful notification txmission. Optimistically take some big number
     * for counting Semaphore */
    notify_sem = xSemaphoreCreateCounting(100, 0);
    // sem_transparent_notify = xSemaphoreCreateBinary();
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();
    vSemaphoreDelete(notify_sem);
    nimble_port_freertos_deinit();
}
// OTA 相关代码
// static const char *TAG = "ESP_BLE_OTA";
#include "freertos/ringbuf.h"
#include "esp_ota_ops.h"
#include "ble_ota.h"
#define OTA_RINGBUF_SIZE                    8192
#define OTA_TASK_SIZE                       8192
static esp_ota_handle_t out_handle;
SemaphoreHandle_t notify_sem_ota;
static RingbufHandle_t s_ringbuf = NULL;

bool
ble_ota_ringbuf_init(uint32_t ringbuf_size)
{
    s_ringbuf = xRingbufferCreate(ringbuf_size, RINGBUF_TYPE_BYTEBUF);
    if (s_ringbuf == NULL) {
        return false;
    }

    return true;
}
size_t
write_to_ringbuf(const uint8_t *data, size_t size)
{
    BaseType_t done = xRingbufferSend(s_ringbuf, (void *)data, size, (TickType_t)portMAX_DELAY);
    if (done) {
        return size;
    } else {
        return 0;
    }
}
void
ota_task(void *arg)
{
    esp_partition_t *partition_ptr = NULL;
    esp_partition_t partition;
    const esp_partition_t *next_partition = NULL;

    uint32_t recv_len = 0;
    uint8_t *data = NULL;
    size_t item_size = 0;
    ESP_LOGI(TAG, "ota_task start");

    notify_sem_ota = xSemaphoreCreateCounting(100, 0);
    xSemaphoreGive(notify_sem_ota);

#ifdef CONFIG_EXAMPLE_USE_DELTA_OTA
    esp_err_t err;
    esp_delta_ota_cfg_t cfg = {
        .read_cb = &read_cb,
    };

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0))
    char *user_data = "ble_delta_ota";
    cfg.write_cb_with_user_data = &write_cb;
    cfg.user_data = user_data;
#else
    cfg.write_cb = &write_cb;
#endif

    const esp_partition_t *destination_partition;

    esp_delta_ota_handle_t handle = esp_delta_ota_init(&cfg);
    if (handle == NULL) {
        ESP_LOGE(TAG, "delta_ota_set_cfg failed!");
        goto OTA_ERROR;
    }
    /* search ota partition */
    current_partition = esp_ota_get_running_partition();
    destination_partition = esp_ota_get_next_update_partition(NULL);

    if (current_partition == NULL || destination_partition == NULL) {
        ESP_LOGE(TAG, "Error getting partition information");
        goto OTA_ERROR;
    }
    if (current_partition->subtype >= ESP_PARTITION_SUBTYPE_APP_OTA_MAX ||
            destination_partition->subtype >= ESP_PARTITION_SUBTYPE_APP_OTA_MAX) {
        goto OTA_ERROR;
    }
    err = esp_ota_begin(destination_partition, OTA_SIZE_UNKNOWN, &(out_handle));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        goto OTA_ERROR;
    }
#else
    partition_ptr = (esp_partition_t *)esp_ota_get_boot_partition();
    if (partition_ptr == NULL) {
        ESP_LOGE(TAG, "boot partition NULL!\r\n");
        goto OTA_ERROR;
    }
    if (partition_ptr->type != ESP_PARTITION_TYPE_APP) {
        ESP_LOGE(TAG, "esp_current_partition->type != ESP_PARTITION_TYPE_APP\r\n");
        goto OTA_ERROR;
    }

    if (partition_ptr->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY) {
        partition.subtype = ESP_PARTITION_SUBTYPE_APP_OTA_0;
    } else {
        next_partition = esp_ota_get_next_update_partition(partition_ptr);
        if (next_partition) {
            partition.subtype = next_partition->subtype;
        } else {
            partition.subtype = ESP_PARTITION_SUBTYPE_APP_OTA_0;
        }
    }
    partition.type = ESP_PARTITION_TYPE_APP;

    partition_ptr = (esp_partition_t *)esp_partition_find_first(partition.type, partition.subtype, NULL);
    if (partition_ptr == NULL) {
        ESP_LOGE(TAG, "partition NULL!\r\n");
        goto OTA_ERROR;
    }

    memcpy(&partition, partition_ptr, sizeof(esp_partition_t));
    if (esp_ota_begin(&partition, OTA_SIZE_UNKNOWN, &out_handle) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed!\r\n");
        goto OTA_ERROR;
    }
#endif
    ESP_LOGI(TAG, "wait for data from ringbuf! fw_len = %u", esp_ble_ota_get_fw_length());
    /*deal with all receive packet*/
    for (;;) {
        data = (uint8_t *)xRingbufferReceive(s_ringbuf, &item_size, (TickType_t)portMAX_DELAY);

        xSemaphoreTake(notify_sem_ota, portMAX_DELAY);

#ifdef CONFIG_EXAMPLE_USE_DELTA_OTA
        static int flag = 0;
        static char ota_write_data[65] = { 0 };

        /*deal with receive patch header*/
        if (flag == 0) {

            flag = 1;

            /* Read size equal to patch header to verify the header*/
            memcpy(ota_write_data, data, PATCH_HEADER_SIZE);
            if (!verify_patch_header(ota_write_data)) {
                ESP_LOGE(TAG, "Patch Header verification failed!");
                goto OTA_ERROR;
            }

            data += 64;
            item_size -= 64;
            recv_len += 64;
        }
#endif
        ESP_LOGI(TAG, "recv: %u, recv_total:%"PRIu32"\n", item_size, recv_len + item_size);

        if (item_size != 0) {
#ifdef CONFIG_EXAMPLE_USE_DELTA_OTA
            if (esp_delta_ota_feed_patch(handle, (const uint8_t *)data, item_size) < 0) {
                ESP_LOGE(TAG, "Error while applying patch");
                goto OTA_ERROR;
            }
#else
            if (esp_ota_write(out_handle, (const void *)data, item_size) != ESP_OK) {
                ESP_LOGE(TAG, "esp_ota_write failed!\r\n");
                goto OTA_ERROR;
            }
#endif
            recv_len += item_size;
            vRingbufferReturnItem(s_ringbuf, (void *)data);

            if (recv_len >= esp_ble_ota_get_fw_length()) {
                xSemaphoreGive(notify_sem_ota);
                break;
            }
        }
        xSemaphoreGive(notify_sem_ota);
    }

#ifdef CONFIG_EXAMPLE_USE_DELTA_OTA
    err = esp_delta_ota_finalize(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_delta_ota_finalize() failed : %s", esp_err_to_name(err));
        goto OTA_ERROR;
    }

    err = esp_delta_ota_deinit(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_delta_ota_deinit() failed : %s", esp_err_to_name(err));
        goto OTA_ERROR;
    }
#endif

    if (esp_ota_end(out_handle) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed!\r\n");
        goto OTA_ERROR;
    }

#ifdef CONFIG_EXAMPLE_USE_DELTA_OTA
    if (esp_ota_set_boot_partition(destination_partition) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed!\r\n");
        goto OTA_ERROR;
    }
#else
    if (esp_ota_set_boot_partition(&partition) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed!\r\n");
        goto OTA_ERROR;
    }
#endif

    vSemaphoreDelete(notify_sem_ota);
    esp_restart();

OTA_ERROR:
    ESP_LOGE(TAG, "OTA failed");
    vTaskDelete(NULL);
}
void
ota_recv_fw_cb(uint8_t *buf, uint32_t length)
{
    write_to_ringbuf(buf, length);
}

static void
ota_task_init(void)
{
    xTaskCreate(&ota_task, "ota_task", OTA_TASK_SIZE, NULL, 5, NULL);
    return;
}


void app_main(void)
{
    int rc;

    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (!ble_ota_ringbuf_init(OTA_RINGBUF_SIZE)) {
        ESP_LOGE(TAG, "%s init ringbuf fail", __func__);
        return;
    }

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "Failed to init nimble %d ", ret);
        return;
    }

    /* Initialize the NimBLE host configuration */
    ble_hs_cfg.sync_cb = gatts_on_sync;
    ble_hs_cfg.reset_cb = gatts_on_reset;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

//     ble_hs_cfg.sm_io_cap = 3; /* NoInputNoOutput */
// #ifdef CONFIG_EXAMPLE_BONDING
//     ble_hs_cfg.sm_bonding = 1;
// #endif
// #ifdef CONFIG_EXAMPLE_MITM
//     ble_hs_cfg.sm_mitm = 1;
// #endif
// #ifdef CONFIG_EXAMPLE_USE_SC
//     ble_hs_cfg.sm_sc = 1;
// #else
//     ble_hs_cfg.sm_sc = 0;
// #endif
// #ifdef CONFIG_EXAMPLE_BONDING
//     ble_hs_cfg.sm_our_key_dist = 1;
//     ble_hs_cfg.sm_their_key_dist = 1;
// #endif

    /* Store host configuration */
    ble_store_config_init();

    /* Initialize Notify Task */
    xTaskCreate(notify_task, "notify_task", 4096, NULL, 10, NULL);
    sem_notify_uart = xSemaphoreCreateBinary();
    sem_transparent_notify = xSemaphoreCreateBinary();
    // xTaskCreate(transparent_notify_task, "transparent_task", 4096, NULL, 10, NULL);

    rc = gatt_svr_init();
    assert(rc == 0);

    /* Set the default device name */
    rc = ble_svc_gap_device_name_set(device_name);
    assert(rc == 0);

    //jeffery 9/12/2025
    uart_task_init();
    //jeffery 9/12/2025 End
    /* Start the task */
    nimble_port_freertos_init(gatts_host_task);

#if CONFIG_EXAMPLE_USE_PRE_ENC_OTA
    esp_decrypt_cfg_t cfg = {};
    cfg.rsa_pub_key = rsa_private_pem_start;
    cfg.rsa_pub_key_len = rsa_private_pem_end - rsa_private_pem_start;
    decrypt_handle = esp_encrypted_img_decrypt_start(&cfg);
    if (!decrypt_handle) {
        ESP_LOGE(TAG, "OTA upgrade failed");
        vTaskDelete(NULL);
    }

    esp_ble_ota_recv_fw_data_callback(ota_recv_fw_cb, decrypt_handle);
#else
    esp_ble_ota_recv_fw_data_callback(ota_recv_fw_cb);
#endif /* CONFIG_EXAMPLE_USE_PRE_ENC_OTA */
    ota_task_init();
}
