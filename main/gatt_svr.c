/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "gatts_sens.h"
#include "esp_log.h"
#include "uart_ble_tasks.h"
#include "gatt_svc.h"

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// #include "nimble/nimble_port.h"
// #include "nimble/nimble_port_freertos.h"
// #include "freertos/FreeRTOSConfig.h"
// // #include "host/ble_hs.h"
// #include "host/util/util.h"
// #include "console/console.h"
// // #include "services/gap/ble_svc_gap.h"
// // #include "gatts_sens.h"
// #include "../src/ble_hs_hci_priv.h"

/* jeffery add */
/* Device Information Service*/
static const ble_uuid16_t dev_info_svc_uuid = BLE_UUID16_INIT(0x180A);

static const ble_uuid16_t manufacturer_name_chr_uuid = BLE_UUID16_INIT(0x2A29);
static uint16_t manufacturer_name_chr_val_handle;
static const char *manufacturer_name = "Copeland_ESP";
static const ble_uuid16_t model_number_chr_uuid = BLE_UUID16_INIT(0x2A24);
static uint16_t model_number_chr_val_handle;
static const char *model_number = "ESP32C2_8684_MINI_1";
static const ble_uuid16_t serial_number_chr_uuid = BLE_UUID16_INIT(0x2A25);
static uint16_t serial_number_chr_val_handle;
static const char *serial_number = "SN_0001";
static const ble_uuid16_t hardware_rev_chr_uuid = BLE_UUID16_INIT(0x2A27);
static uint16_t hardware_rev_chr_val_handle;
static const char *hardware_rev = "Rev_1.0";
static const ble_uuid16_t firmware_rev_chr_uuid = BLE_UUID16_INIT(0x2A26);
static uint16_t firmware_rev_chr_val_handle;
static const char *firmware_rev = "FW_1.0";
static const ble_uuid16_t software_rev_chr_uuid = BLE_UUID16_INIT(0x2A28);
static uint16_t software_rev_chr_val_handle;
static const char *software_rev = "SW_1.0";
// jeffery add
static int device_information_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);

/* Transparent Transmission */
/* Service Declaration*/
/* 49535343-fe7d-4ae5-8fa9-9fafd205e455 */
#define TRANSP_SVC_UUID_DECLARE()                         \
    ((const ble_uuid_t *) (&(ble_uuid128_t) BLE_UUID128_INIT(   \
    0x55, 0xe4, 0x05, 0xd2, 0xfa, 0x9f, 0xa8, 0x8f,             \
    0xe5, 0x4a, 0x7d, 0xfe, 0x43, 0x53, 0x53, 0x49              \
    )))
/* Characteristic Declaration for write, notify and indicate*/
/* 49535343-1e4d-4bd9-ba61-23c647249616 */
#define TRANSP_CHR_WNI_UUID_DECLARE()                          \
    ((const ble_uuid_t *) (&(ble_uuid128_t) BLE_UUID128_INIT(   \
    0x16, 0x96, 0x24, 0x47, 0xc6, 0x23, 0x61, 0xba,             \
    0xd9, 0x4b, 0x4d, 0x1e, 0x43, 0x53, 0x53, 0x49              \
    )))
/* Characteristic Declaration for only write*/
/* 49535343-8841-43f4-a8d4-ecbe34729bb3 */
#define TRANSP_CHR_W_UUID_DECLARE()                      \
    ((const ble_uuid_t *) (&(ble_uuid128_t) BLE_UUID128_INIT(   \
    0xb3, 0x9b, 0x72, 0x34, 0xbe, 0xec, 0xd4, 0xa8,             \
    0xf4, 0x43, 0x41, 0x88, 0x43, 0x53, 0x53, 0x49              \
    )))
/* Characteristic Declaration for write and notify*/
/* 49535343-438a-39b3-2f49-511cff073be7 */
#define TRANSP_CHR_WN_UUID_DECLARE()                     \
    ((const ble_uuid_t *) (&(ble_uuid128_t) BLE_UUID128_INIT(   \
    0xe7, 0x3b, 0x07, 0xcf, 0x1c, 0x51, 0x49, 0x2f,             \
    0xb3, 0x39, 0x8a, 0x43, 0x43, 0x53, 0x53, 0x49              \
    )))


/* 0000xxxx-8c26-476f-89a7-a108033a69c7 */
#define THRPT_UUID_DECLARE(uuid16)                                \
    ((const ble_uuid_t *) (&(ble_uuid128_t) BLE_UUID128_INIT(   \
    0xc7, 0x69, 0x3a, 0x03, 0x08, 0xa1, 0xa7, 0x89,             \
    0x6f, 0x47, 0x26, 0x8c, uuid16, uuid16 >> 8, 0x00, 0x00     \
    )))

/* 0000xxxx-8c26-476f-89a7-a108033a69c6 */
#define THRPT_UUID_DECLARE_ALT(uuid16)                            \
    ((const ble_uuid_t *) (&(ble_uuid128_t) BLE_UUID128_INIT(   \
    0xc6, 0x69, 0x3a, 0x03, 0x08, 0xa1, 0xa7, 0x89,             \
    0x6f, 0x47, 0x26, 0x8c, uuid16, uuid16 >> 8, 0x00, 0x00     \
    )))
static int transparent_chr_write_notify_indicate(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);
static int transparent_chr_write(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg);
static int transparent_chr_write_indicate(uint16_t conn_handle, uint16_t attr_handle,
                                        struct ble_gatt_access_ctxt *ctxt, void *arg);
uint16_t transparent_notify_handle;

//OTA Service
#include "ble_ota.h"
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
#include "esp_nimble_hci.h"

/*
 * This is a workaround for the missing os_mbuf_len function in NimBLE.
 * It is not present in NimBLE 1.3, but is present in NimBLE 1.4.
 * This function is used to get the length of an os_mbuf.
 */
uint16_t
os_mbuf_len(const struct os_mbuf *om)
{
    uint16_t len;

    len = 0;
    while (om != NULL) {
        len += om->om_len;
        om = SLIST_NEXT(om, om_next);
    }

    return len;
}
#endif

#ifdef CONFIG_PRE_ENC_OTA
#define SECTOR_END_ID                       2
#define ENC_HEADER                          512
esp_decrypt_handle_t decrypt_handle_cmp;
#endif

#define BUF_LENGTH                          4098
#define OTA_IDX_NB                          4
#define CMD_ACK_LENGTH                      20

#define character_declaration_uuid          BLE_ATT_UUID_CHARACTERISTIC
#define character_client_config_uuid        BLE_ATT_UUID_CHARACTERISTIC

#define GATT_SVR_SVC_ALERT_UUID             0x1811
#define BLE_OTA_SERVICE_UUID                0x8018

#define RECV_FW_UUID                        0x8020
#define OTA_BAR_UUID                        0x8021
#define COMMAND_UUID                        0x8022
#define CUSTOMER_UUID                       0x8023

#if CONFIG_EXAMPLE_EXTENDED_ADV
static uint8_t ext_adv_pattern[] = {
    0x02, 0x01, 0x06,
    0x03, 0x03, 0xab, 0xcd,
    0x03, 0x03, 0x18, 0x11,
    0x0f, 0X09, 'n', 'i', 'm', 'b', 'l', 'e', '-', 'o', 't', 'a', '-', 'e', 'x', 't',
};
#endif

static bool counter = false;
static const char *TAG = "NimBLE_BLE_OTA";

static bool start_ota = false;
static uint32_t cur_sector = 0;
static uint32_t cur_packet = 0;
static uint8_t *fw_buf = NULL;
static uint32_t fw_buf_offset = 0;

static uint32_t ota_total_len = 0;
static uint32_t ota_block_size = BUF_LENGTH;

esp_ble_ota_callback_funs_t ota_cb_fun_t = {
    .recv_fw_cb = NULL
};

#ifndef CONFIG_OTA_WITH_PROTOCOMM
esp_ble_ota_notification_check_t ota_notification = {
    .recv_fw_ntf_enable = false,
    .process_bar_ntf_enable = false,
    .command_ntf_enable = false,
    .customer_ntf_enable = false,
};

static uint8_t own_addr_type;

uint16_t connection_handle;
static uint16_t attribute_handle;

static uint16_t ota_handle_table[OTA_IDX_NB];

static uint16_t receive_fw_val;
static uint16_t ota_status_val;
static uint16_t command_val;
static uint16_t custom_val;

static int
esp_ble_ota_gap_event(struct ble_gap_event *event, void *arg);
// static esp_ble_ota_char_t
// find_ota_char_and_desr_by_handle(uint16_t handle);
static int esp_ble_ota_notification_data(uint16_t conn_handle, uint16_t attr_handle, uint8_t cmd_ack[],
                              esp_ble_ota_char_t ota_char);
#endif
//OTA Handler
static uint16_t
crc16_ccitt(const unsigned char *buf, int len);
static esp_err_t
esp_ble_ota_recv_fw_handler(uint8_t *buf, uint32_t length);
static void
esp_ble_ota_write_chr(struct os_mbuf *om);
void esp_ble_ota_set_fw_length(uint32_t length);
unsigned int esp_ble_ota_get_fw_length(void);
static int
ble_ota_gatt_handler(uint16_t conn_handle, uint16_t attr_handle,
                     struct ble_gatt_access_ctxt *ctxt,
                     void *arg);

static void
esp_ble_ota_bar_resp_get(void)
{
    #ifndef CONFIG_OTA_WITH_PROTOCOMM
    esp_ble_ota_char_t ota_char = find_ota_char_and_desr_by_handle(attribute_handle);
    #endif
    uint8_t cmd_ack[CMD_ACK_LENGTH] = {0x03, 0x00, 0x00, 0x00, 0x00,
                                       0x00, 0x00, 0x00, 0x00, 0x00,
                                       0x00, 0x00, 0x00, 0x00, 0x00,
                                       0x00, 0x00, 0x00, 0x00, 0x00
                                      };
    uint16_t crc16;

    // if ((om->om_data[0] + (om->om_data[1] * 256)) != cur_sector) {
    //     // sector error
    //     if ((om->om_data[0] == 0xff) && (om->om_data[1] == 0xff)) {
    //         // last sector
    //         ESP_LOGD(TAG, "Last sector");
    //     } else {
            // sector error
            // ESP_LOGE(TAG, "%s - sector index error, cur: %" PRIu32 ", recv: %d", __func__,
            //          cur_sector, (om->om_data[0] + (om->om_data[1] * 256)));
            // cmd_ack[0] = om->om_data[0];
            // cmd_ack[1] = om->om_data[1];
            // cmd_ack[2] = 0x02; //sector index error
            // cmd_ack[3] = 0x00;
            // cmd_ack[4] = cur_sector & 0xff;
            // cmd_ack[5] = (cur_sector & 0xff00) >> 8;
            // crc16 = crc16_ccitt(cmd_ack, 18);
            // cmd_ack[18] = crc16 & 0xff;
            // cmd_ack[19] = (crc16 & 0xff00) >> 8;
            cmd_ack[0] = 0xFF;
            ESP_LOGI("Jeffery_OTA_BAR",
             "%02X", cmd_ack[0]);
#ifndef CONFIG_OTA_WITH_PROTOCOMM
            esp_ble_ota_notification_data(connection_handle, attribute_handle, cmd_ack, ota_char);
#endif
    //     }
    // }
}

static void
esp_ble_ota_write_chr(struct os_mbuf *om)
{
#ifndef CONFIG_OTA_WITH_PROTOCOMM
    esp_ble_ota_char_t ota_char = find_ota_char_and_desr_by_handle(attribute_handle);
#endif

#ifdef CONFIG_PRE_ENC_OTA
    esp_err_t err;
    pre_enc_decrypt_arg_t pargs = {};

    pargs.data_in_len = os_mbuf_len(om) - 3;

    pargs.data_in = (const char *)malloc(pargs.data_in_len);
    err = os_mbuf_copydata(om, 3, pargs.data_in_len, pargs.data_in);

    if (om->om_data[2] == 0xff) {
        pargs.data_in_len -= SECTOR_END_ID;
    }

    err = esp_encrypted_img_decrypt_data(decrypt_handle_cmp, &pargs);
    if (err != ESP_OK && err != ESP_ERR_NOT_FINISHED) {
        return;
    }
#endif

    uint8_t cmd_ack[CMD_ACK_LENGTH] = {0x03, 0x00, 0x00, 0x00, 0x00,
                                       0x00, 0x00, 0x00, 0x00, 0x00,
                                       0x00, 0x00, 0x00, 0x00, 0x00,
                                       0x00, 0x00, 0x00, 0x00, 0x00
                                      };
    uint16_t crc16;

    if ((om->om_data[0] + (om->om_data[1] * 256)) != cur_sector) {
        // sector error
        if ((om->om_data[0] == 0xff) && (om->om_data[1] == 0xff)) {
            // last sector
            ESP_LOGD(TAG, "Last sector");
        } else {
            // sector error
            ESP_LOGE(TAG, "%s - sector index error, cur: %" PRIu32 ", recv: %d", __func__,
                     cur_sector, (om->om_data[0] + (om->om_data[1] * 256)));
            cmd_ack[0] = om->om_data[0];
            cmd_ack[1] = om->om_data[1];
            cmd_ack[2] = 0x02; //sector index error
            cmd_ack[3] = 0x00;
            cmd_ack[4] = cur_sector & 0xff;
            cmd_ack[5] = (cur_sector & 0xff00) >> 8;
            crc16 = crc16_ccitt(cmd_ack, 18);
            cmd_ack[18] = crc16 & 0xff;
            cmd_ack[19] = (crc16 & 0xff00) >> 8;
#ifndef CONFIG_OTA_WITH_PROTOCOMM
            esp_ble_ota_notification_data(connection_handle, attribute_handle, cmd_ack, ota_char);
#endif
        }
    }

    if (om->om_data[2] != cur_packet) { // packet seq error
        if (om->om_data[2] == 0xff) { // last packet
            ESP_LOGD(TAG, "last packet");
            goto write_ota_data;
        } else { // packet seq error
            ESP_LOGE(TAG, "%s - packet index error, cur: %" PRIu32 ", recv: %d", __func__,
                     cur_packet, om->om_data[2]);
        }
    }

write_ota_data:
#ifdef CONFIG_PRE_ENC_OTA
    if (pargs.data_out_len > 0) {
        memcpy(fw_buf + fw_buf_offset, pargs.data_out, pargs.data_out_len);

        free(pargs.data_out);
        free(pargs.data_in);

        fw_buf_offset += pargs.data_out_len;
    }
    ESP_LOGD(TAG, "DEBUG: Sector:%" PRIu32 ", total length:%" PRIu32 ", length:%d", cur_sector,
             fw_buf_offset, pargs.data_out_len);
#else
    os_mbuf_copydata(om, 3, os_mbuf_len(om) - 3, fw_buf + fw_buf_offset);
    fw_buf_offset += os_mbuf_len(om) - 3;

    ESP_LOGD(TAG, "DEBUG: Sector:%" PRIu32 ", total length:%" PRIu32 ", length:%d", cur_sector,
             fw_buf_offset, os_mbuf_len(om) - 3);
#endif
    if (om->om_data[2] == 0xff) {
        cur_packet = 0;
        cur_sector++;
        ESP_LOGD(TAG, "DEBUG: recv %" PRIu32 " sector", cur_sector);
        goto sector_end;
    } else {
        ESP_LOGD(TAG, "DEBUG: wait next packet");
        cur_packet++;
    }
    return;

sector_end:
    if (fw_buf_offset < ota_block_size) {
        esp_ble_ota_recv_fw_handler(fw_buf, fw_buf_offset);
    } else {
        esp_ble_ota_recv_fw_handler(fw_buf, 4096);
    }

    fw_buf_offset = 0;
    memset(fw_buf, 0x0, ota_block_size);

    cmd_ack[0] = om->om_data[0];
    cmd_ack[1] = om->om_data[1];
    cmd_ack[2] = 0x00; //success
    cmd_ack[3] = 0x00;
    crc16 = crc16_ccitt(cmd_ack, 18);
    cmd_ack[18] = crc16 & 0xff;
    cmd_ack[19] = (crc16 & 0xff00) >> 8;
    counter = true;
#ifndef CONFIG_OTA_WITH_PROTOCOMM
    esp_ble_ota_notification_data(connection_handle, attribute_handle, cmd_ack, ota_char);
#endif
}

static uint16_t crc16_ccitt(const unsigned char *buf, int len)
{
    uint16_t crc16 = 0;
    int32_t i;

    while (len--) {
        crc16 ^= *buf++ << 8;

        for (i = 0; i < 8; i++) {
            if (crc16 & 0x8000) {
                crc16 = (crc16 << 1) ^ 0x1021;
            } else {
                crc16 = crc16 << 1;
            }
        }
    }

    return crc16;
}

void esp_ble_ota_set_fw_length(uint32_t length)
{
    ota_total_len = length;
}

unsigned int esp_ble_ota_get_fw_length(void)
{
    return ota_total_len;
}

static esp_err_t
esp_ble_ota_recv_fw_handler(uint8_t *buf, uint32_t length)
{
    if (ota_cb_fun_t.recv_fw_cb) {
        ota_cb_fun_t.recv_fw_cb(buf, length);
    }

    return ESP_OK;
}

#ifndef CONFIG_PRE_ENC_OTA
esp_err_t esp_ble_ota_recv_fw_data_callback(esp_ble_ota_recv_fw_cb_t callback)
{
    ota_cb_fun_t.recv_fw_cb = callback;
    return ESP_OK;
}
#else
esp_err_t esp_ble_ota_recv_fw_data_callback(esp_ble_ota_recv_fw_cb_t callback,
                                            esp_decrypt_handle_t esp_decrypt_handle)
{
    decrypt_handle_cmp = esp_decrypt_handle;
    ota_cb_fun_t.recv_fw_cb = callback;
    return ESP_OK;
}
#endif

/*----------------------------------------------------
 * Protocomm specific api's
 *----------------------------------------------------*/

#ifdef CONFIG_OTA_WITH_PROTOCOMM
void
esp_ble_ota_set_sizes(size_t file_size, size_t block_size)
{
    ota_total_len = file_size;
    ota_block_size = block_size;
}

void
esp_ble_ota_finish(void)
{
    ESP_LOGI(TAG, "Received OTA end command");
    start_ota = false;
    ota_total_len = 0;
    ota_block_size = 0;
    free(fw_buf);
    fw_buf = NULL;
}

void
esp_ble_ota_write(uint8_t *file, size_t length)
{
    struct os_mbuf *om = ble_hs_mbuf_from_flat(file, length);
    esp_ble_ota_write_chr(om);
}
#else

/*----------------------------------------------------
 * OTA without protocomm api's
 *----------------------------------------------------*/

static void
ble_ota_start_write_chr(struct os_mbuf *om)
{
    uint8_t cmd_ack[CMD_ACK_LENGTH] = {0x03, 0x00, 0x00, 0x00, 0x00,
                                       0x00, 0x00, 0x00, 0x00, 0x00,
                                       0x00, 0x00, 0x00, 0x00, 0x00,
                                       0x00, 0x00, 0x00, 0x00, 0x00
                                      };
    uint16_t crc16;

    esp_ble_ota_char_t ota_char = find_ota_char_and_desr_by_handle(attribute_handle);
    if ((om->om_data[0] == 0x01) && (om->om_data[1] == 0x00)) {
        start_ota = true;

        ota_total_len = (om->om_data[2]) + (om->om_data[3] * 256) +
                        (om->om_data[4] * 256 * 256) + (om->om_data[5] * 256 * 256 * 256);

#ifdef CONFIG_PRE_ENC_OTA
        ota_total_len -= ENC_HEADER;
#endif
        ESP_LOGI(TAG, "recv ota start cmd, fw_length = %" PRIu32 "", ota_total_len);

        fw_buf = (uint8_t *)malloc(ota_block_size * sizeof(uint8_t));
        if (fw_buf == NULL) {
            ESP_LOGE(TAG, "%s -  malloc fail", __func__);
        } else {
            memset(fw_buf, 0x0, ota_block_size);
        }
        cmd_ack[2] = 0x01;
        cmd_ack[3] = 0x00;
        crc16 = crc16_ccitt(cmd_ack, 18);
        cmd_ack[18] = crc16 & 0xff;
        cmd_ack[19] = (crc16 & 0xff00) >> 8;
        esp_ble_ota_notification_data(connection_handle, attribute_handle, cmd_ack, ota_char);
    } else if ((om->om_data[0] == 0x02) && (om->om_data[1] == 0x00)) {
        printf("\nCMD_CHAR -> 0 : %d, 1 : %d", om->om_data[0],
               om->om_data[1]);
#ifdef CONFIG_PRE_ENC_OTA
        if (esp_encrypted_img_decrypt_end(decrypt_handle_cmp) != ESP_OK) {
            ESP_LOGI(TAG, "Decryption end failed");
        }
#endif
        extern SemaphoreHandle_t notify_sem_ota;
        xSemaphoreTake(notify_sem_ota, portMAX_DELAY);

        start_ota = false;
        ota_total_len = 0;

        xSemaphoreGive(notify_sem_ota);

        ESP_LOGD(TAG, "recv ota stop cmd");
        cmd_ack[2] = 0x02;
        cmd_ack[3] = 0x00;
        crc16 = crc16_ccitt(cmd_ack, 18);
        cmd_ack[18] = crc16 & 0xff;
        cmd_ack[19] = (crc16 & 0xff00) >> 8;
        esp_ble_ota_notification_data(connection_handle, attribute_handle, cmd_ack, ota_char);
        free(fw_buf);
        fw_buf = NULL;
    }
}

static int
ble_ota_gatt_handler(uint16_t conn_handle, uint16_t attr_handle,
                     struct ble_gatt_access_ctxt *ctxt,
                     void *arg)
{
    esp_ble_ota_char_t ota_char;

    attribute_handle = attr_handle;

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        ota_char = find_ota_char_and_desr_by_handle(attr_handle);
        ESP_LOGI(TAG, "client read, ota_char: %d", ota_char);
        if(ota_char == OTA_STATUS_CHAR) {
            esp_ble_ota_bar_resp_get();
        }
        break;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:

        ota_char = find_ota_char_and_desr_by_handle(attr_handle);
        ESP_LOGD(TAG, "client write; len = %d", os_mbuf_len(ctxt->om));

        if (ota_char == RECV_FW_CHAR) {
            if (start_ota) {
                esp_ble_ota_write_chr(ctxt->om);

            } else {
                ESP_LOGE(TAG, "%s -  don't receive the start cmd", __func__);
            }
        } else if (ota_char == CMD_CHAR) {
            ble_ota_start_write_chr(ctxt->om);
        }
        break;

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
    return 0;
}

esp_ble_ota_char_t
find_ota_char_and_desr_by_handle(uint16_t handle)
{
    esp_ble_ota_char_t ret = INVALID_CHAR;

    for (int i = 0; i < OTA_IDX_NB ; i++) {
        if (handle == ota_handle_table[i]) {
            switch (i) {
            case RECV_FW_CHAR_VAL_IDX:
                ret = RECV_FW_CHAR;
                break;
            case OTA_STATUS_CHAR_VAL_IDX:
                ret = OTA_STATUS_CHAR;
                break;
            case CMD_CHAR_VAL_IDX:
                ret = CMD_CHAR;
                break;
            case CUS_CHAR_VAL_IDX:
                ret = CUS_CHAR;
                break;
            default:
                ret = INVALID_CHAR;
                break;
            }
        }
    }
    return ret;
}

static int
esp_ble_ota_notification_data(uint16_t conn_handle, uint16_t attr_handle, uint8_t cmd_ack[],
                              esp_ble_ota_char_t ota_char)
{
    struct os_mbuf *txom;
    bool notify_enable = false;
    int rc;
    txom = ble_hs_mbuf_from_flat(cmd_ack, CMD_ACK_LENGTH);

    switch (ota_char) {
    case RECV_FW_CHAR:
        if (ota_notification.recv_fw_ntf_enable) {
            notify_enable = true;
        }
        break;
    case OTA_STATUS_CHAR:
        if (ota_notification.process_bar_ntf_enable) {
            notify_enable = true;
            txom = ble_hs_mbuf_from_flat(cmd_ack, 1);
        }
        break;
    case CMD_CHAR:
        if (ota_notification.command_ntf_enable) {
            notify_enable = true;
        }
        break;
    case CUS_CHAR:
        if (ota_notification.customer_ntf_enable) {
            notify_enable = true;
        }
        break;
    case INVALID_CHAR:
        break;
    }

    if (notify_enable) {
        rc = ble_gattc_notify_custom(conn_handle, attr_handle, txom);
        if (rc == 0) {
            ESP_LOGD(TAG, "Notification sent, attr_handle = %d", attr_handle);
        } else {
            ESP_LOGE(TAG, "Error in sending notification, rc = %d", rc);
        }
        return rc;
    }

    /* If notifications are disabled return ESP_FAIL */
    ESP_LOGI(TAG, "Notify is disabled");
    return ESP_FAIL;
}

void
esp_ble_ota_fill_handle_table(void)
{
    ota_handle_table[RECV_FW_CHAR] = receive_fw_val;
    ota_handle_table[OTA_STATUS_CHAR] = ota_status_val;
    ota_handle_table[CMD_CHAR] = command_val;
    ota_handle_table[CUS_CHAR] = custom_val;
}

#endif
// OTA add end


#define  THRPT_SVC                           0x0001
#define  THRPT_CHR_READ_WRITE                0x0006
#define  THRPT_CHR_NOTIFY                    0x000a
#define  THRPT_LONG_CHR_READ_WRITE           0x000b

#define READ_THROUGHPUT_PAYLOAD            500
#define WRITE_THROUGHPUT_PAYLOAD           500

static const char *tag = "bleprph_throughput";

static uint8_t gatt_svr_thrpt_static_long_val[READ_THROUGHPUT_PAYLOAD];
static uint8_t gatt_svr_thrpt_static_short_val[WRITE_THROUGHPUT_PAYLOAD];
uint16_t notify_handle;

static int
gatt_svr_read_write_long_test(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt,
                              void *arg);


static const struct ble_gatt_svc_def gatts_test_svcs[] = {
    /* jeffery add 
        * service Device Information Service should be put head,
        * then it will be displayed in the nRF Connect app first
    */
    /* Device Information */
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &dev_info_svc_uuid.u,
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {/* Manufacturer Name String */
                 .uuid = &manufacturer_name_chr_uuid.u,
                 .access_cb = device_information_access,
                 .flags = BLE_GATT_CHR_F_READ,
                 .val_handle = &manufacturer_name_chr_val_handle},
                {/* Model Number String */
                 .uuid = &model_number_chr_uuid.u,
                 .access_cb = device_information_access,
                 .flags = BLE_GATT_CHR_F_READ,
                 .val_handle = &model_number_chr_val_handle},
                {/* Serial Number String */
                 .uuid = &serial_number_chr_uuid.u,
                 .access_cb = device_information_access,
                 .flags = BLE_GATT_CHR_F_READ,
                 .val_handle = &serial_number_chr_val_handle},
                {/* Hardware Revision String */
                 .uuid = &hardware_rev_chr_uuid.u,
                 .access_cb = device_information_access,
                 .flags = BLE_GATT_CHR_F_READ,
                 .val_handle = &hardware_rev_chr_val_handle},
                {/* Firmware Revision String */
                 .uuid = &firmware_rev_chr_uuid.u,
                 .access_cb = device_information_access,
                 .flags = BLE_GATT_CHR_F_READ,
                 .val_handle = &firmware_rev_chr_val_handle},
                {/* Software Revision String */
                 .uuid = &software_rev_chr_uuid.u,
                 .access_cb = device_information_access,
                 .flags = BLE_GATT_CHR_F_READ,
                 .val_handle = &software_rev_chr_val_handle},
                {0, /* No more characteristics in this service. */}},
    },
    /* Transparent Transmission */
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = TRANSP_SVC_UUID_DECLARE(),
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {/* Write, Notify and Indicate */
                 .uuid = TRANSP_CHR_WNI_UUID_DECLARE(),
                 .access_cb = transparent_chr_write_notify_indicate,
                 .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE|BLE_GATT_CHR_F_WRITE_NO_RSP,
                 .val_handle = &transparent_notify_handle},
                {/* Write only */
                 .uuid = TRANSP_CHR_W_UUID_DECLARE(),
                 .access_cb = transparent_chr_write,
                 .flags = BLE_GATT_CHR_F_WRITE|BLE_GATT_CHR_F_WRITE_NO_RSP},
                {/* Write and Notify */
                 .uuid = TRANSP_CHR_WN_UUID_DECLARE(),
                 .access_cb = transparent_chr_write_indicate,
                 .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY},
                {0, /* No more characteristics in this service. */}},
    },

    {
        /* OTA Service Declaration */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_OTA_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[])
        {
            {
                /* Receive Firmware Characteristic */
                .uuid = BLE_UUID16_DECLARE(RECV_FW_UUID),
                .access_cb = ble_ota_gatt_handler,
                .val_handle = &receive_fw_val,
                .flags = BLE_GATT_CHR_F_INDICATE | BLE_GATT_CHR_F_WRITE,
            }, {
                /* OTA Characteristic */
                .uuid = BLE_UUID16_DECLARE(OTA_BAR_UUID),
                .access_cb = ble_ota_gatt_handler,
                .val_handle = &ota_status_val,
                .flags = BLE_GATT_CHR_F_INDICATE | BLE_GATT_CHR_F_READ,
            }, {
                /* Command Characteristic */
                .uuid = BLE_UUID16_DECLARE(COMMAND_UUID),
                .access_cb = ble_ota_gatt_handler,
                .val_handle = &command_val,
                .flags = BLE_GATT_CHR_F_INDICATE | BLE_GATT_CHR_F_WRITE,
            }, {
                /* Customer characteristic */
                .uuid = BLE_UUID16_DECLARE(CUSTOMER_UUID),
                .access_cb = ble_ota_gatt_handler,
                .val_handle = &custom_val,
                .flags = BLE_GATT_CHR_F_INDICATE | BLE_GATT_CHR_F_WRITE,
            }, {
                0, /* No more characteristics in this service */
            }
        },
    },

    /* Throughput Test Service */
    {
        /*** Service: THRPT test. */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = THRPT_UUID_DECLARE(THRPT_SVC),
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                .uuid = THRPT_UUID_DECLARE(THRPT_CHR_READ_WRITE),
                .access_cb = gatt_svr_read_write_long_test,
                .flags = BLE_GATT_CHR_F_READ |
                BLE_GATT_CHR_F_WRITE,

            }, {
                .uuid = THRPT_UUID_DECLARE(THRPT_CHR_NOTIFY),
                .access_cb = gatt_svr_read_write_long_test,
                .val_handle = &notify_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            }, {
                .uuid = THRPT_UUID_DECLARE(THRPT_LONG_CHR_READ_WRITE),
                .access_cb = gatt_svr_read_write_long_test,
                .flags = BLE_GATT_CHR_F_WRITE |
                BLE_GATT_CHR_F_READ,
            }, {
                0, /* No more characteristics in this service. */
            }
        },
    },

    {
        0, /* No more services. */
    },
};

static uint16_t
extract_uuid16_from_thrpt_uuid128(const ble_uuid_t *uuid)
{
    const uint8_t *u8ptr;
    uint16_t uuid16;

    u8ptr = BLE_UUID128(uuid)->value;
    uuid16 = u8ptr[12];
    uuid16 |= (uint16_t)u8ptr[13] << 8;
    return uuid16;
}

static int
gatt_svr_chr_write(uint16_t conn_handle, uint16_t attr_handle,
                   struct os_mbuf *om, uint16_t min_len, uint16_t max_len,
                   void *dst, uint16_t *len)
{
    uint16_t om_len;
    int rc;

    om_len = OS_MBUF_PKTLEN(om);
    if (om_len < min_len || om_len > max_len) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    rc = ble_hs_mbuf_to_flat(om, dst, max_len, len);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    return 0;
}

static int
gatt_svr_read_write_long_test(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt,
                              void *arg)
{
    uint16_t uuid16;
    int rc;

    uuid16 = extract_uuid16_from_thrpt_uuid128(ctxt->chr->uuid);
    assert(uuid16 != 0);

    switch (uuid16) {
    case THRPT_LONG_CHR_READ_WRITE:
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            rc = gatt_svr_chr_write(conn_handle, attr_handle,
                                    ctxt->om, 0,
                                    sizeof gatt_svr_thrpt_static_long_val,
                                    &gatt_svr_thrpt_static_long_val, NULL);
            //jeffery 8/27/2025
            ESP_LOGI("Jeffery",
             "%02X, %02X, %02X, %02X, %02X, %02X",
             gatt_svr_thrpt_static_long_val[0], gatt_svr_thrpt_static_long_val[1], gatt_svr_thrpt_static_long_val[2],
             gatt_svr_thrpt_static_long_val[3], gatt_svr_thrpt_static_long_val[4], gatt_svr_thrpt_static_long_val[5]);
             //jeffery 8/27/2025 End
            return rc;
        } else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            gatt_svr_thrpt_static_long_val[0] = rand();
            rc = os_mbuf_append(ctxt->om, &gatt_svr_thrpt_static_long_val,
                                sizeof gatt_svr_thrpt_static_long_val);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        return 0;

    case THRPT_CHR_READ_WRITE:
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            rc = gatt_svr_chr_write(conn_handle, attr_handle,
                                    ctxt->om, 0,
                                    sizeof gatt_svr_thrpt_static_short_val,
                                    gatt_svr_thrpt_static_short_val, NULL);
            return rc;
        } else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            rc = os_mbuf_append(ctxt->om, gatt_svr_thrpt_static_short_val,
                                sizeof gatt_svr_thrpt_static_short_val);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        return BLE_ATT_ERR_UNLIKELY;

    default:
        assert(0);
        return BLE_ATT_ERR_UNLIKELY;
    }
}

// jeffery add
static int 
device_information_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg) {
    /* Local variables */
    int rc;
    const char *value = NULL;

    /* Handle access events */
    /* Note: Device Information characteristic is read only */
    switch (ctxt->op) {

    /* Read characteristic event */
    case BLE_GATT_ACCESS_OP_READ_CHR:
        /* Verify connection handle */
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            ESP_LOGI(tag, "characteristic read; conn_handle=%d attr_handle=%d",
                     conn_handle, attr_handle);
        } else {
            ESP_LOGI(tag, "characteristic read by nimble stack; attr_handle=%d",
                     attr_handle);
        }

        /* Verify attribute handle and set corresponding value */
        if (attr_handle == manufacturer_name_chr_val_handle) {
            value = manufacturer_name;
        } else if (attr_handle == model_number_chr_val_handle) {
            value = model_number;
        } else if (attr_handle == serial_number_chr_val_handle) {
            value = serial_number;
        } else if (attr_handle == hardware_rev_chr_val_handle) {
            value = hardware_rev;
        } else if (attr_handle == firmware_rev_chr_val_handle) {
            value = firmware_rev;
        } else if (attr_handle == software_rev_chr_val_handle) {
            value = software_rev;
        } else {
            goto error;
        }

        /* Update access buffer value */
        rc = os_mbuf_append(ctxt->om, value, strlen(value));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

    /* Unknown event */
    default:
        goto error;
    }
error:
    ESP_LOGE(tag,
             "unexpected access operation to device information characteristic, opcode: %d",
             ctxt->op);
    return BLE_ATT_ERR_UNLIKELY; 
}
extern uint8_t transparent_payload[20];
extern SemaphoreHandle_t sem_transparent_notify;
extern bool notify_state_transparent;
extern int32_t bleWrTimeStart;
extern int32_t bleWrTimeEnd;
static int transparent_chr_write_notify_indicate(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg){
    int rc = BLE_ATT_ERR_UNLIKELY;
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            if(notify_state_transparent == true) {
                bleWrTimeStart = esp_log_timestamp();
                ESP_LOGI("Jeffery_NOTIFY_INDICATE", "Time %ld us", (int32_t)(bleWrTimeStart));
                queue_item_t ble_write_item;
                ble_write_item.len = ctxt->om->om_len;
                memcpy(ble_write_item.data, ctxt->om->om_data, ble_write_item.len);
                ESP_LOGI("Jeffery BLE_WRITE_NOTIFY", "len=%d", ble_write_item.len);
                ESP_LOGI("Jeffery",
                "%02X, %02X, %02X, %02X, %02X, %02X",
                ble_write_item.data[0], ble_write_item.data[1], ble_write_item.data[2],
                ble_write_item.data[3], ble_write_item.data[4], ble_write_item.data[5]);
                // rc = gatt_svr_chr_write(conn_handle, attr_handle,
                //                         ctxt->om, 0,
                //                         sizeof transparent_payload,
                //                         transparent_payload, NULL);
                //jeffery 9/09/2025
                // ESP_LOGI("Jeffery",
                // "%02X, %02X, %02X, %02X, %02X, %02X",
                // transparent_payload[0], transparent_payload[1], transparent_payload[2],
                // transparent_payload[3], transparent_payload[4], transparent_payload[5]);
                ESP_LOGI("Jeffery", "write_notify_indicate");           
                xSemaphoreGive(sem_transparent_notify);           
                xQueueSend(q_ble2uart, (void *)&ble_write_item, 0);
         }
         else {
            ESP_LOGI("Jeffery", "Notify is not enabled");
         }
         //jeffery 9/09/2025 End
        return rc;
    } 
    else {
        assert(0);
        return BLE_ATT_ERR_UNLIKELY;
    }
}
static int transparent_chr_write(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg){
    int rc = BLE_ATT_ERR_UNLIKELY; 
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        if(notify_state_transparent == true) {
            queue_item_t ble_write_item;
            ble_write_item.len = ctxt->om->om_len;
            memcpy(ble_write_item.data, ctxt->om->om_data, ble_write_item.len);
            ESP_LOGI("Jeffery BLE_WRITE", "len=%d", ble_write_item.len);
            xQueueSend(q_ble2uart, (void *)&ble_write_item, 0);
            // rc = gatt_svr_chr_write(conn_handle, attr_handle,
            //                         ctxt->om, 0,
            //                         sizeof transparent_payload,
            //                         transparent_payload, NULL);
            //jeffery 9/09/2025
            ESP_LOGI("Jeffery", "chr_write");        
            xSemaphoreGive(sem_transparent_notify);           
        }
        else {
            ESP_LOGI("Jeffery", "Notify is not enabled");
        }
        return rc;
    } 
    else {
        assert(0);
        return BLE_ATT_ERR_UNLIKELY;
    }
}
static int transparent_chr_write_indicate(uint16_t conn_handle, uint16_t attr_handle,
                                        struct ble_gatt_access_ctxt *ctxt, void *arg){
    int rc; 
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        rc = gatt_svr_chr_write(conn_handle, attr_handle,
                                ctxt->om, 0,
                                sizeof(transparent_payload),
                                transparent_payload, NULL);
        return rc;
    } 
    else {
        assert(0);
        return BLE_ATT_ERR_UNLIKELY;
    }
}

void
gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGD(tag, "registered service %s with handle=%d",
                 ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                 ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGD(tag, "registering characteristic %s with "
                 "def_handle=%d val_handle=%d\n",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                 ctxt->chr.def_handle,
                 ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGD(tag, "registering descriptor %s with handle=%d",
                 ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                 ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

int
gatt_svr_init(void)
{
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatts_test_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatts_test_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}
