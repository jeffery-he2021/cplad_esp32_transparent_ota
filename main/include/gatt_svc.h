/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#ifndef GATT_SVR_H
#define GATT_SVR_H

/* Includes */
#include <stddef.h>
#include <stdint.h>
#include "ble_ota.h"
/* NimBLE GATT APIs */
// #include "host/ble_gatt.h"
// #include "services/gatt/ble_svc_gatt.h"

/* NimBLE GAP APIs */
extern uint16_t transparent_notify_handle;
esp_ble_ota_char_t
find_ota_char_and_desr_by_handle(uint16_t handle);

/* Public function declarations */
// void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
// void gatt_svr_subscribe_cb(struct ble_gap_event *event);
// int gatt_svc_init(void);

#endif // GATT_SVR_H
