

#pragma once

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"

#define PROFILE_APP_IDX 0

// Объявления глобальных переменных для доступа из других файлов
extern uint16_t telemetry_conn_id;
extern esp_gatt_if_t telemetry_gatts_if;
extern uint16_t telemetry_handle_table[];
extern bool device_connected;

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void telemetry_init(void);
void telemetry_start(void);