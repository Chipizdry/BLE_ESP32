

#pragma once


#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"

#define PROFILE_APP_IDX 0
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void start_ble_telemetry_service(void);