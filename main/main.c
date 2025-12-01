/**
 ******************************************************************************
 * @file        main.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-01
 * @brief       WiFi-AP 模式创建 wifi 热点实验
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ******************************************************************************
 * @attention
 * 
 * 实验平台:正点原子 ESP32-S3 开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 ******************************************************************************
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_mac.h"
#include <netdb.h>
#include "led.h"
#include "myiic.h"
#include "my_spi.h"
#include "spilcd.h"
#include "xl9555.h"
#include <stdio.h>
#include <inttypes.h>

/* 蓝牙相关头文件 */
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"


static const char *TAG = "AP";
#define EXAMPLE_ESP_WIFI_SSID   "ESP32_S3_AP"
#define EXAMPLE_ESP_WIFI_PASS   "123456789"
#define EXAMPLE_MAX_STA_CONN    5
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
static char lcd_buff[100] = {0};

/* 蓝牙相关定义 */
static const char *BLE_TAG = "BLE";
#define GATTS_TABLE_TAG "BLE_GATTS_DEMO"

/* 服务UUID */
#define GATTS_SERVICE_UUID   0x00FF
/* 特征UUID */
#define GATTS_CHAR_UUID      0xFF01
#define GATTS_NUM_HANDLE     4

#define PROFILE_NUM 1
#define PROFILE_APP_IDX 0

/* 蓝牙设备名称 */
#define DEVICE_NAME          "ESP32-S3_BLE"
#define MANUFACTURER_DATA_LEN  17

static uint8_t manufacturer_data[MANUFACTURER_DATA_LEN] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0x10,0x11};

static uint8_t adv_config_done = 0;

#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

static uint8_t char_data[20] = {0x00};

/* 使用ESP-IDF v5.x兼容的蓝牙广告配置 */
static uint8_t adv_data_raw[] = {
    /* Flags */
    0x02, 0x01, 0x06,
    /* Complete Local Name */
    0x0d, 0x09, 'E', 'S', 'P', '3', '2', '-', 'S', '3', '_', 'B', 'L', 'E',
    /* Service UUID */
    0x03, 0x03, 0xFF, 0x00,
    /* Manufacturer Data */
    0x12, 0xFF, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11
};

static uint8_t scan_rsp_data_raw[] = {
    /* Flags */
    0x02, 0x01, 0x06,
    /* Complete Local Name */
    0x0d, 0x09, 'E', 'S', 'P', '3', '2', '-', 'S', '3', '_', 'B', 'L', 'E',
    /* Service UUID */
    0x03, 0x03, 0xFF, 0x00
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* 启动蓝牙广告的函数 */
static void start_ble_advertising(void)
{
    esp_err_t ret;
    
    /* 设置设备名称 */
    ret = esp_ble_gap_set_device_name(DEVICE_NAME);
    if (ret != ESP_OK) {
        ESP_LOGE(BLE_TAG, "Set device name failed: %s", esp_err_to_name(ret));
        return;
    }

    /* 使用ESP-IDF v5.x兼容的API配置广告数据 - 更简单的版本 */
    /* 配置广告数据：只包含设备名称和通用发现标志 */
    esp_ble_adv_data_t adv_data = {
        .set_scan_rsp = false,
        .include_name = true,
        .include_txpower = true,
        .min_interval = 0x20,
        .max_interval = 0x40,
        .appearance = 0x00,
        .manufacturer_len = 0,
        .p_manufacturer_data = NULL,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = 0,
        .p_service_uuid = NULL,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
    };

    /* 配置广告数据 */
    ret = esp_ble_gap_config_adv_data(&adv_data);
    if (ret != ESP_OK) {
        ESP_LOGE(BLE_TAG, "Config adv data failed: %s", esp_err_to_name(ret));
        return;
    }

    /* 配置扫描响应数据：包含设备名称 */
    esp_ble_adv_data_t scan_rsp_data = {
        .set_scan_rsp = true,
        .include_name = true,
        .include_txpower = true,
        .min_interval = 0x20,
        .max_interval = 0x40,
        .appearance = 0x00,
        .manufacturer_len = 0,
        .p_manufacturer_data = NULL,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = 0,
        .p_service_uuid = NULL,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
    };

    ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
    if (ret != ESP_OK) {
        ESP_LOGE(BLE_TAG, "Config scan response data failed: %s", esp_err_to_name(ret));
        return;
    }

    /* 启动广告 - 使用全局定义的adv_params */
    ret = esp_ble_gap_start_advertising(&adv_params);
    if (ret != ESP_OK) {
        ESP_LOGE(BLE_TAG, "Start advertising failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(BLE_TAG, "BLE advertising started successfully");
    spilcd_show_string(0, 130, 240, 16, 16, "BLE Advertising", BLUE);
}

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* 一个简单的profile */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* 尚未获取到gatts_if，初始化为ESP_GATT_IF_NONE */
    },
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);


/**
 * @brief       WIFI链接糊掉函数
 * @param       arg:传入网卡控制块
 * @param       event_base:WIFI事件
 * @param       event_id:事件ID
 * @param       event_data:事件数据
 * @retval      无
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    /* 设备连接 */
    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        spilcd_fill(0,90,320,240,WHITE);
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac), event->aid);
        sprintf(lcd_buff, "MACSTR:"MACSTR,MAC2STR(event->mac));
        spilcd_show_string(0, 90, 320, 16, 16, lcd_buff, BLUE);
        spilcd_show_string(0, 110, 320, 16, 16, "With device connection", BLUE);
    } 
    /* 设备断开 */
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
        spilcd_fill(0,90,320,320,WHITE);
        sprintf(lcd_buff, "Device disconnected:"MACSTR,MAC2STR(event->mac));
        spilcd_show_string(0, 90, 320, 16, 16, lcd_buff, BLUE);
    }
}

/**
 * @brief       WIFI初始化
 * @param       无
 * @retval      无
 */
static void wifi_init_softap(void)
{
    /* 初始化网卡 */
    ESP_ERROR_CHECK(esp_netif_init());

    /* 创建新的事件循环 */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    /* 使用默认配置初始化包括netif的Wi-Fi */
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    /* 配置WIFI */
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip_info);

    char ip_addr[16];
    inet_ntoa_r(ip_info.ip.addr, ip_addr, 16);
    ESP_LOGI(TAG, "Set up softAP with IP: %s", ip_addr);

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:'%s' password:'%s'",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    
    spilcd_show_string(0, 90, 240, 16, 16, "wifi connecting......", BLUE);
}

/**
 * @brief       蓝牙GAP事件处理函数
 * @param       event: GAP事件
 * @param       param: 事件参数
 * @retval      无
 */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        /* 广告开始完成 */
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(BLE_TAG, "Advertising start failed");
        } else {
            ESP_LOGI(BLE_TAG, "Advertising start successfully");
            spilcd_show_string(0, 130, 240, 16, 16, "BLE Advertising", BLUE);
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(BLE_TAG, "Advertising stop failed");
        } else {
            ESP_LOGI(BLE_TAG, "Stop adv successfully");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(BLE_TAG, "Update connection params status = %d, min_int = %"PRIu16", max_int = %"PRIu16",conn_int = %"PRIu16",latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    /* 配对相关事件 */
    case ESP_GAP_BLE_SEC_REQ_EVT:
        ESP_LOGI(BLE_TAG, "ESP_GAP_BLE_SEC_REQ_EVT");
        /* 接受配对请求 */
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        ESP_LOGI(BLE_TAG, "ESP_GAP_BLE_AUTH_CMPL_EVT");
        if (!param->ble_security.auth_cmpl.success) {
            ESP_LOGE(BLE_TAG, "Pairing failed: %d", param->ble_security.auth_cmpl.fail_reason);
            spilcd_show_string(0, 150, 240, 16, 16, "Pairing Failed", RED);
        } else {
            ESP_LOGI(BLE_TAG, "Pairing success");
            spilcd_show_string(0, 150, 240, 16, 16, "Paired Success", GREEN);
            /* 显示配对设备地址 */
            char addr_str[18];
            sprintf(addr_str, "%02x:%02x:%02x:%02x:%02x:%02x",
                    param->ble_security.auth_cmpl.bd_addr[0],
                    param->ble_security.auth_cmpl.bd_addr[1],
                    param->ble_security.auth_cmpl.bd_addr[2],
                    param->ble_security.auth_cmpl.bd_addr[3],
                    param->ble_security.auth_cmpl.bd_addr[4],
                    param->ble_security.auth_cmpl.bd_addr[5]);
            ESP_LOGI(BLE_TAG, "Paired with: %s", addr_str);
        }
        break;
    case ESP_GAP_BLE_KEY_EVT:
        ESP_LOGI(BLE_TAG, "ESP_GAP_BLE_KEY_EVT key type = %d", param->ble_security.ble_key.key_type);
        break;
    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
        ESP_LOGI(BLE_TAG, "ESP_GAP_BLE_PASSKEY_NOTIF_EVT passkey = %"PRIu32, param->ble_security.key_notif.passkey);
        /* 显示配对密码 */
        char passkey_str[7];
        sprintf(passkey_str, "%06"PRIu32, param->ble_security.key_notif.passkey);
        spilcd_show_string(0, 170, 240, 16, 16, passkey_str, BLUE);
        break;
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:
        ESP_LOGI(BLE_TAG, "ESP_GAP_BLE_PASSKEY_REQ_EVT");
        /* 设置固定配对密码：000000 */
        esp_ble_passkey_reply(param->ble_security.ble_req.bd_addr, true, 0);
        break;
    default:
        break;
    }
}

/**
 * @brief       蓝牙GATTS事件处理函数
 * @param       event: GATTS事件
 * @param       gatts_if: GATTS接口
 * @param       param: 事件参数
 * @retval      无
 */
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* 如果事件是注册事件，我们将为每个profile存储gatts_if */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGI(BLE_TAG, "Reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* 如果gatts_if等于profile的gatts_if，则profile处理事件，
     * 否则其他profile处理事件 */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE，不指定gatts_if，表示所有gatts_if都处理 */
                    gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

/**
 * @brief       Profile事件处理函数
 * @param       event: GATTS事件
 * @param       gatts_if: GATTS接口
 * @param       param: 事件参数
 * @retval      无
 */
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(BLE_TAG, "REGISTER_APP_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
        gl_profile_tab[PROFILE_APP_IDX].service_id.is_primary = true;
        gl_profile_tab[PROFILE_APP_IDX].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_APP_IDX].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_APP_IDX].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID;

        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_APP_IDX].service_id, GATTS_NUM_HANDLE);
        break;
    case ESP_GATTS_READ_EVT:
        ESP_LOGI(BLE_TAG, "GATT_READ_EVT, conn_id %u, trans_id %u, handle %u",
                (unsigned int)param->read.conn_id, (unsigned int)param->read.trans_id, (unsigned int)param->read.handle);
        {
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.len = sizeof(char_data);
            memcpy(rsp.attr_value.value, char_data, rsp.attr_value.len);
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                        ESP_GATT_OK, &rsp);
        }
        break;
    case ESP_GATTS_WRITE_EVT:
        ESP_LOGI(BLE_TAG, "GATT_WRITE_EVT, conn_id %u, trans_id %u, handle %u",
                (unsigned int)param->write.conn_id, (unsigned int)param->write.trans_id, (unsigned int)param->write.handle);
        if (!param->write.is_prep){
            /* 数据长度 */
            ESP_LOGI(BLE_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
            esp_log_buffer_hex(BLE_TAG, param->write.value, param->write.len);

            if (param->write.len < sizeof(char_data)) {
                memcpy(char_data, param->write.value, param->write.len);
                /* 在LCD上显示接收到的数据 */
                sprintf(lcd_buff, "BLE Data: %s", char_data);
                spilcd_show_string(0, 150, 240, 16, 16, lcd_buff, BLUE);
            }
        }
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        break;
    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(BLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(BLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(BLE_TAG, "ESP_GATTS_CONF_EVT, status %d", param->conf.status);
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(BLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(BLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        gl_profile_tab[PROFILE_APP_IDX].conn_id = param->connect.conn_id;
        sprintf(lcd_buff, "BLE Connected");
        spilcd_show_string(0, 130, 240, 16, 16, lcd_buff, GREEN);
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(BLE_TAG, "ESP_GATTS_DISCONNECT_EVT, conn_id %d", param->disconnect.conn_id);
        esp_ble_gap_start_advertising(&adv_params);
        sprintf(lcd_buff, "BLE Disconnected");
        spilcd_show_string(0, 130, 240, 16, 16, lcd_buff, RED);
        break;
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(BLE_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d", param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_APP_IDX].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_APP_IDX].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_APP_IDX].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID;
        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_APP_IDX].service_handle);
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT:
        ESP_LOGI(BLE_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d",
                 param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        gl_profile_tab[PROFILE_APP_IDX].char_handle = param->add_char.attr_handle;
        break;
    default:
        break;
    }
}

/**
 * @brief       蓝牙初始化函数
 * @param       无
 * @retval      无
 */
static void ble_init(void)
{
    esp_err_t ret;

    ESP_LOGI(BLE_TAG, "BLE INIT START");

    /* 初始化蓝牙控制器 */
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    /* 使能蓝牙控制器 */
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    /* 初始化蓝牙主机栈 */
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    /* 使能蓝牙主机栈 */
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    /* 注册GATT回调函数 */
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(BLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    /* 注册GAP回调函数 */
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(BLE_TAG, "gap register error, error code = %x", ret);
        return;
    }

    /* 设置配对参数 - 启用自动配对 */
    /* 设置IO能力为无输入无输出（Just Works配对） */
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    
    /* 设置认证要求：绑定，MITM保护 */
    uint8_t auth_req = ESP_LE_AUTH_BOND | ESP_LE_AUTH_REQ_MITM;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    
    /* 设置密钥大小 */
    uint8_t key_size = 16;
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    
    /* 设置初始化密钥分发 */
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    
    /* 设置响应密钥分发 */
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    /* 设置MTU大小 */
    ret = esp_ble_gatt_set_local_mtu(500);
    if (ret){
        ESP_LOGE(BLE_TAG, "set local  MTU failed, error code = %x", ret);
        return;
    }

    /* 初始化特征数据 */
    strcpy((char *)char_data, "Hello BLE");

    /* 注册应用 */
    ret = esp_ble_gatts_app_register(PROFILE_APP_IDX);
    if (ret != ESP_OK) {
        ESP_LOGE(BLE_TAG, "GATTS app register failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(BLE_TAG, "BLE INIT FINISHED");
    spilcd_show_string(0, 110, 240, 16, 16, "BLE Initialized", BLUE);
    
    /* 启动蓝牙广告 */
    start_ble_advertising();
}

/* 简化的蓝牙实现 - 移除复杂的属性表定义 */


/**
 * @brief       程序入口
 * @param       无
 * @retval      无
 */
void app_main(void)
{
    esp_err_t ret;

    ret = nvs_flash_init();     /* 初始化NVS */
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    led_init();                 /* LED初始化 */
    my_spi_init();              /* SPI初始化 */
    myiic_init();               /* IIC初始化 */  
    xl9555_init();              /* 初始化按键 */
    spilcd_init();              /* LCD屏初始化 */
 
    spilcd_show_string(0, 0, 240, 32, 32, "ESP32-S3", RED);
    spilcd_show_string(0, 40, 240, 24, 24, "WiFi+BLE Test", RED);
    spilcd_show_string(0, 70, 240, 16, 16, "ATOM@ALIENTEK", RED);
    
    /* 初始化WiFi AP */
    wifi_init_softap();
    
    /* 初始化蓝牙 */
    ble_init();

    while (1)
    {
        LED0_TOGGLE();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
