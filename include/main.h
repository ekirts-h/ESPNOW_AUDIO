#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include <esp_log.h>
#include "esp_system.h"
#include "driver/i2s_std.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_crc.h"
#include "esp_chip_info.h"
#include "esp_private/wifi.h"


#define ESP32
//#define ESP32_S3
//#define IS_SINK 
#define IS_SOURCE 
//#define EN_SPDIF

#ifdef EN_SPDIF
#include "SPDIF.h"
#endif





#ifdef ESP32_S3
//#define SAMPLE_RATE     96000
#define I2S_NUM         I2S_NUM_0
#define I2S_BCK_IO      11
#define I2S_WS_IO       12
#define I2S_DI_IO       13
#define I2S_DO_IO       14
#endif

#ifdef ESP32
//#define SAMPLE_RATE     96000
#define I2S_NUM         I2S_NUM_0
#define I2S_BCK_IO      26
#define I2S_WS_IO       25
#define I2S_DI_IO       22
//#define I2S_DO_IO       14
#endif

#define CONFIG_ESPNOW_CHANNEL 4 //default 0 // range 0 14

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

#define ESPNOW_QUEUE_SIZE  75

#define ESPNOW_MAXDELAY 512

#define ESPNOW_MAX_PAYLOAD_SIZE 250
#define ESPNOW_MAX_SENDCOUNT_MAX 250

#define ESPNOW_PMK "pmk1234567890123" //ESPNOW primary master key ESPNOW primary master for the example to use. The length of ESPNOW primary master must be 16 bytes.
#define ESPNOW_LMK "lmk1234567890123" //"ESPNOW local master key" SPNOW local master for the example to use. The length of ESPNOW local master must be 16 bytes.

#if CONFIG_ESPNOW_WIFI_MODE_STATION
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif

#define GET_PAYLOAD_SIZE //macro
#define GET_SEND_COUNT //macro
#define MAX_ESPNOW_SEND_LEN 250
//#define ESPNOW_SEND_LEN 248  //            Length of ESPNOW data to be sent, unit: byte. //  range 10 250
//#define ESPNOW_SEND_COUNT 100 //  Total count of unicast ESPNOW data to be sent. //range 1 65535
//#define ESPNOW_SEND_DELAY 2 //Delay between sending two ESPNOW data, unit: ms. //  range 0 65535 default 1000
/* 
    config ESPNOW_SEND_LEN
        int "Send len"
        range 10 250
        default 10
        help
            Length of ESPNOW data to be sent, unit: byte.
 */

enum{
    espnow_bd_sink,
    espnow_bd_source,
    espnow_bd_SOF_L,
    espnow_bd_EOF = 4,
};

enum{
    espnow_broadcast_ng,
    espnow_broadcast_ok,
};

static QueueHandle_t espnow_queue;

static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint8_t multicasttest_mac[ESP_NOW_ETH_ALEN] = {0x94, 0xE6, 0x86, 0x12, 0x29, 0xF8};
//f4:12:fa:84:78:4c
typedef enum {
    ESPNOW_SEND_CB,
    ESPNOW_RECV_CB,
} espnow_event_id_t;

enum {
    ESPNOW_DATA_BROADCAST,
    ESPNOW_DATA_UNICAST,
   // ESPNOW_DATA_MULTICAST,
    ESPNOW_DATA_MAX,
};
typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} espnow_event_send_cb_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} espnow_event_recv_cb_t;

typedef union {
    espnow_event_send_cb_t send_cb;
    espnow_event_recv_cb_t recv_cb;
} espnow_event_info_t;

/* When ESPNOW sending or receiving callback function is called, post event to ESPNOW task. */
typedef struct {
    espnow_event_id_t id;
    espnow_event_info_t info;
} espnow_event_t;

//add issource/sink here?
/* User defined field of ESPNOW data in this example. */
typedef struct {
    uint8_t type;                         //Broadcast or unicast ESPNOW data.
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint16_t seq_num;                     //Sequence number of ESPNOW data.
    uint16_t crc;                         //CRC16 value of ESPNOW data.
    uint8_t src_sink;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
    uint8_t payload[0];//ESPNOW_SEND_LEN];                   //Real payload of ESPNOW data.//0   since espnow_data_t is sent  payload size is smaller
} __attribute__((packed)) espnow_data_t;   // tells compiler to not add padding   so 10 byte doesnt become 12 etc to align data (32bit sys = 4 bytes)

typedef struct{
    bool unicast;                         //Send unicast ESPNOW data.
    bool broadcast;                       //Send broadcast ESPNOW data.
  //  bool multicast;
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint8_t src_sink;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
    uint16_t count;                       //Total count of unicast ESPNOW data to be sent.
    uint16_t delay;                       //Delay between sending two ESPNOW data, unit: ms.
    int len;                              //Length of ESPNOW data to be sent, unit: byte.
    uint8_t *buffer;                      //Buffer pointing to ESPNOW data.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   //MAC address of destination device.

} espnow_send_param_t;
