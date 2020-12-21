#include "esp_system.h"
#ifdef CONFIG_TARGET_DEVICE_ESP8266

#include <string.h>

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_smartconfig.h"
#include "esp_netif.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"


#include "app_wifi.h"
#include "app_nvs.h"
#include "app_smart_config.h"

#include "app_main.h"
#include "app_relay.h"

static const char *TAG = "MQTTS_SMARTCONFIG";

const char * smartconfigTAG="smartconfigFlag";
int smartconfigFlag = 0;

const char * wifi_ssid_tag;
const char * wifi_pass_tag;

char wifi_ssid[MAX_WIFI_CONFIG_LEN];
char wifi_pass[MAX_WIFI_CONFIG_LEN];

EventGroupHandle_t wifi_event_group;
extern const int WIFI_CONNECTED_BIT;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int REQUESTED_BIT = BIT0;
static const int CONNECTED_BIT = BIT1;
static const int ESPTOUCH_DONE_BIT = BIT2;


extern QueueHandle_t smartconfigQueue;
EventGroupHandle_t smartconfig_event_group;

#if CONFIG_MQTT_RELAYS_NB
extern int relayStatus[CONFIG_MQTT_RELAYS_NB];
extern QueueHandle_t relayQueue;
#endif //CONFIG_MQTT_RELAYS_NB

#define TICKS_FORMAT "%u"

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    ESP_LOGW(TAG, "Wifi: SYSTEM_EVENT_STA_START");
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    ESP_LOGW(TAG, "SYSTEM_EVENT_STA_DISCONNECTED");
    xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    esp_wifi_connect();
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
    xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
  }
}

static void initialise_wifi(void)
{

  memset(wifi_ssid, 0, MAX_WIFI_CONFIG_LEN);
  memset(wifi_pass, 0, MAX_WIFI_CONFIG_LEN);

  size_t length = sizeof(wifi_ssid);
  esp_err_t err=read_nvs_str(wifi_ssid_tag, wifi_ssid, &length);
  ESP_ERROR_CHECK( err );

  length = sizeof(wifi_pass);
  err=read_nvs_str(wifi_pass_tag, wifi_pass, &length);
  ESP_ERROR_CHECK( err );


  ESP_ERROR_CHECK(esp_netif_init());

  ESP_ERROR_CHECK(esp_event_loop_create_default());

  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  wifi_config_t wifi_config = {
     .sta = {
      .ssid = CONFIG_WIFI_SSID,
      .password = CONFIG_WIFI_PASSWORD,
      /* Setting a password implies station will connect to all security modes including WEP/WPA.
       * However these modes are deprecated and not advisable to be used. Incase your Access point
       * doesn't support WPA2, these mode can be enabled by commenting below line */
      .threshold.authmode = WIFI_AUTH_WPA2_PSK,

      .pmf_cfg = {
                  .capable = true,
                  .required = false
                  },
             },
  };

  if (strlen(wifi_ssid) && strlen(wifi_pass)) {
    ESP_LOGI(TAG, "using nvs wifi config");
    strcpy((char*)wifi_config.sta.ssid, wifi_ssid);
    strcpy((char*)wifi_config.sta.password, wifi_pass);
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
  ESP_LOGI(TAG, "start the WIFI SSID:[%s]", wifi_config.sta.ssid);
  ESP_LOGI(TAG, "connecting with pass:[%s]", wifi_config.sta.password);
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_LOGI(TAG, "wifi_init_sta finished, waiting for wifi");

  xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);

}

void smartconfig_cmd_task(void* pvParameters)
{
  ESP_LOGI(TAG, "smartconfig_cmd_task started");
  smartconfig_event_group = xEventGroupCreate();
  struct SmartConfigMessage scm;;
  EventBits_t uxBits;
  if (smartconfigFlag) {
    ESP_LOGI(TAG, "starting smartconfig");
    initialise_wifi();
    while (1) {
      uxBits = xEventGroupWaitBits(smartconfig_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);

      if(uxBits & CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi Connected to ap");
      }
      if(uxBits & ESPTOUCH_DONE_BIT) {
        ESP_LOGI(TAG, "smartconfig over");

        esp_err_t err=write_nvs_str(wifi_ssid_tag, wifi_ssid);
        ESP_ERROR_CHECK( err );

        err=write_nvs_str(wifi_pass_tag, wifi_pass);
        ESP_ERROR_CHECK( err );


        ESP_LOGI(TAG, "Prepare to restart system in 10 seconds!");
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        esp_restart();
      }
      vTaskDelay(1000 / portTICK_PERIOD_MS);

    }
  } else {
    ESP_LOGI(TAG, "smartconfig not enabled, waiting request");
    TickType_t pushTick = 0;
    TickType_t lastRead = 0;
    const TickType_t ticksToWait = 3000/portTICK_PERIOD_MS ; // 3 seconds
    ESP_LOGI(TAG, "ticksToWait: " TICKS_FORMAT, ticksToWait);
    while (1) {
      if( xQueueReceive( smartconfigQueue, &scm , portMAX_DELAY) )
        {
          ESP_LOGI(TAG, "received switch event at: %d", scm.ticks);
          if (scm.ticks - lastRead < 5) {
            ESP_LOGI(TAG, "duplicate call, ignored ");
            lastRead = scm.ticks;
            continue;
          }
          lastRead = scm.ticks;
          if (pushTick == 0) {
            ESP_LOGI(TAG, "down ");
            pushTick = scm.ticks;
          } else {
            ESP_LOGI(TAG, "up ");
            if ((scm.ticks - pushTick ) < ticksToWait) {
#if CONFIG_MQTT_RELAYS_NB
              struct RelayMessage r={RELAY_CMD_STATUS, scm.relayId, !(relayStatus[(int)scm.relayId] == RELAY_ON)};
              xQueueSend(relayQueue,
                         ( void * )&r,
                         RELAY_QUEUE_TIMEOUT);
#endif //CONFIG_MQTT_RELAYS_NB
            }
            else {
              ESP_LOGI(TAG, "received smartconfig request:");
              ESP_ERROR_CHECK(write_nvs_integer(smartconfigTAG, ! smartconfigFlag));
              ESP_LOGI(TAG, "Prepare to restart system in 10 seconds!");
              vTaskDelay(10000 / portTICK_PERIOD_MS);
              esp_restart();
            }
            pushTick = 0;
          }
          ESP_LOGI(TAG, "pushTick: " TICKS_FORMAT, pushTick);
        }
    }
  }
}
#endif //CONFIG_TARGET_DEVICE_ESP8266
