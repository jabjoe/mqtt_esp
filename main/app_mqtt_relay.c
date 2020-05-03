#include "esp_system.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "mqtt_client.h"

#include "app_main.h"

#include "app_sensors.h"
#include "app_mqtt.h"

#include "app_mqtt_util.h"

#if CONFIG_MQTT_RELAYS_NB
static const char *TAG = "MQTTS_MQTT_RELAY";

#include "app_relay.h"
extern QueueHandle_t relayQueue;

void handle_relay_mqtt_status_cmd(signed char relayId, const char *payload)
{
  struct RelayMessage rm;
  memset(&rm, 0, sizeof(struct RelayMessage));
  rm.msgType = RELAY_CMD_STATUS;
  rm.relayId = relayId;

  if (strcmp(payload, "ON") == 0)
    rm.data = RELAY_STATUS_ON;
  else if (strcmp(payload, "OFF") == 0)
    rm.data = RELAY_STATUS_OFF;

  if (xQueueSend( relayQueue
                  ,( void * )&rm
                  ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
    ESP_LOGE(TAG, "Cannot send to relayQueue");
  }
}

void handle_relay_mqtt_sleep_cmd(signed char relayId, const char *payload)
{
  struct RelayMessage rm;
  memset(&rm, 0, sizeof(struct RelayMessage));
  rm.msgType = RELAY_CMD_SLEEP;
  rm.relayId = relayId;

  rm.data = atoi(payload);

  if (xQueueSend( relayQueue
                  ,( void * )&rm
                  ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
    ESP_LOGE(TAG, "Cannot send to relayQueue");
  }
}


void handle_relay_mqtt_cmd(const char* topic, const char* payload)
{
  char action[16];
  getAction(action, topic);

  signed char relayId = getServiceId(topic);
  if (relayId != -1) {
    if (strcmp(action, "status") == 0) {
      handle_relay_mqtt_status_cmd(relayId, payload);
      return;
    }
    if (strcmp(action, "sleep") == 0) {
      handle_relay_mqtt_sleep_cmd(relayId, payload);
      return;
    }
    ESP_LOGW(TAG, "unhandled relay action: %s", action);
    return;
  }
  ESP_LOGW(TAG, "unhandled relay id: %d", relayId);
}

#endif // CONFIG_MQTT_RELAYS_NB
