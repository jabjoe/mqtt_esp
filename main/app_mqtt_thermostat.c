#include "esp_system.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "mqtt_client.h"

#include "app_main.h"

#include "app_sensors.h"
#include "app_mqtt.h"

#if CONFIG_MQTT_THERMOSTATS_NB > 0

void handle_thermostat_mqtt_mode_cmd(signed char thermostatId, const char *payload)
{
  struct ThermostatMessage tm;
  memset(&tm, 0, sizeof(struct ThermostatMessage));
  tm.msgType = THERMOSTAT_CMD_MODE;
  tm.thermostatId = thermostatId;
  if (strcmp(payload, "heat") == 0)
    tm.data.thermostatMode = THERMOSTAT_MODE_HEAT;
  else if (strcmp(payload, "off") == 0)
    tm.data.thermostatMode = THERMOSTAT_MODE_OFF;

  if (tm.data.thermostatMode == THERMOSTAT_MODE_UNSET) {
    ESP_LOGE(TAG, "wrong payload");
    return;
  }

  if (xQueueSend( thermostatQueue
                  ,( void * )&tm
                  ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
    ESP_LOGE(TAG, "Cannot send to thermostatQueue");
  }
}

void handle_thermostat_mqtt_temp_cmd(signed char thermostatId, const char *payload)
{
  struct ThermostatMessage tm;
  memset(&tm, 0, sizeof(struct ThermostatMessage));
  tm.msgType = THERMOSTAT_CMD_TARGET_TEMPERATURE;
  tm.thermostatId = thermostatId;
  tm.data.targetTemperature = atof(payload) * 10;

  if (xQueueSend( thermostatQueue
                  ,( void * )&tm
                  ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
    ESP_LOGE(TAG, "Cannot send to thermostatQueue");
  }
}

void handle_thermostat_mqtt_tolerance_cmd(signed char thermostatId, const char *payload)
{
  struct ThermostatMessage tm;
  memset(&tm, 0, sizeof(struct ThermostatMessage));
  tm.msgType = THERMOSTAT_CMD_TOLERANCE;
  tm.thermostatId = thermostatId;
  tm.data.tolerance = atof(payload) * 10;

  if (xQueueSend( thermostatQueue
                  ,( void * )&tm
                  ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
    ESP_LOGE(TAG, "Cannot send to thermostatQueue");
  }
}

void handle_thermostat_mqtt_cmd(const char* topic, const char* payload)
{
  char action[16];
  getAction(action, topic);
  signed char thermostatId = getServiceId(topic);
  if (thermostatId != -1) {
    if (strcmp(action, "mode") == 0) {
      handle_thermostat_mqtt_mode_cmd(thermostatId, payload);
      return;
    }
    if (strcmp(action, "temp") == 0) {
      handle_thermostat_mqtt_temp_cmd(thermostatId, payload);
      return;
    }
    if (strcmp(action, "tolerance") == 0) {
      handle_thermostat_mqtt_tolerance_cmd(thermostatId, payload);
      return;
    }
  }
  ESP_LOGW(TAG, "unhandled water thermostat: %s", action);
}

#endif // CONFIG_MQTT_THERMOSTATS_NB > 0

