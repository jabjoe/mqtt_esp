#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include <string.h>
#include <stdlib.h>

#include "mqtt_client.h"

#include "app_main.h"

#include "app_sensors.h"
#include "app_mqtt.h"

#include "cJSON.h"

#ifdef CONFIG_MQTT_SCHEDULERS

#include "app_scheduler.h"
extern QueueHandle_t schedulerCfgQueue;
//FIXME hack until decide if queue+thread really usefull
extern struct SchedulerCfgMessage schedulerCfg;
//FIXME end hack until decide if queue+thread really usefull

#define SCHEDULER_TOPICS_NB 1

#else // CONFIG_MQTT_SCHEDULERS

#define SCHEDULER_TOPICS_NB 0

#endif // CONFIG_MQTT_SCHEDULERS

#if CONFIG_MQTT_RELAYS_NB
#include "app_relay.h"
extern QueueHandle_t relayCmdQueue;

#define RELAY_SERVICE "relay"

#define RELAYS_TOPICS_NB 1 //CMD and CFG

#else //CONFIG_MQTT_RELAYS_NB

#define RELAYS_TOPICS_NB 0 //CMD and CFG

#endif //CONFIG_MQTT_RELAYS_NB

#ifdef CONFIG_MQTT_OTA

#include "app_ota.h"
extern QueueHandle_t otaQueue;
#define OTA_TOPIC CONFIG_MQTT_DEVICE_TYPE "/" CONFIG_MQTT_CLIENT_ID "/cmd/ota"
#define OTA_TOPICS_NB 1

#else // CONFIG_MQTT_OTA

#define OTA_TOPICS_NB 0

#endif // CONFIG_MQTT_OTA

#ifdef CONFIG_MQTT_THERMOSTAT

#include "app_thermostat.h"
extern QueueHandle_t thermostatQueue;
#define THERMOSTAT_TOPICS_NB 2
#define CMD_THERMOSTAT_TOPIC CONFIG_MQTT_DEVICE_TYPE "/" CONFIG_MQTT_CLIENT_ID "/cmd/+/thermostat"
#define CMD_WATER_THERMOSTAT_TOPIC CONFIG_MQTT_DEVICE_TYPE "/" CONFIG_MQTT_CLIENT_ID "/cmd/+/wthermostat"

#else // CONFIG_MQTT_THERMOSTAT

#define THERMOSTAT_TOPICS_NB 0

#endif // CONFIG_MQTT_THERMOSTAT

#ifdef CONFIG_MQTT_THERMOSTAT_HEATING_OPTIMIZER

#define OTHERMOSTAT_TOPICS_NB 1
#define CMD_OTHERMOSTAT_TOPIC CONFIG_MQTT_DEVICE_TYPE "/" CONFIG_MQTT_CLIENT_ID "/cmd/+/thermostat"

#else //CONFIG_MQTT_THERMOSTAT_HEATING_OPTIMIZER

#define OTHERMOSTAT_TOPICS_NB 0

#endif //CONFIG_MQTT_THERMOSTAT_HEATING_OPTIMIZER


esp_mqtt_client_handle_t client = NULL;

const char * available_topic = CONFIG_MQTT_DEVICE_TYPE "/" CONFIG_MQTT_CLIENT_ID "/evt/state/available";

EventGroupHandle_t mqtt_event_group;
const int MQTT_CONNECTED_BIT = BIT0;
const int MQTT_SUBSCRIBED_BIT = BIT1;
const int MQTT_PUBLISHED_BIT = BIT2;
const int MQTT_INIT_FINISHED_BIT = BIT3;

int mqtt_reconnect_counter;

#define FW_VERSION "0.02.12n2"

extern QueueHandle_t mqttQueue;

static const char *TAG = "MQTTS_MQTT";


#define NB_SUBSCRIPTIONS  (OTA_TOPICS_NB + OTHERMOSTAT_TOPICS_NB + THERMOSTAT_TOPICS_NB + RELAYS_TOPICS_NB + SCHEDULER_TOPICS_NB + CONFIG_MQTT_THERMOSTAT_ROOMS_SENSORS_NB)

#define RELAY_CMD_TOPIC CONFIG_MQTT_DEVICE_TYPE"/"CONFIG_MQTT_CLIENT_ID"/cmd/+/relay/"

#define SCHEDULER_CFG_TOPIC CONFIG_MQTT_DEVICE_TYPE"/"CONFIG_MQTT_CLIENT_ID"/cfg/scheduler/"

const char *SUBSCRIPTIONS[NB_SUBSCRIPTIONS] =
  {
#ifdef CONFIG_MQTT_OTA
    OTA_TOPIC,
#endif //CONFIG_MQTT_OTA
#ifdef CONFIG_MQTT_SCHEDULERS
    SCHEDULER_CFG_TOPIC "+",
#endif // CONFIG_MQTT_SCHEDULERS
#if CONFIG_MQTT_RELAYS_NB
    RELAY_CMD_TOPIC "+",
#endif //CONFIG_MQTT_RELAYS_NB
#ifdef CONFIG_MQTT_THERMOSTAT
    CMD_THERMOSTAT_TOPIC,
    CMD_WATER_THERMOSTAT_TOPIC,
#endif // CONFIG_MQTT_THERMOSTAT
#ifdef CONFIG_MQTT_THERMOSTAT_HEATING_OPTIMIZER
    CMD_OTHERMOSTAT_TOPIC,
#endif //CONFIG_MQTT_THERMOSTAT_HEATING_OPTIMIZER
#if CONFIG_MQTT_THERMOSTAT_ROOMS_SENSORS_NB
    CONFIG_MQTT_THERMOSTAT_ROOM_0_SENSORS_TOPIC,
#if CONFIG_MQTT_THERMOSTAT_ROOMS_SENSORS_NB > 1
    CONFIG_MQTT_THERMOSTAT_ROOM_1_SENSORS_TOPIC,
#if CONFIG_MQTT_THERMOSTAT_ROOMS_SENSORS_NB > 2
    CONFIG_MQTT_THERMOSTAT_ROOM_2_SENSORS_TOPIC,
#if CONFIG_MQTT_THERMOSTAT_ROOMS_SENSORS_NB > 3
    CONFIG_MQTT_THERMOSTAT_ROOM_3_SENSORS_TOPIC,
#endif //CONFIG_MQTT_THERMOSTAT_ROOMS_SENSORS_NB > 3
#endif //CONFIG_MQTT_THERMOSTAT_ROOMS_SENSORS_NB > 2
#endif //CONFIG_MQTT_THERMOSTAT_ROOMS_SENSORS_NB > 1
#endif //CONFIG_MQTT_THERMOSTAT_ROOMS_SENSORS_NB
  };


extern const char mqtt_iot_cipex_ro_pem_start[] asm("_binary_mqtt_iot_cipex_ro_pem_start");

unsigned char get_topic_id(esp_mqtt_event_handle_t event, int maxTopics, const char * topic)
{
  char fullTopic[MQTT_MAX_TOPIC_LEN];
  memset(fullTopic,0,MQTT_MAX_TOPIC_LEN);

  unsigned char topicId = 0;
  bool found = false;
  while (!found && topicId <= maxTopics) {
    sprintf(fullTopic, "%s%d", topic, topicId);
    if (strncmp(event->topic, fullTopic, strlen(fullTopic)) == 0) {
      found = true;
    } else {
      topicId++;
    }
  }
  if (!found) {
    topicId = JSON_BAD_TOPIC_ID;
  }
  return topicId;
}

char* getToken(char* buffer, const char* topic, unsigned char place)
{
  if (topic == NULL)
    return NULL;

  if (strlen(topic) >= 64)
    return NULL;

  char str[64];
  strcpy(str, topic);

  char *token = strtok(str, "/");
  int i = 0;
  while(i < place && token) {
    token = strtok(NULL, "/");
    i += 1;
  }
  if (!token)
    return NULL;

  return strcpy(buffer, token);
}

char* getActionType(char* actionType, const char* topic)
{
  return getToken(actionType, topic, 2);
}
char* getAction(char* action, const char* topic)
{
  return getToken(action, topic, 3);
}

char* getService(char* service, const char* topic)
{
  return getToken(service, topic, 4);
}

signed char getServiceId(const char* topic)
{
  signed char serviceId=-1;
  char buffer[8];
  char* s = getToken(buffer, topic, 5);
  if (s) {
    serviceId = atoi(s);
  }
  return serviceId;
}

char get_relay_json_value(const char* tag, esp_mqtt_event_handle_t event)
{
  char ret = JSON_BAD_RELAY_VALUE;
  char tmpBuf[MAX_MQTT_DATA_LEN_RELAY];
  memcpy(tmpBuf, event->data, event->data_len);
  tmpBuf[event->data_len] = 0;
  cJSON * root   = cJSON_Parse(tmpBuf);
  if (root)
    {
      cJSON * state = cJSON_GetObjectItem(root, tag);
      if (state)
        {
          char value = state->valueint;
          ret= value;
        }
      cJSON_Delete(root);
    }
  return ret;
}

bool handle_scheduler_mqtt_event(esp_mqtt_event_handle_t event)
{
#ifdef CONFIG_MQTT_SCHEDULERS
  unsigned char schedulerId = get_topic_id(event, MAX_SCHEDULER_NB, SCHEDULER_CFG_TOPIC);

  if (schedulerId != JSON_BAD_TOPIC_ID) {
    if (event->data_len >= MAX_MQTT_DATA_SCHEDULER) {
      ESP_LOGI(TAG, "unexpected scheduler cfg payload length");
      return true;
    }
    struct SchedulerCfgMessage s = {0, 0, 0, 0, {{0}}};
    s.schedulerId = schedulerId;

    char tmpBuf[MAX_MQTT_DATA_SCHEDULER];
    memcpy(tmpBuf, event->data, event->data_len);
    tmpBuf[event->data_len] = 0;
    cJSON * root   = cJSON_Parse(tmpBuf);
    if (root) {
      cJSON * timestamp = cJSON_GetObjectItem(root,"ts");
      if (timestamp) {
        s.timestamp = timestamp->valueint;
      }
      cJSON * actionId = cJSON_GetObjectItem(root,"aId");
      if (actionId) {
        s.actionId = actionId->valueint;
      }
      cJSON * actionState = cJSON_GetObjectItem(root,"aState");
      if (actionState) {
        s.actionState = actionState->valueint;
      }
      if (s.actionId == RELAY_ACTION) {
        cJSON * relayId = cJSON_GetObjectItem(root,"relayId");
        s.data.relayActionData.msgType = RELAY_STATE_CMD;
        if (relayId) {
          s.data.relayActionData.relayId = relayId->valueint;
        }
        cJSON * relayValue = cJSON_GetObjectItem(root,"relayValue");
        if (relayValue) {
          s.data.relayActionData.data.state = relayValue->valueint;
        }
      } /* else if(s.actionId == THERMOSTAT_ACTION) { */
      /*   cJSON * holdOffMode = cJSON_GetObjectItem(root,"holdOffMode"); */
      /*   s.data.thermostatActionData.holdOffMode = holdOffMode->valueint; */
      /* } */
      cJSON_Delete(root);

      if (xQueueSend(schedulerCfgQueue
                     ,( void * )&s
                     ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
        ESP_LOGE(TAG, "Cannot send to scheduleCfgQueue");
      }
    }
    return true;
  }
#endif // CONFIG_MQTT_SCHEDULERS
  return false;
}

#if CONFIG_MQTT_RELAYS_NB

void handle_relay_mqtt_timeout_cmd(char relayId, const char *payload)
{
  struct RelayMessage r;
  memset(&r, 0, sizeof(struct RelayMessage));
  r.msgType = RELAY_TIMEOUT_CMD;
  r.relayId = relayId;
  r.data.timeout = atoi(payload);

  if (xQueueSend( relayCmdQueue
                  ,( void * )&r
                  ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
    ESP_LOGE(TAG, "Cannot send to relayCmdQueue");
  }
}
void handle_thermostat_mqtt_mode_cmd(const char *payload)
{
  struct ThermostatMessage tm;
  memset(&tm, 0, sizeof(struct ThermostatMessage));
  tm.msgType = THERMOSTAT_CMD_MODE;

  if (strcmp(payload, "heat") == 0)
    tm.data.thermostatMode = TERMOSTAT_MODE_HEAT;
  else if (strcmp(payload, "off") == 0)
    tm.data.thermostatMode = TERMOSTAT_MODE_OFF;

  if (tm.data.thermostatMode == TERMOSTAT_MODE_UNSET) {
    ESP_LOGE(TAG, "wrong payload");
    return;
  }

  if (xQueueSend( thermostatQueue
                  ,( void * )&tm
                  ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
    ESP_LOGE(TAG, "Cannot send to thermostatQueue");
  }
}

void handle_thermostat_mqtt_temp_cmd(const char *payload)
{
  struct ThermostatMessage tm;
  memset(&tm, 0, sizeof(struct ThermostatMessage));
  tm.msgType = THERMOSTAT_CMD_TARGET_TEMPERATURE;
  // FIXME: add tests to convert from decimal integer to 0.1dC
  tm.data.targetTemperature = atoi(payload);

  if (xQueueSend( thermostatQueue
                  ,( void * )&tm
                  ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
    ESP_LOGE(TAG, "Cannot send to thermostatQueue");
  }
}

void handle_thermostat_mqtt_tolerance_cmd(const char *payload)
{
  struct ThermostatMessage tm;
  memset(&tm, 0, sizeof(struct ThermostatMessage));
  tm.msgType = THERMOSTAT_CMD_TOLERANCE;
  // FIXME: add tests to convert from decimal integer to 0.1dC
  tm.data.tolerance = atoi(payload);

  if (xQueueSend( thermostatQueue
                  ,( void * )&tm
                  ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
    ESP_LOGE(TAG, "Cannot send to thermostatQueue");
  }
}

void handle_water_thermostat_mqtt_temp_cmd(const char *payload)
{
  struct ThermostatMessage tm;
  memset(&tm, 0, sizeof(struct ThermostatMessage));
  tm.msgType = WATER_THERMOSTAT_CMD_TARGET_TEMPERATURE;
  // FIXME: add tests to convert from decimal integer to 0.1dC
  tm.data.targetTemperature = atoi(payload);

  if (xQueueSend( thermostatQueue
                  ,( void * )&tm
                  ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
    ESP_LOGE(TAG, "Cannot send to thermostatQueue");
  }
}

void handle_water_thermostat_mqtt_tolerance_cmd(const char *payload)
{
  struct ThermostatMessage tm;
  memset(&tm, 0, sizeof(struct ThermostatMessage));
  tm.msgType = WATER_THERMOSTAT_CMD_TOLERANCE;
  // FIXME: add tests to convert from decimal integer to 0.1dC
  tm.data.tolerance = atoi(payload);

  if (xQueueSend( thermostatQueue
                  ,( void * )&tm
                  ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
    ESP_LOGE(TAG, "Cannot send to thermostatQueue");
  }
}

void handle_water_thermostat_mqtt_mode_cmd(const char *payload)
{
  struct ThermostatMessage tm;
  memset(&tm, 0, sizeof(struct ThermostatMessage));
  tm.msgType = WATER_THERMOSTAT_CMD_MODE;

  if (strcmp(payload, "heat") == 0)
    tm.data.thermostatMode = TERMOSTAT_MODE_HEAT;
  else if (strcmp(payload, "off") == 0)
    tm.data.thermostatMode = TERMOSTAT_MODE_OFF;

  if (tm.data.thermostatMode == TERMOSTAT_MODE_UNSET) {
    ESP_LOGE(TAG, "wrong payload");
    return;
  }

  if (xQueueSend( thermostatQueue
                  ,( void * )&tm
                  ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
    ESP_LOGE(TAG, "Cannot send to thermostatQueue");
  }
}

void handle_relay_mqtt_state_cmd(char relayId, const char *payload)
{
  struct RelayMessage r;
  memset(&r, 0, sizeof(struct RelayMessage));
  r.msgType = RELAY_STATE_CMD;
  r.relayId = relayId;

  if (strcmp(payload, "ON") == 0)
    r.data.state = RELAY_STATE_ON;
  else if (strcmp(payload, "OFF") == 0)
    r.data.state = RELAY_STATE_OFF;

  if (r.data.state == RELAY_STATE_UNSET) {
    ESP_LOGE(TAG, "wrong payload");
    return;
  }

  if (xQueueSend( relayCmdQueue
                  ,( void * )&r
                  ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
    ESP_LOGE(TAG, "Cannot send to relayCmdQueue");
  }
}
void handle_thermostat_mqtt_cmd(const char* topic, const char* payload)
{
  char action[16];
  getAction(action, topic);
  if (strcmp(action, "mode") == 0) {
    handle_thermostat_mqtt_mode_cmd(payload);
    return;
  }
  if (strcmp(action, "temp") == 0) {
    handle_thermostat_mqtt_temp_cmd(payload);
    return;
  }
  if (strcmp(action, "tolerance") == 0) {
    handle_thermostat_mqtt_tolerance_cmd(payload);
    return;
  }
  ESP_LOGW(TAG, "unhlandled relay cmd: %s", action);
}

void handle_optimizer_thermostat_mqtt_cw_low_temp_cmd(const char *payload)
{
  struct ThermostatMessage tm;
  memset(&tm, 0, sizeof(struct ThermostatMessage));
  tm.msgType = OPTIMIZER_THERMOSTAT_CMD_CW_LOW_TEMP_CMD;
  // FIXME: add tests to convert from decimal integer to 0.1dC
  tm.data.targetTemperature = atoi(payload);

  if (xQueueSend( thermostatQueue
                  ,( void * )&tm
                  ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
    ESP_LOGE(TAG, "Cannot send to thermostatQueue");
  }
}

void handle_optimizer_thermostat_mqtt_min_cycle_duration_cmd(const char *payload)
{
  struct ThermostatMessage tm;
  memset(&tm, 0, sizeof(struct ThermostatMessage));
  tm.msgType = OPTIMIZER_THERMOSTAT_CMD_MIN_CYCLE_DURATION;
  // FIXME: add tests to convert from decimal integer to 0.1dC
  tm.data.min_cycle_duration = atoi(payload);

  if (xQueueSend( thermostatQueue
                  ,( void * )&tm
                  ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
    ESP_LOGE(TAG, "Cannot send to thermostatQueue");
  }
}

void handle_optimizer_thermostat_mqtt_cmd(const char* topic, const char* payload)
{
  char action[16];
  getAction(action, topic);
  if (strcmp(action, "min_cycle_duration") == 0) {
    handle_optimizer_thermostat_mqtt_min_cycle_duration_cmd(payload);
    return;
  }
  if (strcmp(action, "cw_low_temp") == 0) {
    handle_optimizer_thermostat_mqtt_cw_low_temp_cmd(payload);
    return;
  }

}
void handle_water_thermostat_mqtt_cmd(const char* topic, const char* payload)
{
  char action[16];
  getAction(action, topic);
  if (strcmp(action, "mode") == 0) {
    handle_water_thermostat_mqtt_mode_cmd(payload);
    return;
  }
  if (strcmp(action, "temp") == 0) {
    handle_water_thermostat_mqtt_temp_cmd(payload);
    return;
  }
  if (strcmp(action, "tolerance") == 0) {
    handle_water_thermostat_mqtt_tolerance_cmd(payload);
    return;
  }
  ESP_LOGW(TAG, "unhlandled relay cmd: %s", action);
}
void handle_relay_mqtt_cmd(const char* topic, const char* payload)
{
  signed char relayId = getServiceId(topic);
  if (relayId == -1) {
    ESP_LOGE(TAG, "wrong relay id");
    return;
  }

  char action[16];
  getAction(action, topic);
  if (strcmp(action, "state") == 0) {
    handle_relay_mqtt_state_cmd(relayId, payload);
    return;
  }
  if (strcmp(action, "timeout") == 0) {
    handle_relay_mqtt_timeout_cmd(relayId, payload);
    return;
  }
  ESP_LOGW(TAG, "unhlandled relay cmd: %s", action);


}
#endif //CONFIG_MQTT_RELAYS_NB


bool handle_ota_mqtt_event(esp_mqtt_event_handle_t event)
{
#ifdef CONFIG_MQTT_OTA
  if (strncmp(event->topic, OTA_TOPIC, strlen(OTA_TOPIC)) == 0) {
    struct OtaMessage o={"https://sw.iot.cipex.ro:8911/" CONFIG_MQTT_CLIENT_ID ".bin"};
    if (xQueueSend( otaQueue
                    ,( void * )&o
                    ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
      ESP_LOGE(TAG, "Cannot send to otaQueue");

    }
    return true;
  }
#endif //CONFIG_MQTT_OTA
  return false;
}

bool getTemperatureValue(short* value, const cJSON* root, const char* tag)
{
  cJSON * object = cJSON_GetObjectItem(root,tag);
  if (object) {
    *value = (short) (object->valuedouble * 10);
    ESP_LOGI(TAG, "%s: %d.%01d", tag, (*value)/10, (*value)%10);
    return true;
  }
  return false;
}

bool handle_cmd_thermostat_mqtt_event(esp_mqtt_event_handle_t event)
{
#ifdef CONFIG_MQTT_THERMOSTAT
  if (strncmp(event->topic, CMD_THERMOSTAT_TOPIC, strlen(CMD_THERMOSTAT_TOPIC)) == 0) {
    if (event->data_len >= MAX_MQTT_DATA_THERMOSTAT )
      {
        ESP_LOGI(TAG, "unexpected thermostat cmd payload length");
        return true;
      }
    char tmpBuf[MAX_MQTT_DATA_THERMOSTAT];
    memcpy(tmpBuf, event->data, event->data_len);
    tmpBuf[event->data_len] = 0;
    cJSON * root   = cJSON_Parse(tmpBuf);
    if (root) {
      struct ThermostatMessage tm;
      memset(&tm, 0, sizeof(struct ThermostatMessage));
      tm.msgType = THERMOSTAT_CMD_MSG;
      bool updated = false;

      cJSON * holdOffMode = cJSON_GetObjectItem(root,"holdOffMode");
      if (holdOffMode) {
        /* tm.data.cmdData.holdOffMode = holdOffMode->valueint; */
        /* ESP_LOGI(TAG, "holdOffMode: %d", tm.data.cmdData.holdOffMode); */
        updated = true;
      }

      if (updated) {
        if (xQueueSend( thermostatQueue
                        ,( void * )&tm
                        ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
          ESP_LOGE(TAG, "Cannot send to thermostatQueue");
        }
      }
      cJSON_Delete(root);
    }
    return true;
  }
#endif // CONFIG_MQTT_THERMOSTAT
  return false;
}

bool handle_cfg_thermostat_mqtt_event(esp_mqtt_event_handle_t event)
{
#ifdef CONFIG_MQTT_THERMOSTAT
  /* if (strncmp(event->topic, CFG_THERMOSTAT_TOPIC, strlen(CFG_THERMOSTAT_TOPIC)) == 0) { */
  /*   if (event->data_len >= MAX_MQTT_DATA_THERMOSTAT ) */
  /*     { */
  /*       ESP_LOGI(TAG, "unexpected thermostat cfg payload length"); */
  /*       return true; */
  /*     } */
  /*   char tmpBuf[MAX_MQTT_DATA_THERMOSTAT]; */
  /*   memcpy(tmpBuf, event->data, event->data_len); */
  /*   tmpBuf[event->data_len] = 0; */
  /*   cJSON * root   = cJSON_Parse(tmpBuf); */
  /*   if (root) { */
  /*     struct ThermostatMessage tm; */
  /*     memset(&tm, 0, sizeof(struct ThermostatMessage)); */
  /*     tm.msgType = THERMOSTAT_CFG_MSG; */
  /*     bool updated = false; */
  /*     if (getTemperatureValue(&tm.data.cfgData.circuitTargetTemperature, */
  /*                             root, "circuitTargetTemperature")) { */
  /*       updated = true; */
  /*     } */
  /*     if (getTemperatureValue(&tm.data.cfgData.waterTemperatureSensibility, */
  /*                             root, "waterTemperatureSensibility")) { */
  /*       updated = true; */
  /*     } */
  /*     if (getTemperatureValue(&tm.data.cfgData.room0TemperatureSensibility, */
  /*                             root, "room0TemperatureSensibility")) { */
  /*       updated = true; */
  /*     } */


  /*     if (updated) { */
  /*       if (xQueueSend( thermostatQueue */
  /*                       ,( void * )&tm */
  /*                       ,MQTT_QUEUE_TIMEOUT) != pdPASS) { */
  /*         ESP_LOGE(TAG, "Cannot send to thermostatQueue"); */
  /*       } */
  /*     } */
  /*     cJSON_Delete(root); */
  /*   } */
  /*   return true; */
  /* } */
#endif // CONFIG_MQTT_THERMOSTAT
  return false;
}

bool handle_room_sensors_mqtt_event(esp_mqtt_event_handle_t event)
{
#if CONFIG_MQTT_THERMOSTAT_ROOMS_SENSORS_NB > 0
  if (strncmp(event->topic, CONFIG_MQTT_THERMOSTAT_ROOM_0_SENSORS_TOPIC, strlen(CONFIG_MQTT_THERMOSTAT_ROOM_0_SENSORS_TOPIC)) == 0) {
    if (event->data_len >= MAX_MQTT_DATA_SENSORS )
      {
        ESP_LOGI(TAG, "unexpected room sensors cfg payload length");
        return true;
      }
    char tmpBuf[MAX_MQTT_DATA_SENSORS];
    memcpy(tmpBuf, event->data, event->data_len);
    tmpBuf[event->data_len] = 0;
    cJSON * root = cJSON_Parse(tmpBuf);
    if (root) {
      struct ThermostatRoomMessage t;
      if (getTemperatureValue(&t.temperature, root, "temperature")) {
        struct ThermostatMessage tm;
        memset(&tm, 0, sizeof(struct ThermostatMessage));
        tm.msgType = THERMOSTAT_ROOM_0_MSG;
        tm.data.roomData = t;
        if (xQueueSend( thermostatQueue
                        ,( void * )&tm
                        ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
          ESP_LOGE(TAG, "Cannot send to thermostatQueue");
        }
      }
      cJSON_Delete(root);
    }
    return true;
  }

#endif // MQTT_THERMOSTAT_ROOMS_SENSORS_NB > 0
  return false;
}
void dispatch_mqtt_event(const esp_mqtt_event_handle_t event)
{
  char actionType[16];
  if (getActionType(actionType, event->topic) && strcmp(actionType, "cmd") == 0) {
    char payload[16];
    //FIXME this check should be generic and 16 should get a define
    if (event->data_len > 16 - 1) { //including '\0'
      ESP_LOGE(TAG, "payload to big");
      return;
    }

    memcpy(payload, event->data, event->data_len);
    payload[event->data_len] = 0;

    char service[16];
    getService(service, event->topic);

#if CONFIG_MQTT_RELAYS_NB
    //FIXME should be RELAY_SERVICE
    if (strcmp(service, "relay") == 0) {
      handle_relay_mqtt_cmd(event->topic, payload);
      return;
    }
#endif //CONFIG_MQTT_RELAYS_NB
#ifdef CONFIG_MQTT_THERMOSTAT
    if (strcmp(service, "thermostat") == 0) {
      handle_thermostat_mqtt_cmd(event->topic, payload);
      return;
    }
    if (strcmp(service, "wthermostat") == 0) {
      handle_water_thermostat_mqtt_cmd(event->topic, payload);
      return;
    }
    if (strcmp(service, "othermostat") == 0) {
      handle_optimizer_thermostat_mqtt_cmd(event->topic, payload);
      return;
    }
#endif //CONFIG_MQTT_THERMOSTAT
    ESP_LOGI(TAG, "unhandled service %s", service);
  }

  //old code
  if (handle_scheduler_mqtt_event(event))
    return;
  if (handle_room_sensors_mqtt_event(event))
    return;
  if (handle_ota_mqtt_event(event))
    return;
  if (handle_cmd_thermostat_mqtt_event(event))
    return;
  if (handle_cfg_thermostat_mqtt_event(event))
    return;
}

void mqtt_publish_data(const char * topic,
                       const char * data,
                       int qos, int retain)
{
  if (xEventGroupGetBits(mqtt_event_group) & MQTT_INIT_FINISHED_BIT) {
    xEventGroupClearBits(mqtt_event_group, MQTT_PUBLISHED_BIT);
    int msg_id = esp_mqtt_client_publish(client, topic, data, strlen(data), qos, retain);
    if (qos == QOS_0) {
      ESP_LOGI(TAG, "published qos0 data");
    } else if (msg_id > 0) {
      ESP_LOGI(TAG, "sent publish data successful, msg_id=%d", msg_id);
      EventBits_t bits = xEventGroupWaitBits(mqtt_event_group, MQTT_PUBLISHED_BIT, false, true, MQTT_FLAG_TIMEOUT);
      if (bits & MQTT_PUBLISHED_BIT) {
        ESP_LOGI(TAG, "publish ack received, msg_id=%d", msg_id);
      } else {
        ESP_LOGW(TAG, "publish ack not received, msg_id=%d", msg_id);
      }
    } else {
      ESP_LOGW(TAG, "failed to publish qos1, msg_id=%d", msg_id);
    }
  }
}

void publish_online_availability()
{
  char* data = "online";
  mqtt_publish_data(available_topic, data, QOS_1, RETAIN);
}


void publish_fw_version()
{
  const char * topic = CONFIG_MQTT_DEVICE_TYPE "/" CONFIG_MQTT_CLIENT_ID "/evt/version/firmware";

  char* data = FW_VERSION;
  mqtt_publish_data(topic, data, QOS_1, RETAIN);
}


static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
  switch (event->event_id) {
  case MQTT_EVENT_CONNECTED:
    xEventGroupSetBits(mqtt_event_group, MQTT_CONNECTED_BIT);
    void * unused;
    if (xQueueSend( mqttQueue
                    ,( void * )&unused
                    ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
      ESP_LOGE(TAG, "Cannot send to mqttQueue");
    }
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
    mqtt_reconnect_counter=0;
    break;
  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    xEventGroupClearBits(mqtt_event_group, MQTT_CONNECTED_BIT | MQTT_SUBSCRIBED_BIT | MQTT_PUBLISHED_BIT | MQTT_INIT_FINISHED_BIT);
    mqtt_reconnect_counter += 1; //one reconnect each 10 seconds
    ESP_LOGI(TAG, "mqtt_reconnect_counter: %d", mqtt_reconnect_counter);
    if (mqtt_reconnect_counter  > (6 * 5)) //5 mins, force wifi reconnect
      {
        esp_wifi_stop();
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        esp_wifi_start();
        mqtt_reconnect_counter = 0;
      }
    break;

  case MQTT_EVENT_SUBSCRIBED:
    xEventGroupSetBits(mqtt_event_group, MQTT_SUBSCRIBED_BIT);
    ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_UNSUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_PUBLISHED:
    xEventGroupSetBits(mqtt_event_group, MQTT_PUBLISHED_BIT);
    ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_DATA:
    ESP_LOGI(TAG, "MQTT_EVENT_DATA");
    ESP_LOGI(TAG, "TOPIC=%.*s", event->topic_len, event->topic);
    ESP_LOGI(TAG, "DATA=%.*s", event->data_len, event->data);
    dispatch_mqtt_event(event);
    break;
  case MQTT_EVENT_ERROR:
    ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    break;
  }
  return ESP_OK;
}

static void mqtt_subscribe(esp_mqtt_client_handle_t client)
{
  int msg_id;

  for (int i = 0; i < NB_SUBSCRIPTIONS; i++) {
    xEventGroupClearBits(mqtt_event_group, MQTT_SUBSCRIBED_BIT);
    msg_id = esp_mqtt_client_subscribe(client, SUBSCRIPTIONS[i], 1);
    if (msg_id > 0) {
      ESP_LOGI(TAG, "sent subscribe %s successful, msg_id=%d", SUBSCRIPTIONS[i], msg_id);
      EventBits_t bits = xEventGroupWaitBits(mqtt_event_group, MQTT_SUBSCRIBED_BIT, false, true, MQTT_FLAG_TIMEOUT);
      if (bits & MQTT_SUBSCRIBED_BIT) {
        ESP_LOGI(TAG, "subscribe ack received, msg_id=%d", msg_id);
      } else {
        ESP_LOGW(TAG, "subscribe ack not received, msg_id=%d", msg_id);
      }
    } else {
      ESP_LOGW(TAG, "failed to subscribe %s, msg_id=%d", SUBSCRIPTIONS[i], msg_id);
    }
  }
}

void mqtt_init_and_start()
{
  const char * lwtmsg = "offline";
  const esp_mqtt_client_config_t mqtt_cfg = {
    .uri = "mqtts://" CONFIG_MQTT_USERNAME ":" CONFIG_MQTT_PASSWORD "@" CONFIG_MQTT_SERVER ":" CONFIG_MQTT_PORT,
    .event_handle = mqtt_event_handler,
    .cert_pem = (const char *)mqtt_iot_cipex_ro_pem_start,
    .client_id = CONFIG_MQTT_CLIENT_ID,
    .lwt_topic = available_topic,
    .lwt_msg = lwtmsg,
    .lwt_qos = 1,
    .lwt_retain = 1,
    .lwt_msg_len = strlen(lwtmsg),
    .keepalive = MQTT_TIMEOUT
  };

  ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
  client = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_start(client);
  xEventGroupWaitBits(mqtt_event_group, MQTT_CONNECTED_BIT, false, true, portMAX_DELAY);
}

void handle_mqtt_sub_pub(void* pvParameters)
{
  void * unused;
  while(1) {
    if( xQueueReceive( mqttQueue, &unused , portMAX_DELAY) )
      {
        xEventGroupClearBits(mqtt_event_group, MQTT_INIT_FINISHED_BIT);
        mqtt_subscribe(client);
        xEventGroupSetBits(mqtt_event_group, MQTT_INIT_FINISHED_BIT);
        publish_online_availability();
        publish_fw_version();
#if CONFIG_MQTT_RELAYS_NB
        publish_all_relays_status();
        publish_all_relays_timeout();
#endif//CONFIG_MQTT_RELAYS_NB
#ifdef CONFIG_MQTT_THERMOSTAT
        publish_thermostat_data();
#endif // CONFIG_MQTT_THERMOSTAT
#ifdef CONFIG_MQTT_OTA
        publish_ota_data(OTA_READY);
#endif //CONFIG_MQTT_OTA
#ifdef CONFIG_MQTT_SENSOR
        publish_sensors_data();
#endif//
      }
  }
}


