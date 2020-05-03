#include "esp_system.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "cJSON.h"

#include "mqtt_client.h"

#include "app_main.h"

#include "app_sensors.h"
#include "app_mqtt.h"

#include "app_mqtt_util.h"

#ifdef CONFIG_MQTT_SCHEDULERS

#include "app_scheduler.h"

static const char *TAG = "MQTTS_MQTT_RELAY";
extern QueueHandle_t schedulerQueue;

void handle_scheduler_mqtt_topic_cmd(signed char schedulerId, const char *payload)
{
  struct SchedulerMessage sm;
  memset(&sm, 0, sizeof(struct SchedulerMessage));
  sm.msgType = SCHEDULER_CMD_TOPIC;
  sm.schedulerId = schedulerId;
  strcpy(sm.data.topic, payload);
  if (xQueueSend( schedulerQueue
                  ,( void * )&sm
                  ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
    ESP_LOGE(TAG, "Cannot send to schedulerQueue");
  }
}

void handle_scheduler_mqtt_payload_cmd(signed char schedulerId, const char *payload)
{
  struct SchedulerMessage sm;
  memset(&sm, 0, sizeof(struct SchedulerMessage));
  sm.msgType = SCHEDULER_CMD_PAYLOAD;
  sm.schedulerId = schedulerId;
  strcpy(sm.data.payload, payload);
  if (xQueueSend( schedulerQueue
                  ,( void * )&sm
                  ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
    ESP_LOGE(TAG, "Cannot send to schedulerQueue");
  }
}


void handle_scheduler_mqtt_starttime_cmd(signed char schedulerId, const char *payload)
{
  struct SchedulerMessage sm;
  memset(&sm, 0, sizeof(struct SchedulerMessage));
  sm.msgType = SCHEDULER_CMD_STARTTIME;
  sm.schedulerId = schedulerId;
  strcpy(sm.data.starttime, payload);
  if (xQueueSend( schedulerQueue
                  ,( void * )&sm
                  ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
    ESP_LOGE(TAG, "Cannot send to schedulerQueue");
  }
}

void handle_scheduler_mqtt_repeat_cmd(signed char schedulerId, const char *payload)
{
  struct SchedulerMessage sm;
  memset(&sm, 0, sizeof(struct SchedulerMessage));
  sm.msgType = SCHEDULER_CMD_REPEAT;
  sm.schedulerId = schedulerId;
  strcpy(sm.data.repeat, payload);
  if (xQueueSend( schedulerQueue
                  ,( void * )&sm
                  ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
    ESP_LOGE(TAG, "Cannot send to schedulerQueue");
  }
}

void handle_scheduler_mqtt_status_cmd(signed char schedulerId, const char *payload)
{
  struct SchedulerMessage sm;
  memset(&sm, 0, sizeof(struct SchedulerMessage));
  sm.msgType = SCHEDULER_CMD_STATUS;
  sm.schedulerId = schedulerId;

  if (strcmp(payload, "ON") == 0)
    sm.data.status = SCHEDULER_STATUS_ON;
  else if (strcmp(payload, "OFF") == 0)
    sm.data.status = SCHEDULER_STATUS_OFF;

  if (xQueueSend( schedulerQueue
                  ,( void * )&sm
                  ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
    ESP_LOGE(TAG, "Cannot send to schedulerQueue");
  }
}

void handle_scheduler_mqtt_aio_cmd(signed char schedulerId, const char *msgpayload)
{
  struct SchedulerMessage sm;
  memset(&sm, 0, sizeof(struct SchedulerMessage));
  sm.msgType = SCHEDULER_CMD_AIO;
  sm.schedulerId = schedulerId;
  
  cJSON * root   = cJSON_Parse(msgpayload);
  if (root) {
    cJSON * topic = cJSON_GetObjectItem(root,"topic");
    if (topic) {
      strcpy(sm.data.aio.topic, topic->valuestring);
    }
    cJSON * payload = cJSON_GetObjectItem(root,"payload");
    if (payload) {
      strcpy(sm.data.aio.payload, payload->valuestring);
    }
    cJSON * starttime = cJSON_GetObjectItem(root,"starttime");
    if (starttime) {
      strcpy(sm.data.aio.starttime, starttime->valuestring);
    }
    cJSON * repeat = cJSON_GetObjectItem(root,"repeat");
    if (repeat) {
      strcpy(sm.data.aio.repeat, repeat->valuestring);
    }
    cJSON * status = cJSON_GetObjectItem(root,"status");
    if (status) {
      if (strcmp(status->valuestring, "ON") == 0)
        sm.data.status = SCHEDULER_STATUS_ON;
      else if (strcmp(status->valuestring, "OFF") == 0)
        sm.data.status = SCHEDULER_STATUS_OFF;
    }
    cJSON_Delete(root);
  }  
  if (xQueueSend( schedulerQueue
                  ,( void * )&sm
                  ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
    ESP_LOGE(TAG, "Cannot send to schedulerQueue");
  }
}

void handle_scheduler_mqtt_cmd(const char* topic, const char* payload)
{
  char action[16];
  getAction(action, topic);

  signed char schedulerId = getServiceId(topic);
  if (schedulerId != -1) {
    if (strcmp(action, "topic") == 0) {
      handle_scheduler_mqtt_topic_cmd(schedulerId, payload);
      return;
    }
    if (strcmp(action, "payload") == 0) {
      handle_scheduler_mqtt_payload_cmd(schedulerId, payload);
      return;
    }
    if (strcmp(action, "starttime") == 0) {
      handle_scheduler_mqtt_starttime_cmd(schedulerId, payload);
      return;
    }
    if (strcmp(action, "repeat") == 0) {
      handle_scheduler_mqtt_repeat_cmd(schedulerId, payload);
      return;
    }
    if (strcmp(action, "status") == 0) {
      handle_scheduler_mqtt_status_cmd(schedulerId, payload);
      return;
    }
    if (strcmp(action, "aio") == 0) {
      handle_scheduler_mqtt_status_cmd(schedulerId, payload);
      return;
    }

    ESP_LOGW(TAG, "unhandled scheduler action: %s", action);
    return;
  }
  ESP_LOGW(TAG, "unhandled scheduler id: %d", schedulerId);
}

#endif // CONFIG_MQTT_SCHEDULERS


/* bool handle_scheduler_mqtt_event(esp_mqtt_event_handle_t event) */
/* { */
/* #ifdef CONFIG_MQTT_SCHEDULERS */
/*   unsigned char schedulerId = get_topic_id(event, MAX_SCHEDULER_NB, SCHEDULER_CFG_TOPIC); */

/*   if (schedulerId != JSON_BAD_TOPIC_ID) { */
/*     if (event->data_len >= MAX_MQTT_DATA_SCHEDULER) { */
/*       ESP_LOGI(TAG, "unexpected scheduler cfg payload length"); */
/*       return true; */
/*     } */
/*     struct SchedulerCfgMessage s = {0, 0, 0, 0, {{0}}}; */
/*     s.schedulerId = schedulerId; */

/*     char tmpBuf[MAX_MQTT_DATA_SCHEDULER]; */
/*     memcpy(tmpBuf, event->data, event->data_len); */
/*     tmpBuf[event->data_len] = 0; */
/*     cJSON * root   = cJSON_Parse(tmpBuf); */
/*     if (root) { */
/*       cJSON * timestamp = cJSON_GetObjectItem(root,"ts"); */
/*       if (timestamp) { */
/*         s.timestamp = timestamp->valueint; */
/*       } */
/*       cJSON * actionId = cJSON_GetObjectItem(root,"aId"); */
/*       if (actionId) { */
/*         s.actionId = actionId->valueint; */
/*       } */
/*       cJSON * actionState = cJSON_GetObjectItem(root,"aState"); */
/*       if (actionState) { */
/*         s.actionState = actionState->valueint; */
/*       } */
/*       cJSON * data = cJSON_GetObjectItem(root,"data"); */
/*       if (data) { */
/*         if (s.actionId == ADD_RELAY_ACTION) { */
/*           cJSON * schedulerId = cJSON_GetObjectItem(data,"schedulerId"); */
/*           if (schedulerId) { */
/*             s.data.relayActionData.schedulerId = schedulerId->valueint; */
/*           } */
/*           cJSON * relayValue = cJSON_GetObjectItem(data,"relayValue"); */
/*           if (relayValue) { */
/*             s.data.relayActionData.data = relayValue->valueint; */
/*           } */
/*         } */
/*       } */
/*       cJSON_Delete(root); */

/*       if (xQueueSend(schedulerCfgQueue */
/*                      ,( void * )&s */
/*                      ,MQTT_QUEUE_TIMEOUT) != pdPASS) { */
/*         ESP_LOGE(TAG, "Cannot send to scheduleCfgQueue"); */
/*       } */
/*     } */
/*     return true; */
/*   } */
/* #endif // CONFIG_MQTT_SCHEDULERS */
/*   return false; */
/* } */

