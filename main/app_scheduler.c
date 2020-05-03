#include "esp_system.h"
#ifdef CONFIG_MQTT_SCHEDULERS

#include <time.h>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "lwip/apps/sntp.h"

#include "string.h"

#include "app_scheduler.h"
#include "app_main.h"
#include "app_nvs.h"
#include "app_mqtt.h"
#include "app_mqtt_util.h"

static const char *TAG = "SCHEDULER";
extern QueueHandle_t schedulerQueue;
extern QueueHandle_t relayQueue;

struct Scheduler_AIO schedulerCfg[MAX_SCHEDULER_NB];
int lastRunMinutes=-1;
void update_time_from_ntp()
{
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;

    // wait for time to be set
    while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
      ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      time(&now);
      localtime_r(&now, &timeinfo);
    }

    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "Current time after ntp update: %s", strftime_buf);
    
    lastRunMinutes = now/60;
    ESP_LOGI(TAG, "updating lastRunMinutes after ntp set: %d", lastRunMinutes);
}

void vSchedulerCallback( TimerHandle_t xTimer )
{

  ESP_LOGI(TAG, "timer scheduler expired, checking scheduled actions");


  //trigerring
  struct SchedulerMessage s;
  s.msgType = SCHEDULER_LIFE_TICK;
  if (xQueueSend(schedulerQueue
                 ,( void * )&s
                 ,SCHEDULE_QUEUE_TIMEOUT) != pdPASS) {
    ESP_LOGE(TAG, "Cannot send to scheduleQueue");
  }

}


void start_scheduler_timer()
{
  ESP_LOGI(TAG, "Initializing SNTP");
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, "pool.ntp.org");
  sntp_init();

  update_time_from_ntp();

  TimerHandle_t th =
    xTimerCreate( "schedulerTimer",           /* Text name. */
                  pdMS_TO_TICKS(60000),  /* Period. */
                  pdTRUE,                /* Autoreload. */
                  (void *)0,                  /* No ID. */
                  vSchedulerCallback );  /* Callback function. */
  if( th != NULL ) {
    ESP_LOGI(TAG, "timer is created");
    if (xTimerStart(th, portMAX_DELAY) != pdFALSE){
      ESP_LOGI(TAG, "timer is active");
    }
  }
}

void publish_scheduler_topic_evt(int id)
{
  const char * scheduler_topic = CONFIG_MQTT_DEVICE_TYPE "/" CONFIG_MQTT_CLIENT_ID "/evt/topic/scheduler";

  char topic[MQTT_MAX_TOPIC_LEN];
  memset(topic,0,MQTT_MAX_TOPIC_LEN);
  sprintf(topic, "%s/%d", scheduler_topic, id);

  mqtt_publish_data(topic, schedulerCfg[id].topic, QOS_1, RETAIN);
}

void publish_scheduler_payload_evt(int id)
{
  const char * scheduler_topic = CONFIG_MQTT_DEVICE_TYPE "/" CONFIG_MQTT_CLIENT_ID "/evt/payload/scheduler";

  char topic[MQTT_MAX_TOPIC_LEN];
  memset(topic,0,MQTT_MAX_TOPIC_LEN);
  sprintf(topic, "%s/%d", scheduler_topic, id);

  mqtt_publish_data(topic, schedulerCfg[id].payload, QOS_1, RETAIN);
}

void publish_scheduler_starttime_evt(int id)
{
  const char * scheduler_topic = CONFIG_MQTT_DEVICE_TYPE "/" CONFIG_MQTT_CLIENT_ID "/evt/starttime/scheduler";

  char topic[MQTT_MAX_TOPIC_LEN];
  memset(topic,0,MQTT_MAX_TOPIC_LEN);
  sprintf(topic, "%s/%d", scheduler_topic, id);

  mqtt_publish_data(topic, schedulerCfg[id].starttime, QOS_1, RETAIN);
}

void publish_scheduler_repeat_evt(int id)
{
  const char * scheduler_topic = CONFIG_MQTT_DEVICE_TYPE "/" CONFIG_MQTT_CLIENT_ID "/evt/repeat/scheduler";

  char topic[MQTT_MAX_TOPIC_LEN];
  memset(topic,0,MQTT_MAX_TOPIC_LEN);
  sprintf(topic, "%s/%d", scheduler_topic, id);

  mqtt_publish_data(topic, schedulerCfg[id].repeat, QOS_1, RETAIN);
}

void publish_scheduler_status_evt(int id)
{
  const char * scheduler_topic = CONFIG_MQTT_DEVICE_TYPE "/" CONFIG_MQTT_CLIENT_ID "/evt/status/scheduler";

  char data[16];
  memset(data,0,16);
  sprintf(data, "%s", schedulerCfg[id].status == SCHEDULER_STATUS_ON ? "ON" : "OFF");
  
  char topic[MQTT_MAX_TOPIC_LEN];
  memset(topic,0,MQTT_MAX_TOPIC_LEN);
  sprintf(topic, "%s/%d", scheduler_topic, id);

  mqtt_publish_data(topic, data, QOS_1, RETAIN);
}

void publish_scheduler_aio_evt(int id)
{
  const char * scheduler_topic = CONFIG_MQTT_DEVICE_TYPE "/" CONFIG_MQTT_CLIENT_ID "/evt/aio/scheduler";

  char data[MAX_MQTT_SCHEDULER_AIO_PAYLOAD_SIZE];
  memset(data,0,MAX_MQTT_SCHEDULER_AIO_PAYLOAD_SIZE);
  sprintf(data, "{\"topic\":\"%s\",\"payload\":\"%s\",\"starttime\":\"%s\",\"repeat\":\"%s\",\"status\":\"%s\"}", schedulerCfg[id].topic, schedulerCfg[id].payload,
          schedulerCfg[id].starttime, schedulerCfg[id].repeat,
          schedulerCfg[id].status == SCHEDULER_STATUS_ON ? "ON" : "OFF");
  
  char topic[MQTT_MAX_TOPIC_LEN];
  memset(topic,0,MQTT_MAX_TOPIC_LEN);
  sprintf(topic, "%s/%d", scheduler_topic, id);

  mqtt_publish_data(topic, data, QOS_1, RETAIN);
}

void publish_scheduler_data(void)
{
  for(int id = 0; id < MAX_SCHEDULER_NB; id++) {
    publish_scheduler_aio_evt(id);
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void log_scheduler(int id, const struct Scheduler_AIO *msg)
{
  ESP_LOGI(TAG, "scheduler id: %d: \n\ttopic: %s\n\tpayload: %s\n\tstarttime: %s\n\trepeat: %s\n\tstatus: %s",
           id,
           msg->topic,
           msg->payload,
           msg->starttime,
           msg->repeat,
           msg->status == SCHEDULER_STATUS_ON ? "ON" : "OFF");
}

void trigger_scheduler(int id, const struct Scheduler_AIO *msg, int nowMinutes)
{
  struct tm tm = { 0 };

  if (strptime(msg->starttime, "%Y-%m-%d %H:%M", &tm) == NULL) {
    ESP_LOGI(TAG, "Cannot parse starttime");
    return;
  }
  
  time_t schedulerTime = mktime(&tm);
  int schedulerMinutes = schedulerTime / 60;
  ESP_LOGI(TAG, "schedulerMinutes: %d", schedulerMinutes);
  if (schedulerMinutes > lastRunMinutes && schedulerMinutes <= nowMinutes ) {
    char service[16];
    getService(service, msg->topic);
    if (strcmp(service, "relay") == 0) {
      handle_relay_mqtt_cmd(msg->topic, msg->payload);
      return;
    }
  }
}


void trigger_schedulers(int nowMinutes)
{
  for (int i = 0; i < MAX_SCHEDULER_NB; ++i) {
    log_scheduler(i, &schedulerCfg[i]);
    trigger_scheduler(i, &schedulerCfg[i], nowMinutes);
  }
}

void read_nvs_scheduler_data()
{
  esp_err_t err;

  char nvs_tag[16];
  size_t length;
  
  for(int id = 0; id < MAX_SCHEDULER_NB; id++) {
    sprintf(nvs_tag, "schTopic%d", id);
    length = sizeof(schedulerCfg[id].topic);
    err=read_nvs_str(nvs_tag, schedulerCfg[id].topic, &length);
    if (length > sizeof(schedulerCfg[id].topic))
      ESP_LOGE(TAG, "%s value in nvs is too big", nvs_tag);
    ESP_ERROR_CHECK( err );
  }
  
  for(int id = 0; id < MAX_SCHEDULER_NB; id++) {
    sprintf(nvs_tag, "schPayload%d", id);
    length = sizeof(schedulerCfg[id].payload);
    err=read_nvs_str(nvs_tag, schedulerCfg[id].payload, &length);
    if (length > sizeof(schedulerCfg[id].payload))
      ESP_LOGE(TAG, "%s value in nvs is too big", nvs_tag);
    ESP_ERROR_CHECK( err );
  }

  for(int id = 0; id < MAX_SCHEDULER_NB; id++) {
    sprintf(nvs_tag, "schStartTime%d", id);
    length = sizeof(schedulerCfg[id].starttime);
    err=read_nvs_str(nvs_tag, schedulerCfg[id].starttime, &length);
    if (length > sizeof(schedulerCfg[id].starttime))
      ESP_LOGE(TAG, "%s value in nvs is too big", nvs_tag);
    ESP_ERROR_CHECK( err );
  }

  for(int id = 0; id < MAX_SCHEDULER_NB; id++) {
    sprintf(nvs_tag, "schRepeat%d", id);
    length = sizeof(schedulerCfg[id].repeat);
    err=read_nvs_str(nvs_tag, schedulerCfg[id].repeat, &length);
    if (length > sizeof(schedulerCfg[id].repeat))
      ESP_LOGE(TAG, "%s value in nvs is too big", nvs_tag);
    ESP_ERROR_CHECK( err );
  }

  for(int id = 0; id < MAX_SCHEDULER_NB; id++) {
    sprintf(nvs_tag, "schStatus%d", id);
    err=read_nvs_short(nvs_tag, (short*) &schedulerCfg[id].status);
    ESP_ERROR_CHECK( err );
  }
}

void handle_scheduler(void* pvParameters)
{
  ESP_LOGI(TAG, "handle_scheduler task started");

  start_scheduler_timer();
  struct SchedulerMessage sm;
  char nvs_tag[16];
  while(1) {
    if( xQueueReceive(schedulerQueue, &sm, portMAX_DELAY)) {

      if (sm.msgType == SCHEDULER_LIFE_TICK) {
        time_t now = 0;
        struct tm timeinfo = { 0 };

        time(&now);
        localtime_r(&now, &timeinfo);

        char strftime_buf[64];
        strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
        ESP_LOGI(TAG, "Current time is: %s", strftime_buf);

        int nowMinutes = now/60;
        ESP_LOGI(TAG, "nowMinutes: %d", nowMinutes);
        ESP_LOGI(TAG, "lastRunMinutes: %d", lastRunMinutes);
        trigger_schedulers(nowMinutes);
        lastRunMinutes = nowMinutes;
      }

      if (sm.msgType == SCHEDULER_CMD_TOPIC) {
        ESP_LOGI(TAG, "Update topic for scheduler %d", sm.schedulerId);
        if (strcmp(schedulerCfg[sm.schedulerId].topic, sm.data.topic) != 0) {
          strcpy(schedulerCfg[sm.schedulerId].topic, sm.data.topic);
          sprintf(nvs_tag, "schTopic%d", sm.schedulerId);
          esp_err_t err = write_nvs_str(nvs_tag,
                                             schedulerCfg[sm.schedulerId].topic);
          ESP_ERROR_CHECK( err );
        }
        publish_scheduler_topic_evt(sm.schedulerId);
      }

      if (sm.msgType == SCHEDULER_CMD_PAYLOAD) {
        ESP_LOGI(TAG, "Update payload for scheduler %d", sm.schedulerId);
        if (strcmp(schedulerCfg[sm.schedulerId].payload, sm.data.payload) != 0) {
          strcpy(schedulerCfg[sm.schedulerId].payload, sm.data.payload);
          sprintf(nvs_tag, "schPayload%d", sm.schedulerId);
          esp_err_t err = write_nvs_str(nvs_tag,
                                        schedulerCfg[sm.schedulerId].payload);
          ESP_ERROR_CHECK( err );
        }
        publish_scheduler_payload_evt(sm.schedulerId);
      }

      if (sm.msgType == SCHEDULER_CMD_STARTTIME) {
        ESP_LOGI(TAG, "Update starttime for scheduler %d", sm.schedulerId);
        if (strcmp(schedulerCfg[sm.schedulerId].starttime, sm.data.starttime) != 0) {
          strcpy(schedulerCfg[sm.schedulerId].starttime, sm.data.starttime);
          sprintf(nvs_tag, "schStartTime%d", sm.schedulerId);
          esp_err_t err = write_nvs_str(nvs_tag,
                                        schedulerCfg[sm.schedulerId].starttime);
          ESP_ERROR_CHECK( err );
        }
        publish_scheduler_starttime_evt(sm.schedulerId);
      }

      if (sm.msgType == SCHEDULER_CMD_REPEAT) {
        ESP_LOGI(TAG, "Update repeat for scheduler %d", sm.schedulerId);
        if (strcmp(schedulerCfg[sm.schedulerId].repeat, sm.data.repeat) != 0) {
          strcpy(schedulerCfg[sm.schedulerId].repeat, sm.data.repeat);
          sprintf(nvs_tag, "schRepeat%d", sm.schedulerId);
          esp_err_t err = write_nvs_str(nvs_tag,
                                        schedulerCfg[sm.schedulerId].repeat);
          ESP_ERROR_CHECK( err );
        }
        publish_scheduler_repeat_evt(sm.schedulerId);
      }

      if (sm.msgType == SCHEDULER_CMD_STATUS) {
        ESP_LOGI(TAG, "Update status for scheduler %d", sm.schedulerId);
        if (schedulerCfg[sm.schedulerId].status != sm.data.status) {
          schedulerCfg[sm.schedulerId].status = sm.data.status;
          sprintf(nvs_tag, "schStatus%d", sm.schedulerId);
          esp_err_t err = write_nvs_short(nvs_tag,
                                          schedulerCfg[sm.schedulerId].status);
          ESP_ERROR_CHECK( err );
        }
        publish_scheduler_status_evt(sm.schedulerId);
      }

      if (sm.msgType == SCHEDULER_CMD_AIO) {
        ESP_LOGI(TAG, "Update aio for scheduler %d", sm.schedulerId);
        if (strcmp(schedulerCfg[sm.schedulerId].topic, sm.data.aio.topic) != 0) {
          ESP_LOGI(TAG, "Update topic for scheduler %d", sm.schedulerId);
          strcpy(schedulerCfg[sm.schedulerId].topic, sm.data.aio.topic);
          sprintf(nvs_tag, "schTopic%d", sm.schedulerId);
          esp_err_t err = write_nvs_str(nvs_tag,
                                             schedulerCfg[sm.schedulerId].topic);
          ESP_ERROR_CHECK( err );
        }

        if (strcmp(schedulerCfg[sm.schedulerId].payload, sm.data.aio.payload) != 0) {
          ESP_LOGI(TAG, "Update payload for scheduler %d", sm.schedulerId);
          strcpy(schedulerCfg[sm.schedulerId].payload, sm.data.aio.payload);
          sprintf(nvs_tag, "schPayload%d", sm.schedulerId);
          esp_err_t err = write_nvs_str(nvs_tag,
                                        schedulerCfg[sm.schedulerId].payload);
          ESP_ERROR_CHECK( err );
        }

        if (strcmp(schedulerCfg[sm.schedulerId].starttime, sm.data.aio.starttime) != 0) {
          ESP_LOGI(TAG, "Update starttime for scheduler %d", sm.schedulerId);
          strcpy(schedulerCfg[sm.schedulerId].starttime, sm.data.aio.starttime);
          sprintf(nvs_tag, "schStartTime%d", sm.schedulerId);
          esp_err_t err = write_nvs_str(nvs_tag,
                                        schedulerCfg[sm.schedulerId].starttime);
          ESP_ERROR_CHECK( err );
        }

        if (strcmp(schedulerCfg[sm.schedulerId].repeat, sm.data.aio.repeat) != 0) {
          ESP_LOGI(TAG, "Update repeat for scheduler %d", sm.schedulerId);
          strcpy(schedulerCfg[sm.schedulerId].repeat, sm.data.aio.repeat);
          sprintf(nvs_tag, "schRepeat%d", sm.schedulerId);
          esp_err_t err = write_nvs_str(nvs_tag,
                                        schedulerCfg[sm.schedulerId].repeat);
          ESP_ERROR_CHECK( err );
        }

        if (schedulerCfg[sm.schedulerId].status != sm.data.aio.status) {
          ESP_LOGI(TAG, "Update status for scheduler %d", sm.schedulerId);
          schedulerCfg[sm.schedulerId].status = sm.data.aio.status;
          sprintf(nvs_tag, "schStatus%d", sm.schedulerId);
          esp_err_t err = write_nvs_short(nvs_tag,
                                          schedulerCfg[sm.schedulerId].status);
          ESP_ERROR_CHECK( err );
        }
        publish_scheduler_aio_evt(sm.schedulerId);
      }

      
      
      

      /*      if (tempSchedulerCfg.actionId == TRIGGER_ACTION) { */
/*         struct tm timeinfo = { 0 }; */
/*         localtime_r(&tempSchedulerCfg.data.triggerActionData.now, &timeinfo); */

/*         char strftime_buf[64]; */
/*         strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo); */
/*         ESP_LOGI(TAG, "Current time is: %s", strftime_buf); */

/*         int nowMinutes =  tempSchedulerCfg.data.triggerActionData.now/ 60; */
/*         ESP_LOGI(TAG, "nowMinutes: %d", nowMinutes); */
/*         handle_action_trigger(schedulerCfg, nowMinutes); */
/*       } else if (tempSchedulerCfg.actionId == ADD_RELAY_ACTION) { */
/*         if (tempSchedulerCfg.schedulerId < MAX_SCHEDULER_NB) { */
/*           ESP_LOGI(TAG, "Updating schedulerId: %d", */
/*                    tempSchedulerCfg.schedulerId); */
/*           schedulerCfg[tempSchedulerCfg.schedulerId] = tempSchedulerCfg; */
/*         } else { */
/*               ESP_LOGE(TAG, "Wrong schedulerId: %d", */
/*                        tempSchedulerCfg.schedulerId); */
/*         } */
/*       } else { */
/*         ESP_LOGE(TAG, "Unknown actionId: %d", */
/*                  tempSchedulerCfg.actionId); */
/*       } */
/*     } */
    }
  }
}
#endif //CONFIG_MQTT_SCHEDULERS
