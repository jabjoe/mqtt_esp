#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "mqtt_client.h"

#include "app_main.h"

#include "app_sensors.h"
#include "app_mqtt.h"
#include "app_mqtt_util.h"

#ifdef CONFIG_MQTT_SCHEDULERS
  #include "app_scheduler.h"

  #define SCHEDULER_TOPICS_NB 1
  #define SCHEDULER_CFG_TOPIC CONFIG_MQTT_DEVICE_TYPE"/"CONFIG_MQTT_CLIENT_ID"/cmd/+/scheduler/"
#else // CONFIG_MQTT_SCHEDULERS
  #define SCHEDULER_TOPICS_NB 0
#endif // CONFIG_MQTT_SCHEDULERS


#if CONFIG_MQTT_RELAYS_NB
  #include "app_relay.h"
  #define RELAYS_TOPICS_NB 1
  #define CMD_RELAY_TOPIC CONFIG_MQTT_DEVICE_TYPE "/" CONFIG_MQTT_CLIENT_ID "/cmd/+/relay/+"
#else //CONFIG_MQTT_RELAYS_NB
  #define RELAYS_TOPICS_NB 0 
#endif //CONFIG_MQTT_RELAYS_NB

#ifdef CONFIG_MQTT_OTA

#include "app_ota.h"
extern QueueHandle_t otaQueue;
#define OTA_TOPIC CONFIG_MQTT_DEVICE_TYPE "/" CONFIG_MQTT_CLIENT_ID "/cmd/ota"
#define OTA_TOPICS_NB 1

#else // CONFIG_MQTT_OTA

#define OTA_TOPICS_NB 0

#endif // CONFIG_MQTT_OTA

#if CONFIG_MQTT_THERMOSTATS_NB > 0

#include "app_thermostat.h"
extern QueueHandle_t thermostatQueue;

#define THERMOSTAT_TOPICS_NB 1
#define CMD_THERMOSTAT_TOPIC CONFIG_MQTT_DEVICE_TYPE "/" CONFIG_MQTT_CLIENT_ID "/cmd/+/thermostat/+"

#else // CONFIG_MQTT_THERMOSTATS_NB > 0

#define THERMOSTAT_TOPICS_NB 0

#endif // CONFIG_MQTT_THERMOSTATS_NB > 0

esp_mqtt_client_handle_t client = NULL;
int connect_reason;
const int mqtt_disconnect = 33; //32+1

const char * available_topic = CONFIG_MQTT_DEVICE_TYPE "/" CONFIG_MQTT_CLIENT_ID "/evt/status/available";
const char * config_topic = CONFIG_MQTT_DEVICE_TYPE "/" CONFIG_MQTT_CLIENT_ID "/evt/config/available";

EventGroupHandle_t mqtt_event_group;
const int MQTT_CONNECTED_BIT = BIT0;
const int MQTT_SUBSCRIBED_BIT = BIT1;
const int MQTT_PUBLISHED_BIT = BIT2;
const int MQTT_INIT_FINISHED_BIT = BIT3;
const int MQTT_PUBLISHING_BIT = BIT4;

int mqtt_reconnect_counter;

#define FW_VERSION "947b169b+4"

extern QueueHandle_t mqttQueue;

static const char *TAG = "MQTTS_MQTT";

#ifdef CONFIG_MQTT_THERMOSTATS_NB0_SENSOR_TYPE_MQTT
#define CONFIG_MQTT_THERMOSTATS_NB0_SENSOR_MQTT 1
#else //CONFIG_MQTT_THERMOSTATS_NB0_SENSOR_TYPE_MQTT
#define CONFIG_MQTT_THERMOSTATS_NB0_SENSOR_MQTT 0
#endif //CONFIG_MQTT_THERMOSTATS_NB0_SENSOR_TYPE_MQTT

#ifdef CONFIG_MQTT_THERMOSTATS_NB1_SENSOR_TYPE_MQTT
#define CONFIG_MQTT_THERMOSTATS_NB1_SENSOR_MQTT 1
#else //CONFIG_MQTT_THERMOSTATS_NB1_SENSOR_TYPE_MQTT
#define CONFIG_MQTT_THERMOSTATS_NB1_SENSOR_MQTT 0
#endif //CONFIG_MQTT_THERMOSTATS_NB1_SENSOR_TYPE_MQTT

#ifdef CONFIG_MQTT_THERMOSTATS_NB2_SENSOR_TYPE_MQTT
#define CONFIG_MQTT_THERMOSTATS_NB2_SENSOR_MQTT 1
#else //CONFIG_MQTT_THERMOSTATS_NB2_SENSOR_TYPE_MQTT
#define CONFIG_MQTT_THERMOSTATS_NB2_SENSOR_MQTT 0
#endif //CONFIG_MQTT_THERMOSTATS_NB2_SENSOR_TYPE_MQTT

#ifdef CONFIG_MQTT_THERMOSTATS_NB3_SENSOR_TYPE_MQTT
#define CONFIG_MQTT_THERMOSTATS_NB3_SENSOR_MQTT 1
#else //CONFIG_MQTT_THERMOSTATS_NB3_SENSOR_TYPE_MQTT
#define CONFIG_MQTT_THERMOSTATS_NB3_SENSOR_MQTT 0
#endif //CONFIG_MQTT_THERMOSTATS_NB3_SENSOR_TYPE_MQTT

#define CONFIG_MQTT_THERMOSTATS_MQTT_SENSORS (CONFIG_MQTT_THERMOSTATS_NB0_SENSOR_MQTT + CONFIG_MQTT_THERMOSTATS_NB1_SENSOR_MQTT + CONFIG_MQTT_THERMOSTATS_NB2_SENSOR_MQTT + CONFIG_MQTT_THERMOSTATS_NB3_SENSOR_MQTT)

#define NB_SUBSCRIPTIONS  (OTA_TOPICS_NB + THERMOSTAT_TOPICS_NB + RELAYS_TOPICS_NB + SCHEDULER_TOPICS_NB + CONFIG_MQTT_THERMOSTATS_MQTT_SENSORS)

const char *SUBSCRIPTIONS[NB_SUBSCRIPTIONS] =
  {
#ifdef CONFIG_MQTT_OTA
    OTA_TOPIC,
#endif //CONFIG_MQTT_OTA
#ifdef CONFIG_MQTT_SCHEDULERS
    SCHEDULER_CFG_TOPIC "+",
#endif // CONFIG_MQTT_SCHEDULERS
#if CONFIG_MQTT_RELAYS_NB
    CMD_RELAY_TOPIC,
#endif //CONFIG_MQTT_RELAYS_NB
#if CONFIG_MQTT_THERMOSTATS_NB > 0
    CMD_THERMOSTAT_TOPIC,
#endif // CONFIG_MQTT_THERMOSTATS_NB > 0
#ifdef CONFIG_MQTT_THERMOSTATS_NB0_SENSOR_TYPE_MQTT
    CONFIG_MQTT_THERMOSTATS_NB0_MQTT_SENSOR_TOPIC,
#endif //CONFIG_MQTT_THERMOSTATS_NB0_SENSOR_TYPE_MQTT
#ifdef CONFIG_MQTT_THERMOSTATS_NB1_SENSOR_TYPE_MQTT
    CONFIG_MQTT_THERMOSTATS_NB1_MQTT_SENSOR_TOPIC,
#endif //CONFIG_MQTT_THERMOSTATS_NB1_SENSOR_TYPE_MQTT
#ifdef CONFIG_MQTT_THERMOSTATS_NB2_SENSOR_TYPE_MQTT
    CONFIG_MQTT_THERMOSTATS_NB2_MQTT_SENSOR_TOPIC,
#endif //CONFIG_MQTT_THERMOSTATS_NB2_SENSOR_TYPE_MQTT
#ifdef CONFIG_MQTT_THERMOSTATS_NB3_SENSOR_TYPE_MQTT
    CONFIG_MQTT_THERMOSTATS_NB3_MQTT_SENSOR_TOPIC,
#endif //CONFIG_MQTT_THERMOSTATS_NB3_SENSOR_TYPE_MQTT
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


#if CONFIG_MQTT_THERMOSTATS_MQTT_SENSORS > 0
void thermostat_publish_data(int thermostat_id, const char * payload)
{
  struct ThermostatMessage tm;
  memset(&tm, 0, sizeof(struct ThermostatMessage));
  tm.msgType = THERMOSTAT_CURRENT_TEMPERATURE;
  tm.thermostatId = thermostat_id;
  tm.data.currentTemperature = atof(payload) * 10;

  if (xQueueSend( thermostatQueue
                  ,( void * )&tm
                  ,MQTT_QUEUE_TIMEOUT) != pdPASS) {
    ESP_LOGE(TAG, "Cannot send to thermostatQueue");
  }
}

bool handleThermostatMqttSensor(esp_mqtt_event_handle_t event)
{
  char payload[MAX_MQTT_PAYLOAD_SIZE];
  memcpy(payload, event->data, event->data_len);
  payload[event->data_len] = 0;

#ifdef CONFIG_MQTT_THERMOSTATS_NB0_SENSOR_TYPE_MQTT
  if (strncmp(event->topic, CONFIG_MQTT_THERMOSTATS_NB0_MQTT_SENSOR_TOPIC, strlen(CONFIG_MQTT_THERMOSTATS_NB0_MQTT_SENSOR_TOPIC)) == 0) {
    thermostat_publish_data(0, payload);
    return true;
  }
#endif //CONFIG_MQTT_THERMOSTATS_NB0_SENSOR_TYPE_MQTT

#ifdef CONFIG_MQTT_THERMOSTATS_NB1_SENSOR_TYPE_MQTT
  if (strncmp(event->topic, CONFIG_MQTT_THERMOSTATS_NB1_MQTT_SENSOR_TOPIC, strlen(CONFIG_MQTT_THERMOSTATS_NB1_MQTT_SENSOR_TOPIC)) == 0) {
    thermostat_publish_data(1, payload);
    return true;
  }
#endif //CONFIG_MQTT_THERMOSTATS_NB1_SENSOR_TYPE_MQTT

#ifdef CONFIG_MQTT_THERMOSTATS_NB2_SENSOR_TYPE_MQTT
  if (strncmp(event->topic, CONFIG_MQTT_THERMOSTATS_NB2_MQTT_SENSOR_TOPIC, strlen(CONFIG_MQTT_THERMOSTATS_NB2_MQTT_SENSOR_TOPIC)) == 0) {
    thermostat_publish_data(2, payload);
    return true;
  }
#endif //CONFIG_MQTT_THERMOSTATS_NB2_SENSOR_TYPE_MQTT

#ifdef CONFIG_MQTT_THERMOSTATS_NB3_SENSOR_TYPE_MQTT
  if (strncmp(event->topic, CONFIG_MQTT_THERMOSTATS_NB3_MQTT_SENSOR_TOPIC, strlen(CONFIG_MQTT_THERMOSTATS_NB3_MQTT_SENSOR_TOPIC)) == 0) {
    thermostat_publish_data(3, payload);
    return true;
  }
#endif //CONFIG_MQTT_THERMOSTATS_NB3_SENSOR_TYPE_MQTT

  return false;
}
#endif // CONFIG_MQTT_THERMOSTATS_MQTT_SENSORS > 0


void handle_local_mqtt_msg(esp_mqtt_event_handle_t event)
{
  if (handle_ota_mqtt_event(event))
    return;

  // ^^^ old mqtt actions
  //=====================
  // vvv new mqtt actions

  char all_topic[64];
  memcpy(all_topic, event->topic, event->topic_len);
  all_topic[event->topic_len] = 0;
  
  char *topic = strstr(all_topic, "/");
  topic++;
  topic = strstr(topic, "/");
  topic++;
  
  char actionType[16];
  if (getActionType(actionType, topic) && strcmp(actionType, "cmd") == 0) {
    char payload[MAX_MQTT_PAYLOAD_SIZE];

    memcpy(payload, event->data, event->data_len);
    payload[event->data_len] = 0;

    char service[16];
    getService(service, topic);

#if CONFIG_MQTT_THERMOSTATS_NB > 0
    if (strcmp(service, "thermostat") == 0) {
      handle_thermostat_mqtt_cmd(topic, payload);
      return;
    }
#endif //CONFIG_MQTT_THERMOSTATS_NB > 0

#if CONFIG_MQTT_RELAYS_NB
    if (strcmp(service, "relay") == 0) {
      handle_relay_mqtt_cmd(topic, payload);
      return;
    }
#endif // CONFIG_MQTT_RELAYS_NB

#ifdef CONFIG_MQTT_SCHEDULERS
    if (strcmp(service, "scheduler") == 0) {
      handle_scheduler_mqtt_cmd(topic, payload);
      return;
    }
#endif // CONFIG_MQTT_SCHEDULERS
  }

}


void dispatch_mqtt_event(esp_mqtt_event_handle_t event)
{
  //including '\0'
  if (event->data_len > (MAX_MQTT_PAYLOAD_SIZE - 1)) {
    ESP_LOGE(TAG, "Payload is too big");
    return ;
  }
#if CONFIG_MQTT_THERMOSTATS_MQTT_SENSORS > 0
  if (handleThermostatMqttSensor(event)){
    return;
  }
#endif // CONFIG_MQTT_THERMOSTATS_MQTT_SENSORS > 0
  handle_local_mqtt_msg(event);
}

void mqtt_publish_data(const char * topic,
                       const char * data,
                       int qos, int retain)
{
  if (xEventGroupGetBits(mqtt_event_group) & MQTT_INIT_FINISHED_BIT) {
    EventBits_t bits = xEventGroupWaitBits(mqtt_event_group, MQTT_PUBLISHING_BIT, false, true, MQTT_FLAG_TIMEOUT);
    if (bits & MQTT_PUBLISHING_BIT) {
      xEventGroupClearBits(mqtt_event_group, MQTT_PUBLISHING_BIT);

      xEventGroupClearBits(mqtt_event_group, MQTT_PUBLISHED_BIT);
      int msg_id = esp_mqtt_client_publish(client, topic, data, strlen(data), qos, retain);
      if (qos == QOS_0) {
        ESP_LOGI(TAG, "published qos0 data, topic: %s", topic);
      } else if (msg_id > 0) {
        ESP_LOGI(TAG, "published qos1 data, msg_id=%d, topic=%s", msg_id, topic);
        EventBits_t bits = xEventGroupWaitBits(mqtt_event_group, MQTT_PUBLISHED_BIT, false, true, MQTT_FLAG_TIMEOUT);
        if (bits & MQTT_PUBLISHED_BIT) {
          ESP_LOGI(TAG, "publish ack received, msg_id=%d", msg_id);
        } else {
          ESP_LOGW(TAG, "publish ack not received, msg_id=%d", msg_id);
        }
      } else {
        ESP_LOGW(TAG, "failed to publish qos1, msg_id=%d", msg_id);
      }
      xEventGroupSetBits(mqtt_event_group, MQTT_PUBLISHING_BIT);
    }
  }
}

void publish_config_msg()
{
  char data[64];
  memset(data,0,64);

  sprintf(data, "{\"fw_version\":\"" FW_VERSION "\", \"connect_reason\":%d}", connect_reason);
  mqtt_publish_data(config_topic, data, QOS_1, RETAIN);

}

void publish_available_msg()
{
  char* data = "online";
  mqtt_publish_data(available_topic, data, QOS_1, RETAIN);
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
    connect_reason=mqtt_disconnect;
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
  connect_reason=esp_reset_reason();
  void * unused;
  while(1) {
    if( xQueueReceive( mqttQueue, &unused , portMAX_DELAY) )
      {
        mqtt_subscribe(client);
        xEventGroupSetBits(mqtt_event_group, MQTT_INIT_FINISHED_BIT);
        xEventGroupSetBits(mqtt_event_group, MQTT_PUBLISHING_BIT);
        publish_available_msg();
        publish_config_msg();
#if CONFIG_MQTT_RELAYS_NB
        publish_all_relays_status();
        publish_all_relays_timeout();
#endif//CONFIG_MQTT_RELAYS_NB
#if CONFIG_MQTT_THERMOSTATS_NB > 0
        publish_thermostat_data();
#endif // CONFIG_MQTT_THERMOSTATS_NB > 0
#if CONFIG_MQTT_SCHEDULERS > 0
        publish_scheduler_data();
#endif // CONFIG_MQTT_SCHEDULERS
#ifdef CONFIG_MQTT_OTA
        publish_ota_data(OTA_READY);
#endif //CONFIG_MQTT_OTA
#ifdef CONFIG_MQTT_SENSOR
        publish_sensors_data();
#endif//
      }
  }
}


