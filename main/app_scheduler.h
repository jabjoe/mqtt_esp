#ifndef APP_SCHEDULER_H
#define APP_SCHEDULER_H

#include "app_relay.h"

void start_scheduler_timer(void);
void handle_scheduler(void* pvParameters);

#define RELAY_ACTION 1
#define ADD_RELAY_ACTION 1
#define TRIGGER_ACTION 255
#define MAX_SCHEDULER_NB 8

#define ACTION_STATE_DISABLED 0
#define ACTION_STATE_ENABLED 1

//FIXME basic structure only

union Data {
  struct RelayMessage relayActionData;
  struct TriggerData {time_t now;} triggerActionData;
};

struct SchedulerCfgMessage
{
  unsigned char schedulerId;
  time_t timestamp;
  unsigned char actionId;
  unsigned char actionState;
  union Data data;
};

#define MAX_MQTT_SCHEDULER_PAYLOAD_SIZE 64
#define MAX_MQTT_SCHEDULER_AIO_PAYLOAD_SIZE 128


struct Scheduler_AIO {
  char topic[MAX_MQTT_SCHEDULER_PAYLOAD_SIZE];
  char payload[MAX_MQTT_SCHEDULER_PAYLOAD_SIZE];
  char starttime[MAX_MQTT_SCHEDULER_PAYLOAD_SIZE];
  char repeat[MAX_MQTT_SCHEDULER_PAYLOAD_SIZE];
  int status;
};


union SchedulerData {
  char topic[MAX_MQTT_SCHEDULER_PAYLOAD_SIZE];
  char payload[MAX_MQTT_SCHEDULER_PAYLOAD_SIZE];
  char starttime[MAX_MQTT_SCHEDULER_PAYLOAD_SIZE];
  char repeat[MAX_MQTT_SCHEDULER_PAYLOAD_SIZE];
  int status;
  struct Scheduler_AIO aio;
};


struct SchedulerMessage {
  unsigned char msgType;
  unsigned char schedulerId;
  union SchedulerData data;
};

#define SCHEDULER_CMD_TOPIC 1
#define SCHEDULER_CMD_PAYLOAD 2
#define SCHEDULER_CMD_STARTTIME 3
#define SCHEDULER_CMD_REPEAT 4
#define SCHEDULER_CMD_STATUS 5
#define SCHEDULER_CMD_AIO 6
#define SCHEDULER_LIFE_TICK 7

#define SCHEDULER_STATUS_ON 1
#define SCHEDULER_STATUS_OFF 2

#define SCHEDULE_TIMEOUT 30
#define SCHEDULE_QUEUE_TIMEOUT (SCHEDULE_TIMEOUT * 1000 / portTICK_PERIOD_MS)

void read_nvs_scheduler_data(void);
void publish_scheduler_data(void);

#endif /* APP_SCHEDULER_H */


