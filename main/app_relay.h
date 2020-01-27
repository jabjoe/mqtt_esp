#ifndef APP_RELAY_H
#define APP_RELAY_H


enum RelayState {
  RELAY_STATE_UNSET=0,
  RELAY_STATE_ON   =1,
  RELAY_STATE_OFF  =2
};

union RelayData
{
  enum RelayState state;
  unsigned char timeout;
};

struct RelayMessage {
  unsigned char msgType;
  char relayId;
  union RelayData data;
};

#define RELAY_STATE_CMD 1
#define RELAY_TIMEOUT_CMD 2


void publish_all_relays_status();
void publish_relay_status(int id);

void publish_all_relays_timeout();


void relays_init(void);

void handle_relay_cmd_task(void* pvParameters);

#define RELAY_TIMEOUT 30
#define RELAY_QUEUE_TIMEOUT (RELAY_TIMEOUT * 1000 / portTICK_PERIOD_MS)

#endif /* APP_RELAY_H */
