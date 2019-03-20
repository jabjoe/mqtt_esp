#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "app_esp8266.h"
#include "app_relay.h"

#include "app_switch.h"

#include "driver/gpio.h"


int switch1_value=0;
int switch2_value=0;

int switch1_gpio=0;
int switch2_gpio=9;


extern QueueHandle_t relayQueue;

static void gpio1_isr_handler(void *arg)
{
  switch1_value = ! switch1_value;
  struct RelayMessage r={0, switch1_value};
  xQueueSendFromISR(relayQueue
                    ,( void * )&r
                    ,NULL);
}

static void gpio2_isr_handler(void *arg)
{
  switch2_value = ! switch2_value;
  struct RelayMessage r={1, switch2_value};
  xQueueSendFromISR(relayQueue
                    ,( void * )&r
                    ,NULL);
}

void gpio_switch_init (void *arg)
{
  gpio_config_t io_conf;
//interrupt of rising edge
  io_conf.intr_type = GPIO_INTR_POSEDGE;
  //bit mask of the pins
  io_conf.pin_bit_mask = 0;
  io_conf.pin_bit_mask |= (1ULL << switch1_gpio);
  io_conf.pin_bit_mask |= (1ULL << switch2_gpio);
  //set as input mode
  io_conf.mode = GPIO_MODE_INPUT;
  gpio_config(&io_conf);


  //install gpio isr service
  gpio_install_isr_service(0);
  //hook isr handler for specific gpio pin
  gpio_isr_handler_add(switch1_gpio, gpio1_isr_handler, (void *) switch1_gpio);
  gpio_isr_handler_add(switch2_gpio, gpio2_isr_handler, (void *) switch2_gpio);
  //hook isr handler for specific gpio pin

}

