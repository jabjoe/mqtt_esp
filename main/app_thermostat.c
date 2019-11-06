#include "esp_system.h"
#ifdef CONFIG_MQTT_THERMOSTAT

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include <string.h>

#include "app_main.h"
#include "app_relay.h"
#include "app_thermostat.h"
#include "app_nvs.h"
#include "app_mqtt.h"

bool thermostatEnabled = false;
bool heatingEnabled = false;
unsigned int thermostatDuration = 0;
unsigned int heatingDuration = 0;

int columnTargetTemperature=23*10; //30 degrees
int waterTargetTemperature=23*10; //30 degrees
int waterTemperatureSensibility=5; //0.5 degrees

int room0TargetTemperature=22*10;
int room0TemperatureSensibility=2;

const char * columnTargetTemperatureTAG="ctgtTemp";
const char * waterTargetTemperatureTAG="wTargetTemp";
const char * waterTemperatureSensibilityTAG="wTempSens";
const char * room0TargetTemperatureTAG="r0targetTemp";
const char * room0TemperatureSensibilityTAG="r0TempSens";

int32_t room0Temperature = 0;
int32_t wtemperature = 0;
int32_t ctemperature = 0;

unsigned int room0TemperatureFlag = SENSOR_LIFETIME;
unsigned int wtemperatureFlag = SENSOR_LIFETIME;
unsigned int ctemperatureFlag = SENSOR_LIFETIME;

int32_t ctemperature_1 = 0;
int32_t ctemperature_2 = 0;
int32_t ctemperature_3 = 0;

extern QueueHandle_t thermostatQueue;


static const char *TAG = "APP_THERMOSTAT";

void publish_thermostat_cfg()
{

  const char * topic = CONFIG_MQTT_DEVICE_TYPE "/" CONFIG_MQTT_CLIENT_ID "/evt/thermostat/cfg";

  char data[256];
  memset(data,0,256);

  sprintf(data, "{\"columnTargetTemperature\":%d.%01d, \"waterTargetTemperature\":%d.%01d, \"waterTemperatureSensibility\":%d.%01d, \"room0TargetTemperature\":%d.%01d, \"room0TemperatureSensibility\":%d.%01d}",
          columnTargetTemperature/10, columnTargetTemperature%10,
          waterTargetTemperature/10, waterTargetTemperature%10,
          waterTemperatureSensibility/10, waterTemperatureSensibility%10,
          room0TargetTemperature/10,room0TargetTemperature%10,
          room0TemperatureSensibility/10, room0TemperatureSensibility%10);
  mqtt_publish_data(topic, data, QOS_1, RETAIN);
}

void publish_thermostat_state(const char* reason, unsigned int duration)
{
  const char * topic = CONFIG_MQTT_DEVICE_TYPE "/" CONFIG_MQTT_CLIENT_ID "/evt/thermostat/state";
  char data[256];
  memset(data,0,256);

  if(!reason) {
    reason = "";
  }
  sprintf(data, "{\"thermostatState\":%d, \"heatingState\":%d, \"reason\":%s, \"duration\":%u}",
          thermostatEnabled, heatingEnabled, reason, duration);

  mqtt_publish_data(topic, data, QOS_1, RETAIN);

}

void publish_thermostat_data()
{
  publish_thermostat_state(NULL, 0);
  publish_thermostat_cfg();
}


void disableThermostat(const char * reason)
{
  publish_thermostat_state(NULL, 0);
  thermostatEnabled=false;
  update_relay_state(CONFIG_MQTT_THERMOSTAT_RELAY_ID, 0);
  publish_thermostat_state(reason, thermostatDuration);
  thermostatDuration = 0;
  ESP_LOGI(TAG, "thermostat disabled");
}

void enableThermostat(const char * reason)
{
  publish_thermostat_state(NULL, 0);
  thermostatEnabled=true;
  update_relay_state(CONFIG_MQTT_THERMOSTAT_RELAY_ID, 1);
  publish_thermostat_state(reason, thermostatDuration);
  thermostatDuration = 0;
  ESP_LOGI(TAG, "thermostat enabled");
}

void enableHeating()
{
  publish_thermostat_state(NULL, 0);
  heatingEnabled = true;
  ESP_LOGI(TAG, "heating enabled");
  publish_thermostat_state("Heating was enabled", heatingDuration);
  heatingDuration = 0;
}
void disableHeating()
{
  publish_thermostat_state(NULL, 0);
  heatingEnabled = false;
  ESP_LOGI(TAG, "heating2 disabled");
  publish_thermostat_state("Heating was disabled", heatingDuration);
  heatingDuration = 0;
}

void update_thermostat()
{
  bool heatingToggledOff = false;
  ESP_LOGI(TAG, "heatingEnabled state is %d", heatingEnabled);
  ESP_LOGI(TAG, "ctemperature_n is %d", ctemperature);
  ESP_LOGI(TAG, "ctemperature_n_1 is %d", ctemperature_1);
  ESP_LOGI(TAG, "ctemperature_n_2 is %d", ctemperature_2);
  ESP_LOGI(TAG, "ctemperature_n_3 is %d", ctemperature_3);
  ESP_LOGI(TAG, "columnTargetTemperature is %d", columnTargetTemperature);
  ESP_LOGI(TAG, "thermostat state is %d", thermostatEnabled);
  ESP_LOGI(TAG, "wtemperature is %d", wtemperature);
  ESP_LOGI(TAG, "waterTargetTemperature is %d", waterTargetTemperature);
  ESP_LOGI(TAG, "waterTemperatureSensibility is %d", waterTemperatureSensibility);
  ESP_LOGI(TAG, "room0Temperature is %d", room0Temperature);
  ESP_LOGI(TAG, "room0TargetTemperature is %d", room0TargetTemperature);
  ESP_LOGI(TAG, "room0TemperatureSensibility is %d", room0TemperatureSensibility);

  if ( wtemperatureFlag == 0 && room0TemperatureFlag == 0 ) {
    ESP_LOGI(TAG, "no live sensor is reporting => no thermostat handling");
    if (thermostatEnabled==true) {
      ESP_LOGI(TAG, "stop thermostat as no live sensor is reporting");
      disableThermostat("Thermostat was disabled as no live sensor is reporting");
    }
    return;
  }

  if (ctemperature_3 && ctemperature_2 && ctemperature_1 && ctemperature) {//four consecutive valid readings
    if (!heatingEnabled
        && ( ctemperature_3 < ctemperature_2
             && ctemperature_2 < ctemperature_1
             && ctemperature_1 < ctemperature )){ //water is heating 1 2 3 4
      enableHeating();
    }

    if (heatingEnabled
        && ( ctemperature_3 >= ctemperature_2
             || ctemperature_2 >= ctemperature_1
             || ctemperature_1 >= ctemperature )) { //heating is disabled   5 4 3 2
      disableHeating();
      heatingToggledOff = true;
    }
  }

  bool waterTooHot = (wtemperatureFlag > 0) &&
    (wtemperature > (waterTargetTemperature + waterTemperatureSensibility));

  bool roomTooHot = (room0TemperatureFlag > 0) &&
    (room0Temperature > (room0TargetTemperature + room0TemperatureSensibility));

  bool waterTooCold = (wtemperatureFlag > 0) &&
    (wtemperature < (waterTargetTemperature - waterTemperatureSensibility));

  bool roomTooCold = (room0TemperatureFlag > 0) &&
    (room0Temperature < (room0TargetTemperature - room0TemperatureSensibility));

  bool circuitColdEnough = (wtemperatureFlag > 0) ? (ctemperature < columnTargetTemperature) : true;

  if (thermostatEnabled &&
      (heatingToggledOff || (roomTooHot && waterTooHot))) {
    const char * reason = heatingToggledOff ?
      ((roomTooHot && waterTooHot) ? "Thermostat was disabled due to heating stopped and rooms are too hot" : "Thermostat was disabled due to heating stopped") :
      ((roomTooHot && waterTooHot) ? "Thermostat was disabled due to rooms are too hot" : "should never print" );
    ESP_LOGI(TAG, "%s", reason);
    disableThermostat(reason);
  }

  if (!thermostatEnabled &&
      (waterTooCold || roomTooCold) && circuitColdEnough) {
    const char * reason = waterTooCold ? (waterTooCold ? "Thermostat enabled because water and room are too cold" : "Thermostat enabled because room is too cold") : (waterTooCold ? "Thermostat enabled because water is too cold" : "should never print");
    ESP_LOGI(TAG, "%s", reason);
    enableThermostat(reason);
    }
  ctemperature_3 = ctemperature_2;
  ctemperature_2 = ctemperature_1;
  ctemperature_1 = ctemperature;
}

void vThermostatTimerCallback( TimerHandle_t xTimer )
{
  ESP_LOGI(TAG, "Thermostat timer expired");
  struct ThermostatMessage t;
  t.msgType = THERMOSTAT_LIFE_TICK;
  if (xQueueSend( thermostatQueue,
                  ( void * )&t,
                  MQTT_QUEUE_TIMEOUT) != pdPASS) {
  }
}

void handle_thermostat_cmd_task(void* pvParameters)
{

  //create period read timer
  TimerHandle_t th =
    xTimerCreate( "thermostatSensorsTimer",           /* Text name. */
                  pdMS_TO_TICKS(60000),  /* Period. */
                  pdTRUE,                /* Autoreload. */
                  (void *)0,                  /* No ID. */
                  vThermostatTimerCallback );  /* Callback function. */
  if( th != NULL ) {
    ESP_LOGI(TAG, "timer is created");
    if (xTimerStart(th, portMAX_DELAY) != pdFALSE){
      ESP_LOGI(TAG, "timer is active");
    }
  }
  struct ThermostatMessage t;
  while(1) {
    if( xQueueReceive( thermostatQueue, &t , portMAX_DELAY) )
      {
        if (t.msgType == THERMOSTAT_CFG_MSG) {
          bool updated = false;
          if (t.data.cfgData.columnTargetTemperature && columnTargetTemperature != t.data.cfgData.columnTargetTemperature) {
            columnTargetTemperature=t.data.cfgData.columnTargetTemperature;
            esp_err_t err = write_nvs_integer(columnTargetTemperatureTAG, columnTargetTemperature);
            ESP_ERROR_CHECK( err );
            updated = true;
          }
          if (t.data.cfgData.waterTargetTemperature && waterTargetTemperature != t.data.cfgData.waterTargetTemperature) {
            waterTargetTemperature=t.data.cfgData.waterTargetTemperature;
            esp_err_t err = write_nvs_integer(waterTargetTemperatureTAG, waterTargetTemperature);
            ESP_ERROR_CHECK( err );
            updated = true;
          }
          if (t.data.cfgData.waterTemperatureSensibility && waterTemperatureSensibility != t.data.cfgData.waterTemperatureSensibility) {
            waterTemperatureSensibility=t.data.cfgData.waterTemperatureSensibility;
            esp_err_t err = write_nvs_integer(waterTemperatureSensibilityTAG, waterTemperatureSensibility);
            ESP_ERROR_CHECK( err );
            updated = true;
          }
          if (t.data.cfgData.room0TargetTemperature && room0TargetTemperature != t.data.cfgData.room0TargetTemperature) {
            room0TargetTemperature=t.data.cfgData.room0TargetTemperature;
            esp_err_t err = write_nvs_integer(room0TargetTemperatureTAG, room0TargetTemperature);
            ESP_ERROR_CHECK( err );
            updated = true;
          }
          if (t.data.cfgData.room0TemperatureSensibility && room0TemperatureSensibility != t.data.cfgData.room0TemperatureSensibility) {
            room0TemperatureSensibility=t.data.cfgData.room0TemperatureSensibility;
            esp_err_t err = write_nvs_integer(room0TemperatureSensibilityTAG, room0TemperatureSensibility);
            ESP_ERROR_CHECK( err );
            updated = true;
          }
          if (updated) {
            publish_thermostat_cfg();
          }
        }
        if (t.msgType == THERMOSTAT_SENSORS_MSG) {
          wtemperature = t.data.sensorsData.wtemperature;
          ctemperature = t.data.sensorsData.ctemperature;
          if (wtemperature > 0) {
            wtemperatureFlag = SENSOR_LIFETIME;
          }
          ctemperature = t.data.sensorsData.ctemperature;
          if (ctemperature > 0) {
            ctemperatureFlag = SENSOR_LIFETIME;
          }
        }
        if (t.msgType == THERMOSTAT_ROOM_0_MSG) {
          room0Temperature = t.data.roomData.temperature;
          if (room0Temperature > 0) {
            room0TemperatureFlag = SENSOR_LIFETIME;
          }
        }
        if (t.msgType == THERMOSTAT_LIFE_TICK) {
          thermostatDuration += 1;
          heatingDuration += 1;

          if (room0TemperatureFlag > 0)
            room0TemperatureFlag -= 1;
          if (wtemperatureFlag > 0)
            wtemperatureFlag -= 1;
          if (ctemperatureFlag > 0)
            ctemperatureFlag -= 1;
          ESP_LOGI(TAG, "room0TemperatureFlag: %d, wtemperatureFlag: %d, ctemperatureFlag: %d",
                   room0TemperatureFlag, wtemperatureFlag, ctemperatureFlag);
        }
        update_thermostat();
      }
  }
}

void read_nvs_thermostat_data()
{
  esp_err_t err=read_nvs_integer(columnTargetTemperatureTAG, &columnTargetTemperature);
  ESP_ERROR_CHECK( err );

  err=read_nvs_integer(waterTargetTemperatureTAG, &waterTargetTemperature);
  ESP_ERROR_CHECK( err );

  err=read_nvs_integer(waterTemperatureSensibilityTAG, &waterTemperatureSensibility);
  ESP_ERROR_CHECK( err );

  err=read_nvs_integer(room0TargetTemperatureTAG, &room0TargetTemperature);
  ESP_ERROR_CHECK( err );

  err=read_nvs_integer(room0TemperatureSensibilityTAG, &room0TemperatureSensibility);
  ESP_ERROR_CHECK( err );
}
#endif // CONFIG_MQTT_THERMOSTAT
