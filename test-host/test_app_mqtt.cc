#include "esp_system.h"
#include "catch.hpp"
#include "hippomocks.h"
#include "cJSON.h"

extern "C" {
  bool getTemperatureValue(short* value, const cJSON* root, const char* tag);
  char* getToken(char* buffer, const char* topic, unsigned char place);
  char* getAction(char* buffer, const char* topic);
  char* getService(char* buffer, const char* topic);
  char getServiceId(const char* topic);
}

TEST_CASE("test get ", "[tag]" ) {
  short temp=0;
  const char* json = "{\"humidity\":53.3,\"temperature\":18.3}";
  cJSON * root   = cJSON_Parse(json);

  REQUIRE(getTemperatureValue(&temp, root, "temperature"));
  REQUIRE(temp == 183);
}

TEST_CASE("get null token from null topic ", "[tag]" ) {
  char buffer[16];
  REQUIRE(getToken(buffer, NULL, 0) == NULL);
}

TEST_CASE("get null token from empty topic ", "[tag]" ) {
  char buffer[16];
  REQUIRE(getToken(buffer, "",0) == NULL);
}

TEST_CASE("get string if no token in string", "[tag]" ) {
  char buffer[16];
  REQUIRE(strcmp(getToken(buffer, "a",0) , "a") == 0);
}

TEST_CASE("get null token if token place to big", "[tag]" ) {
  char buffer[16];
  REQUIRE(getToken(buffer, "a",1) == NULL);
}

TEST_CASE("get first token", "[tag]" ) {
  char buffer[16];
  REQUIRE(strcmp(getToken(buffer, "a/b/c",0),"a") == 0);
}
TEST_CASE("get second token", "[tag]" ) {
  char buffer[16];
  REQUIRE(strcmp(getToken(buffer, "a/b/c",1), "b") == 0);
}
TEST_CASE("get third token", "[tag]" ) {
  char buffer[16];
  REQUIRE(strcmp(getToken(buffer, "a/b/c",2), "c") == 0);
}
TEST_CASE("get null token as no fourth token", "[tag]" ) {
  char buffer[16];
  REQUIRE(getToken(buffer, "a/b/c",3) == NULL);
}

TEST_CASE("get action from topic", "[tag]" ) {
  char action[16];
  REQUIRE(strcmp(getAction(action, "esp32/ground0/cmd/state/relay/0"),"state") == 0);
}
TEST_CASE("get service from topic", "[tag]" ) {
  char service[16];
  REQUIRE(strcmp(getService(service, "esp32/ground0/cmd/state/relay/0"),"relay") == 0);
}
TEST_CASE("get service_id from topic", "[tag]" ) {
  REQUIRE(getServiceId("esp32/ground0/cmd/state/relay/0") == 0);
}
