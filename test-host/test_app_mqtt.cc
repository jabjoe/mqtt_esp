#include "esp_system.h"
#include "catch.hpp"
#include "hippomocks.h"
#include "cJSON.h"

extern "C" {
  bool getTemperatureValue(short* value, const cJSON* root, const char* tag);
  const char* getToken(const char* topic, unsigned char place);
  const char* getAction(const char* topic);
  const char* getService(const char* topic);
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
  REQUIRE(getToken(NULL,0) == NULL);
}

TEST_CASE("get null token from empty topic ", "[tag]" ) {
  REQUIRE(getToken("",0) == NULL);
}

TEST_CASE("get string if no token in string", "[tag]" ) {
  REQUIRE(strcmp(getToken("a",0) , "a") == 0);
}

TEST_CASE("get null token if token place to big", "[tag]" ) {
  REQUIRE(getToken("a",1) == NULL);
}

TEST_CASE("get first token", "[tag]" ) {
  REQUIRE(strcmp(getToken("a/b/c",0),"a") == 0);
}
TEST_CASE("get second token", "[tag]" ) {
  REQUIRE(strcmp(getToken("a/b/c",1), "b") == 0);
}
TEST_CASE("get third token", "[tag]" ) {
  REQUIRE(strcmp(getToken("a/b/c",2), "c") == 0);
}
TEST_CASE("get null token as no fourth token", "[tag]" ) {
  REQUIRE(getToken("a/b/c",3) == NULL);
}

TEST_CASE("get action from topic", "[tag]" ) {
  REQUIRE(strcmp(getAction("esp32/ground0/cmd/state/relay/0"),"state") == 0);
}
TEST_CASE("get service from topic", "[tag]" ) {
  REQUIRE(strcmp(getService("esp32/ground0/cmd/state/relay/0"),"relay") == 0);
}
TEST_CASE("get service_id from topic", "[tag]" ) {
  REQUIRE(getServiceId("esp32/ground0/cmd/state/relay/0") == 0);
}
