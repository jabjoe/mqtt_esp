#include <string.h>
#include <stdlib.h>

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
  return getToken(actionType, topic, 0);
}
char* getAction(char* action, const char* topic)
{
  return getToken(action, topic, 1);
}

char* getService(char* service, const char* topic)
{
  return getToken(service, topic, 2);
}

signed char getServiceId(const char* topic, int topic_)
{
  signed char serviceId=-1;
  char buffer[8];
  char* s = getToken(buffer, topic, 3);
  if (s) {
    serviceId = atoi(s);
  }
  return serviceId;
}
