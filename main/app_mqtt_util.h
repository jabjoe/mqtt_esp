char* getToken(char* buffer, const char* topic, int topic_len, unsigned char place);


char* getActionType(char* actionType, const char* topic);
char* getAction(char* action, const char* topic);
char* getService(char* service, const char* topic);
signed char getServiceId(const char* topic);
