#pragma once

void start_mqtt(void);
void stop_mqtt(void);
void mqtt_sub(char* channel);
void mqtt_unsub(char* channel);
void mqtt_node_publish(char code, char* channel, char* data, uint16_t len);

