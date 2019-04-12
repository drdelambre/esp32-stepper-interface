#pragma once
#include "mqtt_client.h"

typedef void(*str_func)(void* node, char* buffer, uint16_t* offset);
typedef void(*sub_func)(char* prefix, void* node);
typedef bool(*handle_func)(esp_mqtt_event_handle_t evt, char* prefix, void* node);

typedef struct nodedesc {
    void*       node;           // reference to the node

    str_func    to_string;      // serializes a node for the /edge_node/info request

    sub_func    sub;            // registers the node to the mqtt server
    sub_func    unsub;          // unregisters the node from the mqtt server
    handle_func handle;         // handles an mqtt request
} NodeDescription;

void register_node(NodeDescription* node);
void register_nodes_to_string(char* buffer);
void register_sub_nodes(char* prefix);
void register_unsub_nodes(char* prefix);
void register_handle_nodes(esp_mqtt_event_handle_t evt, char* prefix);
char* gen_topic(char* prefix, char axis, char* topic);
