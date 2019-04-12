#include "gll.h"
#include "register.h"

static gll_t m_nodes;

void register_node(NodeDescription* node) {
    //TODO check for dupes here (dd)
    gll_pushBack(&m_nodes, (void *)node);
}

void register_nodes_to_string(char *buff) {
    NodeDescription* data;
    gll_node_t *curr = m_nodes.first;
    uint16_t ni = 0;
    uint16_t len = 0;

    buff[len++] = '[';

    while (ni < m_nodes.size) {
        data = (NodeDescription *)curr->data;
        data->to_string(data->node, buff, &len);

        if (++ni < m_nodes.size) {
            buff[len++] = ',';
        }

        curr = curr->next;
    }

    buff[len++] = ']';
    buff[len++] = '\0';
}

void register_sub_nodes(char* prefix) {
    NodeDescription* data;
    gll_node_t *curr = m_nodes.first;
    uint16_t ni = 0;

    while (ni < m_nodes.size) {
        data = (NodeDescription *)curr->data;
        data->sub(prefix, data->node);

        curr = curr->next;
        ni++;
    }
}

void register_unsub_nodes(char* prefix) {
    NodeDescription* data;
    gll_node_t *curr = m_nodes.first;
    uint16_t ni = 0;

    while (ni < m_nodes.size) {
        data = (NodeDescription *)curr->data;
        data->unsub(prefix, data->node);

        curr = curr->next;
        ni++;
    }
}

void register_handle_nodes(esp_mqtt_event_handle_t evt, char* prefix) {
    NodeDescription* data;
    gll_node_t *curr = m_nodes.first;
    uint16_t ni = 0;

    while (ni < m_nodes.size) {
        data = (NodeDescription *)curr->data;

        if(data->handle(evt, prefix, data->node)) {
            break;
        }

        curr = curr->next;
        ni++;
    }
}

static char str_buff[120];
char* gen_topic(char* prefix, char axis, char* topic) {
    char _axis[2] = { axis, '\0' };

    strcpy(str_buff, "/");
    strcat(str_buff, prefix);
    strcat(str_buff, "/");
    strcat(str_buff, _axis);
    strcat(str_buff, topic);
    return str_buff;
}

