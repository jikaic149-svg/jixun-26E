#ifndef TASK_HEX25E_ROUTE_H
#define TASK_HEX25E_ROUTE_H

#include "hex25e_config.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    HEX25E_NODE_O = 0,
    HEX25E_NODE_A,
    HEX25E_NODE_G,
    HEX25E_NODE_F,
    HEX25E_NODE_E,
    HEX25E_NODE_T1,
    HEX25E_NODE_T2,
    HEX25E_NODE_M,
    HEX25E_NODE_B2,
    HEX25E_NODE_B1,
    HEX25E_NODE_D,
    HEX25E_NODE_C,
    HEX25E_NODE_B,
    HEX25E_NODE_H,
} Hex25E_NodeId;

typedef struct {
    int8_t neighbor[3];
    int8_t heading[3];
    uint8_t degree;
} Hex25E_Node;

const Hex25E_Node *Hex25E_Map_GetNode(Hex25E_NodeId node);
bool Hex25E_Map_IsYJunction(Hex25E_NodeId node);
const Hex25E_NodeId *Hex25E_Map_GetShortestRoute(size_t *node_count);
/* Heading of route edge[index], from route[index] to route[index + 1]. */
bool Hex25E_Map_GetRouteHeading(size_t edge_index, uint8_t *heading);
bool Hex25E_Map_GetRouteEdgeLength(size_t edge_index, float *length_mm);
bool Hex25E_Map_ValidateShortestRoute(void);

#endif
