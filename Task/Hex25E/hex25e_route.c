#include "hex25e_route.h"

#include <stddef.h>

#define X (-1)

static const Hex25E_Node g_nodes[HEX25E_CONFIG_NODE_COUNT] = {
    [HEX25E_NODE_O]  = {{HEX25E_NODE_A, X, X}, {3, X, X}, 1U},
    [HEX25E_NODE_A]  = {{HEX25E_NODE_O, HEX25E_NODE_G, HEX25E_NODE_B},
                       {0, 2, 4}, 3U},
    [HEX25E_NODE_G]  = {{HEX25E_NODE_A, HEX25E_NODE_F, X}, {5, 3, X}, 2U},
    [HEX25E_NODE_F]  = {{HEX25E_NODE_G, HEX25E_NODE_E, HEX25E_NODE_H},
                       {0, 2, 4}, 3U},
    [HEX25E_NODE_E]  = {{HEX25E_NODE_F, HEX25E_NODE_T1, X}, {5, 3, X}, 2U},
    [HEX25E_NODE_T1] = {{HEX25E_NODE_E, HEX25E_NODE_T2, X}, {0, 4, X}, 2U},
    [HEX25E_NODE_T2] = {{HEX25E_NODE_T1, HEX25E_NODE_M, X}, {1, 5, X}, 2U},
    [HEX25E_NODE_M]  = {{HEX25E_NODE_T2, HEX25E_NODE_H, HEX25E_NODE_B2},
                       {2, 0, 4}, 3U},
    [HEX25E_NODE_B2] = {{HEX25E_NODE_M, HEX25E_NODE_B1, X}, {1, 5, X}, 2U},
    [HEX25E_NODE_B1] = {{HEX25E_NODE_B2, HEX25E_NODE_D, X}, {2, 0, X}, 2U},
    [HEX25E_NODE_D]  = {{HEX25E_NODE_B1, HEX25E_NODE_C, X}, {3, 1, X}, 2U},
    [HEX25E_NODE_C]  = {{HEX25E_NODE_D, HEX25E_NODE_H, HEX25E_NODE_B},
                       {4, 2, 0}, 3U},
    [HEX25E_NODE_B]  = {{HEX25E_NODE_C, HEX25E_NODE_A, X}, {3, 1, X}, 2U},
    [HEX25E_NODE_H]  = {{HEX25E_NODE_M, HEX25E_NODE_F, HEX25E_NODE_C},
                       {3, 1, 5}, 3U},
};

/* Unique shortest closed walk from O that visits all five Y nodes when one
 * 180-degree reversal is allowed. The reversal is H->M->H at M. */
static const Hex25E_NodeId g_shortest_route[
    HEX25E_CONFIG_ROUTE_NODE_COUNT] = {
    HEX25E_NODE_O, HEX25E_NODE_A, HEX25E_NODE_G, HEX25E_NODE_F,
    HEX25E_NODE_H, HEX25E_NODE_M, HEX25E_NODE_H, HEX25E_NODE_C,
    HEX25E_NODE_B, HEX25E_NODE_A, HEX25E_NODE_O,
};

static const uint8_t g_route_heading[HEX25E_CONFIG_ROUTE_EDGE_COUNT] = {
    3U, 2U, 3U, 4U, 3U, 0U, 5U, 0U, 1U, 0U,
};

const Hex25E_Node *Hex25E_Map_GetNode(Hex25E_NodeId node)
{
    return ((unsigned int)node < HEX25E_CONFIG_NODE_COUNT) ?
        &g_nodes[node] : NULL;
}

bool Hex25E_Map_IsYJunction(Hex25E_NodeId node)
{
    const Hex25E_Node *entry = Hex25E_Map_GetNode(node);
    return (entry != NULL) && (entry->degree == 3U);
}

const Hex25E_NodeId *Hex25E_Map_GetShortestRoute(size_t *node_count)
{
    if (node_count == NULL) {
        return NULL;
    }
    *node_count = HEX25E_CONFIG_ROUTE_NODE_COUNT;
    return g_shortest_route;
}

bool Hex25E_Map_GetRouteHeading(size_t edge_index, uint8_t *heading)
{
    if ((edge_index >= HEX25E_CONFIG_ROUTE_EDGE_COUNT) ||
        (heading == NULL)) {
        return false;
    }
    *heading = g_route_heading[edge_index];
    return true;
}

bool Hex25E_Map_GetRouteEdgeLength(size_t edge_index, float *length_mm)
{
    if ((edge_index >= HEX25E_CONFIG_ROUTE_EDGE_COUNT) ||
        (length_mm == NULL)) {
        return false;
    }
    /* O->A uses its independently tuned launch distance rather than the
     * regular hexagon-side distance. */
    *length_mm = (edge_index == 0U) ?
        HEX25E_CONFIG_MAP_START_TO_FIRST_Y_MM :
        HEX25E_CONFIG_MAP_SIDE_MM;
    return true;
}

bool Hex25E_Map_ValidateShortestRoute(void)
{
    uint32_t visited_y = 0U;
    size_t u_turn_count = 0U;

    if ((g_shortest_route[0] != HEX25E_NODE_O) ||
        (g_shortest_route[HEX25E_CONFIG_ROUTE_NODE_COUNT - 1U] !=
            HEX25E_NODE_O)) {
        return false;
    }
    for (size_t edge = 0U; edge < HEX25E_CONFIG_ROUTE_EDGE_COUNT; ++edge) {
        const Hex25E_Node *node = &g_nodes[g_shortest_route[edge]];
        bool found = false;
        for (size_t branch = 0U; branch < node->degree; ++branch) {
            if ((node->neighbor[branch] == g_shortest_route[edge + 1U]) &&
                (node->heading[branch] == g_route_heading[edge])) {
                found = true;
            }
        }
        if (!found) {
            return false;
        }
        {
            float edge_length_mm;
            if (!Hex25E_Map_GetRouteEdgeLength(edge, &edge_length_mm) ||
                !(edge_length_mm > 0.0F)) {
                return false;
            }
        }
        if ((edge > 0U) && (edge < HEX25E_CONFIG_ROUTE_EDGE_COUNT)) {
            uint8_t incoming = g_route_heading[edge - 1U];
            uint8_t outgoing = g_route_heading[edge];
            uint8_t change = (uint8_t)((outgoing + 6U - incoming) % 6U);
            if (change == 3U) {
                if (g_shortest_route[edge] != HEX25E_NODE_M) {
                    return false;
                }
                ++u_turn_count;
            }
            else if ((change != 1U) && (change != 5U)) {
                return false;
            }
        }
        if (Hex25E_Map_IsYJunction(g_shortest_route[edge])) {
            visited_y |= UINT32_C(1) << (unsigned int)g_shortest_route[edge];
        }
    }
    const uint32_t all_y = (UINT32_C(1) << HEX25E_NODE_A) |
        (UINT32_C(1) << HEX25E_NODE_F) |
        (UINT32_C(1) << HEX25E_NODE_M) |
        (UINT32_C(1) << HEX25E_NODE_C) |
        (UINT32_C(1) << HEX25E_NODE_H);
    if (((visited_y & all_y) != all_y) || (u_turn_count != 1U)) {
        return false;
    }
    return true;
}

#undef X
