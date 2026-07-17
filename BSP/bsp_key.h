#ifndef BSP_KEY_H
#define BSP_KEY_H

#include <stdbool.h>
#include <stddef.h>

/*
 * A key handle identifies one logical input. Its GPIO, polarity, and input
 * backend are private to the selected MCU BSP.
 */
typedef const struct BSP_Key_Resource *BSP_Key_Handle;

/* Return the number of logical key inputs provided by this BSP; zero is valid. */
size_t BSP_Key_Count(void);

/*
 * Return a program-lifetime handle for index, or NULL when index is outside
 * BSP_Key_Count(). Indices describe BSP resources, not pins or product roles.
 */
BSP_Key_Handle BSP_Key_GetHandle(size_t index);

/*
 * Sample one raw key after BSP_Init(). The BSP normalizes electrical
 * polarity so pressed is true for the active state. No debounce, timing, or
 * short/long/double-click recognition occurs here.
 *
 * Return false for a NULL/foreign handle or NULL output pointer. On failure,
 * the object addressed by pressed is not modified.
 */
bool BSP_Key_Read(BSP_Key_Handle key, bool *pressed);

#endif
