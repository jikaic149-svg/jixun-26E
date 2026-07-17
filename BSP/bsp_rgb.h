#ifndef BSP_RGB_H
#define BSP_RGB_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} BSP_RGB_Pixel;

extern volatile uint8_t g_BSP_RGB_LastGRB[3];
extern volatile uint32_t g_BSP_RGB_FrameCount;

/* Initialize the selected BSP backend. This does not imply that a pixel exists. */
void BSP_RGB_Init(void);

/* Return the number of addressable pixels provided by this BSP; zero is valid. */
size_t BSP_RGB_Count(void);

/* Stage a pixel value. Return false when index is outside BSP_RGB_Count(). */
bool BSP_RGB_SetPixel(size_t index, BSP_RGB_Pixel color);

/* Transmit all staged pixels through the backend selected by this BSP. */
bool BSP_RGB_Show(void);

#endif
