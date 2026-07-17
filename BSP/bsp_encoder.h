#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * An encoder handle identifies one logical signed-count source. Its pins,
 * polarity, timer, and counting backend are private to the selected MCU BSP.
 */
typedef const struct BSP_Encoder_Resource *BSP_Encoder_Handle;

/* Return the number of logical encoders provided by this BSP; zero is valid. */
size_t BSP_Encoder_Count(void);

/*
 * Return a program-lifetime handle for index, or NULL when index is outside
 * BSP_Encoder_Count(). Indices describe BSP resources, not product roles.
 */
BSP_Encoder_Handle BSP_Encoder_GetHandle(size_t index);

/*
 * Atomically snapshot the signed cumulative count since BSP_Init().
 * This read is non-destructive and does not affect BSP_Encoder_ReadDelta().
 * Return false for a NULL/foreign handle or NULL output pointer. On failure,
 * cumulative_count is not modified.
 */
bool BSP_Encoder_GetSnapshot(BSP_Encoder_Handle encoder,
                            int64_t *cumulative_count);

/*
 * Atomically read and consume the signed count accumulated since this
 * handle's previous successful delta read (or BSP_Init() for its first
 * read). Consuming a delta never changes the cumulative snapshot. Use one
 * delta consumer per handle; multiple consumers would share the same window.
 * Return false for a NULL/foreign handle or NULL output pointer. On failure,
 * delta_count is not modified and the pending delta is not consumed.
 */
bool BSP_Encoder_ReadDelta(BSP_Encoder_Handle encoder, int64_t *delta_count);

#endif
