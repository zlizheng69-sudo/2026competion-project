#ifndef CAMERA_H
#define CAMERA_H

#include <stdint.h>

typedef struct {
    int16_t  dx;
    int16_t  dy;
    uint8_t  detected;  /* 1=检测到球, 0=未检测到 */
    uint8_t  fresh;
} CameraData;

void Camera_Init(void);
void Camera_RxByte(uint8_t byte);
CameraData Camera_GetData(void);
CameraData Camera_PeekData(void);
void Camera_GetStats(uint32_t *pkt, uint32_t *err, uint32_t *bytes);

#endif
