#pragma once

#include "FreeRTOS.h"
#include "queue.h"

void cameraTaskInit(QueueHandle_t sendQueue);
bool cameraTaskTest(void);
