#pragma once

#include "FreeRTOS.h"
#include "queue.h"
#include "moveCommand.h"  // We need MoveCommand type

void movementTaskInit(void);
bool movementTaskTest(void);
QueueHandle_t movementTaskGetQueueHandle(void);
