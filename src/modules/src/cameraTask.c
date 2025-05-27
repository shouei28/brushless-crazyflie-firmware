#include "cameraTask.h"
#include "config.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include "static_mem.h"
#include "uart2.h"
#include "moveCommand.h"  // The struct we'll send to movementTask
#include "stabilizer_types.h" // Needed for setpoint_t
#include "system.h"
#include "crtp_commander_high_level.h" // Needed to set setpoints
#include <string.h>
#include "ledseq.h"
#include "led.h"
#include <stdio.h>

static QueueHandle_t movementTaskQueueHandle;
static void cameraTask(void *param);

STATIC_MEM_TASK_ALLOC(cameraTask, CAMERA_TASK_STACKSIZE);

static bool isInit = false;

static const uint32_t BAUD_RATE = 115200;
static const int bufferSize = 8;  // 8 bytes: vx, vy, vz, yawRate (2 bytes each)

void cameraTaskInit(QueueHandle_t sendQueue) {
    movementTaskQueueHandle = sendQueue;

    uart2Init(BAUD_RATE);

    STATIC_MEM_TASK_CREATE(cameraTask, cameraTask, CAMERA_TASK_NAME, NULL, CAMERA_TASK_PRI);
    isInit = true;
}

bool cameraTaskTest(void) {
    return isInit;
}

static void cameraTask(void *param) {
    DEBUG_PRINT("✅ cameraTask started\n");
    systemWaitStart();

    while (true) {
        char startByte = 0;
        uint8_t readBuffer[10];  // Buffer to store incoming packet
        MoveCommand command;
    
        // Step 1: Wait for a start byte
        do {
            uart2GetDataWithTimeout(1, (uint8_t*)&startByte, M2T(10));  // Wait 10 ms per byte
        } while (startByte != 's');  // or whatever start marker you use
    
        // Step 2: Read the full data packet (after 's'), 8 bytes expected
        int bytesReceived = uart2GetData(bufferSize, readBuffer);
        if (bytesReceived >= bufferSize) {
            // ✅ Optional: check end byte or pattern here if you want
    
            // Step 3: Parse command
            memcpy(&(command.vx), &readBuffer[0], 2);
            memcpy(&(command.vy), &readBuffer[2], 2);
            memcpy(&(command.vz), &readBuffer[4], 2);
            memcpy(&(command.yawRate), &readBuffer[6], 2);
    
            // Step 4: Debug and send
            DEBUG_PRINT("✅ MoveCommand: vx=%d vy=%d vz=%d yaw=%d\n", command.vx, command.vy, command.vz, command.yawRate);
            xQueueSend(movementTaskQueueHandle, &command, pdMS_TO_TICKS(0));
    
            // Optional: blink LED for visibility
            ledSet(LED_GREEN_L, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            ledSet(LED_GREEN_L, 0);
        }
    
        // Small delay to prevent overload
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
