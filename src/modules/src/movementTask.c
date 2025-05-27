#include "movementTask.h"
#include "config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "static_mem.h"
#include "moveCommand.h"
#include "stabilizer_types.h" // Needed for setpoint_t
#include "crtp_commander_high_level.h" // Needed to set setpoints
#include "system.h"
#include "commander.h"
#include "debug.h"
#include <stdio.h>
#include <string.h>
#include "ledseq.h"
#include "led.h"
#include "supervisor.h"
#include "motors.h"


static QueueHandle_t movementCommandQueue;
static void movementTask(void *param);

STATIC_MEM_TASK_ALLOC(movementTask, MOVEMENT_TASK_STACKSIZE);

static bool isInit = false;

void movementTaskInit(void) {
    movementCommandQueue = xQueueCreate(5, sizeof(MoveCommand)); // Create a queue for 5 MoveCommands
    STATIC_MEM_TASK_CREATE(movementTask, movementTask, MOVEMENT_TASK_NAME, NULL, MOVEMENT_TASK_PRI);
    isInit = true;
}

bool movementTaskTest() {
    return isInit;
}

QueueHandle_t movementTaskGetQueueHandle(void) {
    return movementCommandQueue;
}

static void movementTask(void *param) {

    systemWaitStart();
    vTaskDelay(M2T(2000));  // Wait 2s for sensors to calibrate
    while (!supervisorCanArm()) {
        DEBUG_PRINT("⏳ Waiting to become armable...\n");
        vTaskDelay(M2T(500));
    }
    if (supervisorRequestArming(true)) {
        DEBUG_PRINT("✅ Arming succeeded\n");
    } else {
        DEBUG_PRINT("❌ Arming failed\n");
    }
    while (true) {
        MoveCommand cmd;
        ledSet(LED_GREEN_L, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        ledSet(LED_GREEN_L, 0);

        if (pdTRUE == xQueueReceive(movementCommandQueue, &cmd, portMAX_DELAY)) {
            // Optional: blink LED for visibility
            ledSet(LED_GREEN_L, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            ledSet(LED_GREEN_L, 0);

            // Build a local setpoint
            setpoint_t mySetpoint = {0};

            // Set mode to velocity control
            mySetpoint.mode.x = modeVelocity;
            mySetpoint.mode.y = modeVelocity;
            mySetpoint.mode.z = modeVelocity;
            mySetpoint.mode.yaw = modeVelocity;

            // Set velocity (converted to meters per second)
            mySetpoint.velocity.x = (float)cmd.vx / 1000.0f;
            mySetpoint.velocity.y = (float)cmd.vy / 1000.0f;
            mySetpoint.velocity.z = (float)cmd.vz / 1000.0f;
            mySetpoint.attitudeRate.yaw = (float)cmd.yawRate * (3.1415926f / 180.0f);
            mySetpoint.thrust = 20000;
            // Now send the setpoint to commander
            commanderSetSetpoint(&mySetpoint, 5);
            vTaskDelay(M2T(10));
        }

    }
}