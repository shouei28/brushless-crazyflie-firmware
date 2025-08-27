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
#include "supervisor.h"
#include "motors.h"
#include "stabilizer.h"
#include "log.h"
#include "estimator.h"

#define LOG_FLOAT  7

// Global variables to log
float camera_vx = 0;
float camera_vy = 0;
float camera_vz = 0;
float camera_yaw_rate = 0;
float zHeight = 0.5f;

// Register them for logging
LOG_GROUP_START(camera_input)
LOG_ADD(LOG_FLOAT, vx, &camera_vx)
LOG_ADD(LOG_FLOAT, vy, &camera_vy)
LOG_ADD(LOG_FLOAT, vz, &camera_vz)
LOG_ADD(LOG_FLOAT, yaw_rate, &camera_yaw_rate)
LOG_GROUP_STOP(camera_input)

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;


  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;


  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;

  setpoint->velocity_body = true;
}

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

    static setpoint_t setpoint;

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
    float height = 0.0f;
    while (height < zHeight) {
        // increase gradually
        setHoverSetpoint(&setpoint, 0.0f, 0.0f, height, 0.0f);     
        commanderSetSetpoint(&setpoint, 5);
        if(height == 0.0f){
            vTaskDelay(M2T(500));
        } else {
            vTaskDelay(M2T(20));
        }
        height += 0.01f; 
    }
    // Hover for 0.5 seconds
    for (int i = 0; i < 50; i++) { 
        setHoverSetpoint(&setpoint, 0.0f, 0.0f, zHeight, 0.0f);
        commanderSetSetpoint(&setpoint, 5);
        vTaskDelay(M2T(10));
    }
    while (true) {
        MoveCommand cmd;

        if (pdTRUE == xQueueReceive(movementCommandQueue, &cmd, portMAX_DELAY)) {  
            if(!cmd.move) {
                float height = zHeight;
                while (height > 0.05f) {
                    height -= 0.05f;
                    setHoverSetpoint(&setpoint, 0.0f, 0.0f, height, 0.0f);     
                    commanderSetSetpoint(&setpoint, 5);
                    vTaskDelay(M2T(10));
                }
                for(int i = 0; i < 5; i++){
                    height -= 0.01f;
                    setHoverSetpoint(&setpoint, 0.0f, 0.0f, height, 0.0f);     
                    commanderSetSetpoint(&setpoint, 5);
                    vTaskDelay(M2T(100));
                }
            } else{        
                float yawrate = (float)cmd.yawRate;
                camera_yaw_rate = yawrate;
                // Now send the setpoint to commander
                setHoverSetpoint(&setpoint, 0.35f, 0.0f, zHeight, yawrate);     
                commanderSetSetpoint(&setpoint, 5);
                vTaskDelay(M2T(10));
            }
        }

    }
}
