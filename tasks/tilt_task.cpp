
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

#include "rack_inteligente_parametros.h"
#include "log_vt100.h"
#include "rack_inteligente.h"
#include "rack_event_groups.h"
#include "tilt_task.hpp"
#include "MPU6050.h"
#include "I2C.hpp"

#include <math.h>

extern "C" {
extern EventGroupHandle_t xEventGroup;
extern environment_t environment;

static inline bool tilt();

static MPU6050 *mpu = nullptr;

void vTiltTask(void *pvParameters){
    LOG_INFO("[Tilt Task] Iniciando...");
    I2C *pMpuI2c = static_cast<I2C*>(pvParameters);

    mpu = new MPU6050(pMpuI2c);
    mpu->begin();
    bool lastTilt = false;
    for (;;) {

        bool tiltFlag = tilt();
        LOG_INFO("[Tilt Task] Tilt: %d", tiltFlag);
        if(tiltFlag != lastTilt){
            LOG_INFO("[Tilt Task] Tilt detectado!");
            environment.tilt = tiltFlag;
            xEventGroupSetBits(xEventGroup, xTiltBitsToWaitFor);
            lastTilt = tiltFlag;
        }

        vTaskDelay(pdMS_TO_TICKS(RACK_TILT_TASK_DELAY));
    }
}

// Implementação de detecção de pancadas usando aceleração do MPU6050
extern "C" bool tilt() {
    static float baseline = 1.0f; // ~1 g parado

    MPU6050::VECT_3D acc;
    mpu->getAccelleration(&acc);

    float mag = sqrtf(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);

    const float alpha = 0.01f;      // filtro lento para baseline
    baseline = baseline + alpha * (mag - baseline);

    float delta = mag - baseline;

    LOG_INFO("[Tilt Task.tilt] Delta: %f", delta);
    return (delta > TILT_THRESHOLD);
}
}
