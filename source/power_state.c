/*
 * power_state.c - Power-on sequencing state machine
 */

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "board.h"
#include "power_state.h"
#include "sensor.h"

/* Task handles - used for notifications between timer callbacks and tasks */
static TaskHandle_t xTaskSoftStart = NULL;
static TaskHandle_t xTaskPFCControl = NULL;
static TaskHandle_t xTaskDCControl = NULL;

/* Timer handles */
static TimerHandle_t xRelayTimer = NULL;
static TimerHandle_t xPFCTimer = NULL;
static TimerHandle_t xDCTimer = NULL;

/* State variables */
static PowerState_t g_powerState = POWER_STATE_INIT;
static bool g_relayStatus = false;
static bool g_pfcStatus = false;

/* Hardware control helpers */
static void prvEnableSoftStart(bool enable)
{
    if (enable) {
        GPIO_PortSet(SS_RLY_EN_GPIO, 1u << SS_RLY_EN_GPIO_PIN);
    }
}

static void prvEnablePFC(bool enable)
{
    if (enable) {
        GPIO_PortSet(PFC1_EN_GPIO, 1u << PFC1_EN_GPIO_PIN);
        GPIO_PortSet(PFC2_EN_GPIO, 1u << PFC2_EN_GPIO_PIN);
    }
}

static void prvEnableDC(bool enable)
{
    if (enable) {
        GPIO_PortSet(DCDC1_EN_GPIO, 1u << DCDC1_EN_GPIO_PIN);
        GPIO_PortSet(DCDC2_EN_GPIO, 1u << DCDC2_EN_GPIO_PIN);
    }
}

static bool prvCheckInputVoltage(void)
{
    return (Vrms.real >= 90.0f && Vrms.real <= 265.0f);
}

/* Timer callback prototypes */
static void prvRelayTimerCallback(TimerHandle_t xTimer);
static void prvPFCTimerCallback(TimerHandle_t xTimer);
static void prvDCTimerCallback(TimerHandle_t xTimer);

/* --- Public state queries --- */

PowerState_t prvGetPowerState(void)
{
    return g_powerState;
}

bool prvIsRelayEnabled(void)
{
    return g_relayStatus;
}

bool prvIsPFCEnabled(void)
{
    return g_pfcStatus;
}

/* --- Timer callbacks --- */

static void prvRelayTimerCallback(TimerHandle_t xTimer)
{
    (void)xTimer;
    prvEnableSoftStart(true);
    g_relayStatus = true;

    if (xTaskSoftStart != NULL) {
        xTaskNotifyGive(xTaskSoftStart);
    }
}

static void prvPFCTimerCallback(TimerHandle_t xTimer)
{
    (void)xTimer;
    PRINTF("PFC delay done, starting PFC\r\n");
    if (xTaskPFCControl != NULL) {
        xTaskNotifyGive(xTaskPFCControl);
    }
}

static void prvDCTimerCallback(TimerHandle_t xTimer)
{
    (void)xTimer;
    PRINTF("DC delay done, starting DC\r\n");
    if (xTaskDCControl != NULL) {
        xTaskNotifyGive(xTaskDCControl);
    }
}

/* --- Task functions --- */

void prvSelfCheckTask(void *pvParameters)
{
    (void)pvParameters;
    bool voltageOK, tempOK;

    for (;;) {
        voltageOK = prvCheckInputVoltage();
        tempOK = true;

        if (voltageOK && tempOK) {
            g_powerState = POWER_STATE_SOFTSTART;

            xRelayTimer = xTimerCreate(
                "RelayTimer",
                pdMS_TO_TICKS(20),
                pdFALSE,
                (void *)0,
                prvRelayTimerCallback
            );
            if (xRelayTimer != NULL) {
                xTimerStart(xRelayTimer, 0);
            }

            xTaskCreate(prvSoftStartTask, "SoftStart", 256, NULL, 2, &xTaskSoftStart);
            vTaskDelete(NULL);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void prvSoftStartTask(void *pvParameters)
{
    (void)pvParameters;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        g_powerState = POWER_STATE_PFC_READY;

        xPFCTimer = xTimerCreate(
            "PFCTimer",
            pdMS_TO_TICKS(20),
            pdFALSE,
            (void *)1,
            prvPFCTimerCallback
        );
        if (xPFCTimer != NULL) {
            xTimerStart(xPFCTimer, 0);
        }

        vTaskSuspend(NULL);
    }
}

void prvPFCControlTask(void *pvParameters)
{
    (void)pvParameters;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        prvEnablePFC(true);
        g_pfcStatus = true;
        g_powerState = POWER_STATE_RUNNING;
        PRINTF("PFC started, system running\r\n");

        xDCTimer = xTimerCreate(
            "DCTimer",
            pdMS_TO_TICKS(10),
            pdFALSE,
            (void *)1,
            prvDCTimerCallback
        );
        if (xDCTimer != NULL) {
            xTimerStart(xDCTimer, 0);
        }

        vTaskSuspend(NULL);
    }
}

void prvDCControlTask(void *pvParameters)
{
    (void)pvParameters;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        prvEnableDC(true);
        PRINTF("DC started, system running\r\n");

        vTaskSuspend(NULL);
    }
}
