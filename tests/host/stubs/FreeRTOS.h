#ifndef HOST_FREERTOS_H
#define HOST_FREERTOS_H
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
typedef uint32_t TickType_t;
typedef unsigned UBaseType_t;
typedef int BaseType_t;
typedef unsigned configSTACK_DEPTH_TYPE;
typedef void (*TaskFunction_t)(void *);
typedef void *TaskHandle_t;
typedef struct HostQueue *QueueHandle_t;
typedef struct { void *handle; } StaticSemaphore_t;
typedef StaticSemaphore_t *SemaphoreHandle_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define pdMS_TO_TICKS(x) ((TickType_t)(x))
#define configASSERT(x) assert(x)
#define taskSCHEDULER_NOT_STARTED 0
#define taskSCHEDULER_RUNNING 2
QueueHandle_t xQueueCreate(UBaseType_t length, UBaseType_t size);
BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t wait);
BaseType_t xQueueOverwrite(QueueHandle_t queue, const void *item);
void vQueueDelete(QueueHandle_t queue);
BaseType_t xTaskCreate(TaskFunction_t fn, const char *name, configSTACK_DEPTH_TYPE size,
                       void *arg, UBaseType_t priority, TaskHandle_t *handle);
void vTaskDelete(TaskHandle_t handle);
void vTaskDelay(TickType_t ticks);
void vTaskDelayUntil(TickType_t *last, TickType_t period);
TickType_t xTaskGetTickCount(void);
BaseType_t xTaskGetSchedulerState(void);
SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t *storage);
BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t ticks);
BaseType_t xSemaphoreGive(SemaphoreHandle_t sem);
#endif
