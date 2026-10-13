#ifndef HOST_MAIN_H
#define HOST_MAIN_H
#include <stdint.h>
typedef struct { void *Instance; } TIM_HandleTypeDef;
#define TIM3 ((void *)3)
#define TIM4 ((void *)4)
extern TIM_HandleTypeDef htim3, htim4;
#define led1_GPIO_Port ((void *)1)
#define led1_Pin 1U
uint32_t __get_PRIMASK(void);
uint32_t __get_IPSR(void);
uint32_t __get_BASEPRI(void);
void __disable_irq(void);
void __enable_irq(void);
uint32_t HAL_GetTick(void);
void HAL_IncTick(void);
void HAL_GPIO_TogglePin(void *port, uint32_t pin);
void Error_Handler(void);
#endif
