#include "main.h"
#include "init.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart3;

void dfu_otter_bootloader(void);

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();

  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim3);


  HAL_GPIO_WritePin(GPIOB, LED_POWER_Pin, 1);

  char otter[50];
  memset(otter, sizeof(otter), ' ');
  
  if (HAL_GPIO_ReadPin(GPIOB, Button_Pin)) {
    dfu_otter_bootloader();
  }
  
  HAL_TIM_OnePulse_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_OnePulse_Start(&htim3, TIM_CHANNEL_ALL);

  while (1)
  {

    HAL_GPIO_TogglePin(GPIOB, LED_STATUS_Pin);
    HAL_Delay(500);

    TIM2->CR1 = TIM2->CR1 | 1;
    TIM3->CR1 = TIM3->CR1 | 1;
    
    memset(otter, sizeof(otter), ' ');
    sprintf(otter, "%d\n\r", HAL_GPIO_ReadPin(GPIOB, Button_Pin));
    CDC_Transmit_FS((uint8_t*)otter, sizeof(otter));
  }
}

void dfu_otter_bootloader(void)
{
  *((unsigned long *)0x20003FF0) = 0xDEADBEEF;
  NVIC_SystemReset();
}

void Error_Handler(void)
{
  while (1) {
    HAL_GPIO_TogglePin(GPIOB, LED_POWER_Pin);
    HAL_GPIO_TogglePin(GPIOB, LED_STATUS_Pin);
    HAL_Delay(100);
  }
}