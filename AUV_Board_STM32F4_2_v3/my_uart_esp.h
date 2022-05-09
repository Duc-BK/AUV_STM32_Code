#ifndef _MY_UART_ESP_H_
#define _MY_UART_ESP_H_

#include "misc.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include <stdbool.h>
#include "my_mx28.h"
#include "my_bms24v40ah.h"
#include "my_keller_pa3.h"
#include "my_io.h"
#include "my_can.h"
#include "my_delay.h"
#include "my_adc.h"

void UART4_ESP_Config(void);
void Pack(void);
void Send_Data_ESP(void);
void Update_Fuzzy_Data(void);

#endif

