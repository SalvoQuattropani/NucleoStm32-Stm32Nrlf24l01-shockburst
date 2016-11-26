
#include "stm32f3xx_hal.h"


/* Private variables ---------------------------------------------------------*/

#define CE_L() HAL_GPIO_WritePin(GPIOC, SPI1_CE_Pin, GPIO_PIN_RESET)
#define CE_H() HAL_GPIO_WritePin(GPIOC, SPI1_CE_Pin, GPIO_PIN_SET)
#define CSN_L() HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_RESET)
#define CSN_H() HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_SET)
#define NOP_MASK			0xFF



//REGISTRI
uint8_t config         = 0x00;
uint8_t en_aa          = 0x01;
uint8_t en_rxaddr      = 0x02;
uint8_t setup_aw       = 0x03;
uint8_t setup_retr     = 0x04;
uint8_t rf_ch          = 0x05;
uint8_t rf_setup       = 0x06;
uint8_t tx_addr        = 0x10;
uint8_t fifo_status    = 0x17;
uint8_t dynpd          = 0x1C;
uint8_t feature        = 0x1D;
uint8_t status         = 0x07;
uint8_t observe_tx     = 0x08;
uint8_t rx_addr_p0     = 0x0A;
uint8_t rx_pw_p0       = 0x11;
//COMANDI PER LA SPI
uint8_t w_register      = 0x20;
uint8_t r_rx_payload    = 0x61;
uint8_t w_tx_payload    = 0xA0;
uint8_t w_tx_payload_na = 0xB0;
uint8_t flush_tx        = 0xE1;
uint8_t flush_rx        = 0xE2;
uint8_t reuse_tx_pl     = 0xE3;




SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void MX_NVIC_Init(void);
//-------------
void flushTx(void);
void memset(uint8_t *, int);

//-------------REG_SET; ADDR; PAYLOAD
uint8_t reg_set[0x1E];
uint8_t addr1[5];
uint8_t addr2[5];
uint8_t rx_buffer[200];








//-------------SETUP------------
uint8_t set_en_aa = 0x01;
uint8_t set_en_rxaddr = 0x01;
uint8_t set_rf_ch = 0x50;
uint8_t set_rf_setup = 0x06;
uint8_t set_config = 0x3E;//3F per l trasmissione
uint8_t set_feature = 0x04;
uint8_t set_retr = 0x00;
uint8_t set_dynpd = 0x01;
uint8_t set_rx_pw_p0 = 0x06;
uint8_t payload_rx[6]={0x00,0x00, 0x00, 0x00, 0x00, 0x00};
  
  
  
  
//INDIRIZZI
uint8_t addr_f0[5] = {0xAA, 0x00, 0x00, 0x00, 0x01};
uint8_t addr_nucleo[5] = {0xAA, 0x00, 0x00, 0x00, 0x00};





uint8_t var1[6] = {0x01, 0x01, 0x00, 0x00, 0x00, 0x00};
uint8_t var2[6] = {0x01, 0x02, 0x00, 0x00, 0x00, 0x00};


uint8_t led_g_on[6] = {0x03, 0x03, 0x01};
//uint8_t led_g_on[6] = {0x03, 0x01, 0x03, 0x03, 0x03, 0x03};
//uint8_t led_b_on[6] = {0x03, 0x03, 0x02, 0x03, 0x03, 0x03};
uint8_t led_b_on[6] = {0x03, 0x03, 0x02};

uint8_t led_g_off[6] = {0x03, 0x01, 0x00, 0x00, 0x00, 0x00};
uint8_t led_b_off[6] = {0x03, 0x02, 0x00, 0x00, 0x00, 0x00};










uint8_t stato;

void scrivi_reg(uint8_t reg, uint8_t data)
{
  stato = reg + w_register;
  CSN_L();
  HAL_SPI_Transmit(&hspi1, &stato, 1, 100);
  HAL_SPI_Transmit(&hspi1, &data, 1, 100);
  CSN_H();
}



void leggi_reg(uint8_t reg, uint8_t i)
{
  CSN_L();
  HAL_SPI_Transmit(&hspi1, &reg, 1, 100);
  HAL_SPI_Receive(&hspi1, &reg_set[i], 1, 100);
  CSN_H();
}

void invia_comando(uint8_t cmd)
{
  CSN_L();
  HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
  CSN_H();
}

void power_up()
{
  scrivi_reg(config, set_config);
  scrivi_reg(en_aa, set_en_aa);
  scrivi_reg(en_rxaddr, set_en_rxaddr);
  scrivi_reg(setup_retr, set_retr);
  scrivi_reg(rf_ch, set_rf_ch);
  scrivi_reg(rf_setup, set_rf_setup);
  scrivi_reg(dynpd, set_dynpd);
  scrivi_reg(feature, set_feature); 
  invia_comando(reuse_tx_pl);
}

void power_down()
{
  scrivi_reg(config, reg_set[0]- 0x02);
}

void stato_registri()
{
  
  leggi_reg(config, 0x00);//registro config
  leggi_reg(en_aa, 0x01); //registro en_aa
  leggi_reg(en_rxaddr, 0x02);  //registro en_rxaddr
  leggi_reg(setup_aw, 0x03);  //registro en_rxaddr
  leggi_reg(setup_retr, 0x04);  //registro setup_retr
  leggi_reg(rf_ch, 0x05);  //registro rf_ch
  leggi_reg(rf_setup, 0x06);  //registro rf_setup
  leggi_reg(status, 0x07);  //registro status
  leggi_reg(fifo_status, 0x17); //registro fifo_status 
  leggi_reg(dynpd, 0x1C);//registro dynpd
  leggi_reg(feature, 0x1D); //registro feature
  leggi_reg(rx_pw_p0, 0x11); //registro rx_pw_p0
  
  //------------------registro rx_addr_p0---------------------------
  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &rx_addr_p0, 1, 10);
  HAL_SPI_Receive(&hspi1, addr1, 5, 10);
  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_SET);
  
  //--------------------Lettura registro tx_addr-------------------------
  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &tx_addr, 1, 10);
  HAL_SPI_Receive(&hspi1, addr2, 5, 10);
  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_SET);  
  
  
  
  
  

  
  
  
}

void setAddr()
{
  //SET TX ADDRESS
  stato = w_register + tx_addr;
  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &stato, 1, 10);
  for(int j = 0; j <= 4; j++)
  {
    HAL_SPI_Transmit(&hspi1, &addr_f0[j], 1, 10);
  }
  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_SET);
  
  //SET RX_P0 ADDRESS
  stato = w_register + rx_addr_p0;
  
  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &stato, 1, 10);
  for(int k = 0; k <= 4; k++)
  {
    HAL_SPI_Transmit(&hspi1, &addr_nucleo[k], 1, 10);
  }
  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_SET);
} 


uint8_t appogg[2];

void addPayload(int var)
{
  
  //SET PAYLOAD
  
  HAL_GPIO_WritePin(GPIOA, SPI1_CE_Pin, GPIO_PIN_RESET);
  
  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &w_tx_payload, 1, 10);
  for(int i = 0; i<=5; i++)
  {
    
    if(var==1){
   HAL_SPI_Transmit(&hspi1, &var1[i], 1, 10);
  }
     if(var==2){
   HAL_SPI_Transmit(&hspi1, &var2[i], 1, 10);
  } 
    
    
  }
  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_SET);
} 


void SET_VALUE(uint8_t var)
{
  
  //SET PAYLOAD
  
  HAL_GPIO_WritePin(GPIOA, SPI1_CE_Pin, GPIO_PIN_RESET);
  
  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &w_tx_payload, 1, 10);
  for(int i = 0; i<=5; i++)
  {
    
    if(var ==1){
   HAL_SPI_Transmit(&hspi1, &var1[i], 1, 10);
  }
     if(var == 2){
   HAL_SPI_Transmit(&hspi1, &var2[i], 1, 10);
  } 
   
  }
  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_SET);
} 


int cont=0;

void SET_LED()
{
  

  
  HAL_GPIO_WritePin(GPIOA, SPI1_CE_Pin, GPIO_PIN_RESET);
  
  
  if(cont==0){
  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &w_tx_payload, 1, 10);
  for(int i = 0; i<=5; i++)
  {
  HAL_SPI_Transmit(&hspi1, &led_b_on[i], 1, 10);}
  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_SET);
  cont=1;
  }
  else
  {
  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_RESET);
  for(int i = 0; i<=5; i++)
  {
  HAL_SPI_Transmit(&hspi1, &led_g_on[i], 1, 10);}
  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_SET);
  cont=0;
  }
  
//  
//   if(cont==1){
//  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_RESET);
//  HAL_SPI_Transmit(&hspi1, &w_tx_payload, 1, 10);
//  for(int i = 0; i<=5; i++)
//  {
//  HAL_SPI_Transmit(&hspi1, &led_g_off[i], 1, 10);}
//  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_SET);
//      
//  
//  
//  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_RESET);
//  for(int i = 0; i<=5; i++)
//  {
//  HAL_SPI_Transmit(&hspi1, &led_b_off[i], 1, 10);}
//  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_SET);
//  cont=0;
//  }
//  
  
   
  
 
  
} 


void readPayload()
{
  

  
 
  
  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &r_rx_payload, 1, 100);

    HAL_SPI_Receive(&hspi1, payload_rx, 6, 100);
 
  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_SET);
} 





void tx()
{
  
  
  
  HAL_GPIO_WritePin(GPIOA, SPI1_CE_Pin, GPIO_PIN_SET);
  HAL_Delay(1);

  HAL_GPIO_WritePin(GPIOA, SPI1_CE_Pin, GPIO_PIN_RESET);

  flushTx();
  
  
} 



void flushRx()
{
  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &flush_rx, 1, 10);
  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_SET);  
}







void rx()
{
  




  HAL_GPIO_WritePin(GPIOA, SPI1_CE_Pin, GPIO_PIN_SET);



  
  HAL_GPIO_WritePin(GPIOA, SPI1_CE_Pin, GPIO_PIN_RESET);
 
  readPayload();
  stato_registri();
  
  
    uint8_t cmd[] = "command:";
    uint8_t id[] = " id var:";
    uint8_t valore[] = " value:";
    uint8_t cp = '\n';
    HAL_UART_Transmit(&huart2, cmd, 8, 10);
    HAL_UART_Transmit(&huart2, &payload_rx[0], 1, 10);
    HAL_UART_Transmit(&huart2, id, 8, 10);
    HAL_UART_Transmit(&huart2, &payload_rx[1], 1, 10);
    HAL_UART_Transmit(&huart2, valore, 7, 10);
    HAL_UART_Transmit(&huart2, &payload_rx[2], 1, 10);
    HAL_UART_Transmit(&huart2, &cp, 1, 10);
    flushRx();

} 








void flushTx()
{
  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &flush_tx, 1, 10);
  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_SET);  
}






void Max_retx_reset()
{
  scrivi_reg(status, 0x70);  
}

void clearTx_ds()
{
  stato = w_register + status;
  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &stato, 1, 10);
  HAL_SPI_Transmit(&hspi1, &reg_set[7], 1, 10);
  HAL_GPIO_WritePin(GPIOC, SPI1_CSN_Pin, GPIO_PIN_SET);  
}

void init_tx(){
    addPayload(1);
    scrivi_reg(config, 0x0E);
    scrivi_reg(rf_ch, set_rf_ch);
    stato_registri();

}



void Clear_Interrupts(void) {
	scrivi_reg(0x07, 0x70);
}

void init_rx(){
  power_up();
  clearTx_ds();
  stato_registri();
  flushRx();
  stato_registri();
  Clear_Interrupts(); //no interrupts
  stato_registri();
  scrivi_reg(config, 0x3F);
  scrivi_reg(rx_pw_p0, set_rx_pw_p0);
  stato_registri();

}





int main(void)
{
  

  

  
  
  
  
  HAL_Init();


  SystemClock_Config();


  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();


  MX_NVIC_Init();



  while (1)
  {
  //----------------inizio prima tx-----  
  set_config=0x3E; 
  power_up();
  stato_registri();
  SET_VALUE(1); //TIM
  stato_registri();
  Max_retx_reset();
  stato_registri();
  setAddr();
  stato_registri();
  init_tx();
  stato_registri();
  tx();
   //----------------inizio prima rx-----   
  
  stato_registri();
  Max_retx_reset();
  power_down();
  stato_registri();
  HAL_Delay(1000);
  set_config=0x3F; //mi metto in ricezione
  init_rx();
  rx();
 
    //----------------inizio seconda tx-----  
  set_config=0x3E; 
  power_up();
  stato_registri();
  SET_VALUE(2); //INC
   
  stato_registri();
  Max_retx_reset();
  stato_registri();
  setAddr();
  stato_registri();
  init_tx();
  stato_registri();
  tx();
   //----------------inizio seconda rx-----   
  
  stato_registri();
  Max_retx_reset();
  power_down();
  stato_registri();
  HAL_Delay(1000);
  set_config=0x3F; //mi metto in ricezione
  init_rx();
  rx();
    
     //----------------inizio terza tx-----  
  set_config=0x3E; 
  power_up();
  stato_registri();
  SET_LED(); //LED
  stato_registri();
  Max_retx_reset();
  stato_registri();
  setAddr();
  stato_registri();
  init_tx();
  stato_registri();
  tx();

    
    
    
   
  
  
  
  
  }


}





/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** NVIC Configuration
*/
void MX_NVIC_Init(void)
{
  /* SPI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  HAL_SPI_Init(&hspi1);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart1);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CSN_Pin */
  GPIO_InitStruct.Pin = SPI1_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CSN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CE_Pin */
  GPIO_InitStruct.Pin = SPI1_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_IRQ_Pin */
  GPIO_InitStruct.Pin = SPI1_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI1_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CE_GPIO_Port, SPI1_CE_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
