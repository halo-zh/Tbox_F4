/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "string.h"
#include "bsp_usartx_CC2541.h"
#include <stdio.h>
int fputc(int c, FILE *stream)
{
	USART3->DR = c;//将c赋给串口1的DR寄存器，即重定向到串口，也可以是其他的接口
	while(!(USART3->SR & (1 << 7))){};//等待数据发送完成
	return c;
}
/* USER CODE END 0 */

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 460800;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = MC20_RX_Pin|MC20_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PC10     ------> USART3_TX
    PC11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = DEBUG_TX_Pin|DEBUG_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, MC20_RX_Pin|MC20_TX_Pin);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PC10     ------> USART3_TX
    PC11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOC, DEBUG_TX_Pin|DEBUG_RX_Pin);

  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
int g_tx_error = 0;
HAL_StatusTypeDef K_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    HAL_StatusTypeDef status = HAL_OK;
   // for(int32_t i = 1; i < 100; ++i)
    {
        status = HAL_UART_Transmit_IT(huart, pData, Size);
        /*
        if(HAL_OK == status)
        {   
            return status;
        }
        else if(HAL_BUSY == status)
        {
            ////printf("HAL_UART_Transmit failed. status:%d, gState:0X%x, Lock:%d\r\n", status, huart->gState, huart->Lock);
            if(HAL_UART_STATE_READY == huart->gState && HAL_LOCKED == huart->Lock && i % 50 == 0)
            {   g_tx_error++;
                __HAL_UNLOCK(huart);
                continue;
            }
        }
        else if(HAL_ERROR == status)
        {   
            return status;
        }
        else if(HAL_TIMEOUT == status)
        {   
            continue;
        }
*/
    }
    return status;
}

void user_send_data_with_delay(char *buffer) {
	K_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer));
        osDelay(200);
}


#define RING_BUFFER_SIZE            (1000)
typedef struct
{
    uint8_t  data[RING_BUFFER_SIZE];
    uint16_t tail;
    uint16_t head;
} uart_ring_buffer_t;

static uart_ring_buffer_t   g_uart_rx_buf = {0};

#define AT_SUCCESS 0
#define AT_FAILED  -1

/**
 * Receive data on a UART interface
 *
 * @param[in]   uart         the UART interface
 * @param[out]  data         pointer to the buffer which will store incoming data
 * @param[in]   expect_size  number of bytes to receive
 * @param[out]  recv_size    number of bytes received
 * @param[in]   timeout      timeout in milisecond, set this value to HAL_WAIT_FOREVER
 *                           if you want to wait forever
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
int32_t _uart_recv(void  *uart, void *data, uint32_t expect_size,
                   uint32_t *recv_size, uint32_t timeout)
{
    uint32_t tmp, read_size;
    uint8_t *buf = (uint8_t *)data;
    uint32_t start_time, expired_time;

    start_time = HAL_GetTick();
    *recv_size = 0;

    expect_size =  expect_size > RING_BUFFER_SIZE ? RING_BUFFER_SIZE : expect_size;

    for (;;) {
        read_size = expect_size;

        tmp = 0;
        /* Loop until data received */
        while (read_size--)
        {
            if (g_uart_rx_buf.head != g_uart_rx_buf.tail)
            {
                /* serial data available, so return data to user */
                *buf++ = g_uart_rx_buf.data[g_uart_rx_buf.head++];
                tmp++;

                /* check for ring buffer wrap */
                if (g_uart_rx_buf.head >= RING_BUFFER_SIZE)
                {
                    /* Ring buffer wrap, so reset head pointer to start of buffer */
                    g_uart_rx_buf.head = 0;
                }
            }
        }

        *recv_size += tmp;
        expect_size -= tmp;

        if(expect_size == 0)
        {
            break;
        }

        expired_time = HAL_GetTick() - start_time;
        if (expired_time > timeout)
        {
            return -1;
        }
    }

    return 0;
}

int user_get_data_with_delay(char *buffer, unsigned int length, unsigned int delay) {
    uint32_t recv_size= 0;
    _uart_recv(NULL, buffer, length,&recv_size, delay);

    char * found = NULL;
    if(strlen(buffer) == 0) {
        return -9;
    }
   
    printf("%s",buffer);
    found = strstr(buffer, "OK\r\n");
    if(NULL != found) {
        return AT_SUCCESS;
    }
    return AT_FAILED;
}

int mc20_normal_check(char *in_cmd, unsigned int time) {
    
  printf("%s",in_cmd);
  user_send_data_with_delay(in_cmd);
#define BUF_LEN 200
    char buffer[BUF_LEN] = {0};
    int ret = user_get_data_with_delay(buffer, BUF_LEN, time);
    
    printf("%s",buffer);
		//////printf("sent AT cmd %s ret is %d\n", in_cmd, ret);
    return ret;
}

uint32_t jd=0;
uint32_t wd=0;
uint8_t gnssDataAval=0;
void getGnssData(char *buffer)
{
  char * ptr;
 
  jd=0;
  wd=0;
  if( strstr(buffer,"GNRMC"))
  {
    if(ptr = strstr(buffer,",A,"))
    {
      for(uint8_t i=0; i<20;i++)
      {
        if(ptr[i+3]=='.')
          i++;
        if(ptr[i+3]==',')
          break;
        wd = wd*10+ptr[i+3]-'0';
        
      }
      
    }
    
    
    if(ptr = strstr(buffer,",N,"))
    {
      for(uint8_t i=0; i<20;i++)
      {
         if(ptr[i+3]=='.')
          i++;
        if(ptr[i+3]==',')
          break;
        jd = jd*10+ptr[i+3]-'0';
        gnssDataAval= 1;
      }
      
    }
    
    
  }
  else
  {
     gnssDataAval =0;
    return;
  }
  
  
  
  
  
}

int mc20_read_gnss(char *in_cmd) {
    user_send_data_with_delay(in_cmd);
#define BUF_LEN 200
    char buffer[BUF_LEN] = {0};
    int ret = user_get_data_with_delay(buffer, BUF_LEN, 2000);
    if(ret != -9)
    {
      getGnssData(buffer);
    }
    
		
    return ret;
}

int mc20_check_sim_card() {
    user_send_data_with_delay("AT+CPIN?\r\n");
#define BUF_LEN 50
    char buffer[BUF_LEN] = {0};
    int ret = user_get_data_with_delay(buffer, BUF_LEN,200);
    if(strstr(buffer, "CPIN: READY") != NULL) {
        return AT_SUCCESS;
    }
    return AT_FAILED;
}

int mc20_check_timesync() {
    user_send_data_with_delay("AT+QGNSSTS?\r\n");
#define BUF_LEN 50
    char buffer[BUF_LEN] = {0};
    int ret = user_get_data_with_delay(buffer, BUF_LEN,2000);
    if(strstr(buffer, "QGNSSTS: 1") != NULL) {
        return AT_SUCCESS;
    }
    return AT_FAILED;
}


int mc20_check_cgatt() {
    user_send_data_with_delay("AT+CGATT?\r\n");
#define BUF_LEN 50
    char buffer[BUF_LEN] = {0};
    int ret = user_get_data_with_delay(buffer, BUF_LEN,100);
    if(strstr(buffer, "CGATT: 1") != NULL) {
        return AT_SUCCESS;
    }
    return AT_FAILED;
}

int mc20_check_creg() {
  
 
    user_send_data_with_delay("AT+CREG?\r\n");
#define BUF_LEN 50
    char buffer[BUF_LEN] = {0};
    int ret = user_get_data_with_delay(buffer, BUF_LEN,500);
    
    printf("%s\r\n",buffer);
    if((strstr(buffer, "CREG: 0,1") != NULL)||(strstr(buffer, "CREG: 0,5") != NULL))
    {
        return AT_SUCCESS;
    }
    return AT_FAILED;
}


int mc20_check_cops() {
    user_send_data_with_delay("AT+COPS?\r\n");
#define BUF_LEN 50
    char buffer[BUF_LEN] = {0};
    int ret = user_get_data_with_delay(buffer, BUF_LEN,100);
    if(strstr(buffer, "CREG: 0,") != NULL) {
        return AT_SUCCESS;
    }
    return AT_FAILED;
}


int mc20_list_cops() {
    user_send_data_with_delay("AT+COPS=?\r\n");
#define BUF_LEN 200
    char buffer[BUF_LEN] = {0};
    int ret = user_get_data_with_delay(buffer, BUF_LEN,50000);
     printf("%s",buffer);
    if(strstr(buffer, "COPS") != NULL) {
        return AT_SUCCESS;
    }
   
    return AT_FAILED;
}


int mc20_select_cops() {
    user_send_data_with_delay("AT+COPS=1,1,\"TM\"\r\n");
#define BUF_LEN 200
    char buffer[BUF_LEN] = {0};
    int ret = user_get_data_with_delay(buffer, BUF_LEN,50000);
     printf("%s",buffer);
    if(strstr(buffer, "OK") != NULL) {
        return AT_SUCCESS;
    }
   
    return AT_FAILED;
}

int mc20_check_mqttonline() {
    char * inbuffer = "AT+MQTTSTATE?\r\n";
    //HAL_UART_Transmit(&huart2, inbuffer, strlen(inbuffer),100 );
    user_send_data_with_delay(inbuffer);

#define BUF_LEN_1 512
    char buffer[BUF_LEN_1] = {0};
    int ret = user_get_data_with_delay(buffer, BUF_LEN_1, 200);

    if(strstr(buffer, "MQTTSTATE: 0") != NULL) {

        return AT_FAILED;
    }
    return AT_SUCCESS;
}




int g_rx_error=0;
int g_rx_other_error=0;

HAL_StatusTypeDef K_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    HAL_StatusTypeDef status = HAL_OK;
    for(int32_t i = 1; i < 1000; ++i)
    {
#if 1
        uint32_t isrflags   = READ_REG(huart->Instance->SR);
        if((__HAL_UART_GET_FLAG(huart, UART_FLAG_PE))!=RESET)
        {
            __HAL_UART_CLEAR_PEFLAG(huart);
        }
        if((__HAL_UART_GET_FLAG(huart, UART_FLAG_FE))!=RESET)
        {
            __HAL_UART_CLEAR_FEFLAG(huart);
        }
        if((__HAL_UART_GET_FLAG(huart, UART_FLAG_NE))!=RESET)
        {
            __HAL_UART_CLEAR_NEFLAG(huart);
        }
        if((__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))!=RESET)
        {
            //READ_REG(huart->Instance->CR1);//ORE???,????CR
            //__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);
            __HAL_UART_CLEAR_OREFLAG(huart);
        }
        if(HAL_UART_ERROR_NONE != huart->ErrorCode)
        {
            huart->ErrorCode = HAL_UART_ERROR_NONE;
					 
        }
#endif
        status = HAL_UART_Receive_IT(huart, pData, Size);
        if(HAL_OK == status)
        {
            return status;
        }
        else if(HAL_BUSY == status)
        {
            ////////printf("HAL_UART_Receive_IT failed. status:%d, RxState:0X%x, Lock:%d\r\n", status, huart->RxState, huart->Lock);
            if(HAL_UART_STATE_READY == huart->RxState && HAL_LOCKED == huart->Lock && i % 500 == 0)
            { 
                __HAL_UNLOCK(huart);
							  g_rx_error++;
                continue;
            }
        }
        else if(HAL_ERROR == status)
        { 
					  g_rx_other_error++;
            return status;
        }
        else if(HAL_TIMEOUT == status)
        {  
        }
    }
    if(HAL_OK != status)
    {
    }
    return status;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  
  if(huart == &huart2)
  {
    if (++g_uart_rx_buf.tail >= RING_BUFFER_SIZE) {
        g_uart_rx_buf.tail = 0;
    }
		
   K_UART_Receive_IT(&huart2, (uint8_t *)&g_uart_rx_buf.data[g_uart_rx_buf.tail], 1);
   
  }
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
