/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MC20.h"
#include "can.h"
#include "bsp_usartx_CC2541.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PubTopic         "/sys/a1fkV4RfRSP/AQ_SCOOTER_TEST/thing/event/property/post"
char BLE_AT[]="AT\r\n";
char BLE_GET_VERSION[]="AT+VERION=?\r\n";
char BLE_GET_STATE[]="AT+SYS_STATE=?\r\n";
char BLE_SET_NAME[]="AT+NAME=";
char BLE_SET_PWD[]="AT+PSWD=";
char BLE_TX_POWER[]="AT+TX=";
char BLE_RX_POWER[]="AT+RX=";
char BLE_SET_ID[]="AT+ADV_MFR_SPC=424C455F4F42445F4151\r\n";// BLE_AQ
uint8_t isConnect=0;
uint8_t CANDataAvalFlag=0;
uint8_t BLEStopSendMsgDelayCount=0;
uint8_t SlaveConnected =0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern uint8_t gprsStatus;
extern uint8_t gnssDataAval;

uint32_t eventData=0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

osMessageQId cloudQueueHandle;
enum CLOUD_EVENT_ID
{
  CLOUD_NULL=0,
  CLOUD_UPLOAD_INFO,
  CLOUD_UPLOAD_GPS,
  CLOUD_UPLOAD_BAT_ERR,
  CLOUD_UPLOAD_INVERTER_ERR,
  GPS_INIT,
  READ_GPS_DATA,
 
};
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osMessageQId cloudQueueHandle;
osTimerId gpsTimerHandle;
osTimerId cloudTimerHandle;
osTimerId errorTimerHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void cloud_queue_exec(osEvent *osEvent);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void gpsTimerFunc(void const * argument);
void cloudTimerFunc(void const * argument);
void errorTimerFunc(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of gpsTimer */
  osTimerDef(gpsTimer, gpsTimerFunc);
  gpsTimerHandle = osTimerCreate(osTimer(gpsTimer), osTimerPeriodic, NULL);

  /* definition and creation of cloudTimer */
  osTimerDef(cloudTimer, cloudTimerFunc);
  cloudTimerHandle = osTimerCreate(osTimer(cloudTimer), osTimerPeriodic, NULL);

  /* definition and creation of errorTimer */
  osTimerDef(errorTimer, errorTimerFunc);
  errorTimerHandle = osTimerCreate(osTimer(errorTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of cloudQueue */
  osMessageQDef(cloudQueue, 50, uint32_t);
  cloudQueueHandle = osMessageCreate(osMessageQ(cloudQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  MC20_Init();
  GPSInit();
  MQTT_Start();
  mqttStatus =1;
  
  osTimerStart(errorTimerHandle,10000);
  osTimerStart(cloudTimerHandle,2000);
  osTimerStart(gpsTimerHandle,10000);
  /* Infinite loop */
  for(;;)
  {
     osEvent event;
     event = osMessageGet(cloudQueueHandle,0);
     if(osEventMessage == event.status)
     {
       cloud_queue_exec(&event);
     }
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* gpsTimerFunc function */
void gpsTimerFunc(void const * argument)
{
  /* USER CODE BEGIN gpsTimerFunc */
    if(gnssDataAval == 0)
    {       
       eventData = READ_GPS_DATA;
       osMessagePut(cloudQueueHandle,eventData,0); 
       return;
    }

  
     eventData = READ_GPS_DATA;
     osMessagePut(cloudQueueHandle,eventData,0); 
     osDelay(10); 
    
    
 
    if(gnssDataAval == 1)  // Data valid then upload.otherwise no upload
    {
      eventData = CLOUD_UPLOAD_GPS;
      osMessagePut(cloudQueueHandle,eventData,0); 
    }
  /* USER CODE END gpsTimerFunc */
}

/* cloudTimerFunc function */
void cloudTimerFunc(void const * argument)
{
  /* USER CODE BEGIN cloudTimerFunc */
     if(gprsStatus == 1)
     {
        eventData = CLOUD_UPLOAD_INFO;
        osMessagePut(cloudQueueHandle,eventData,0); 
     }
  /* USER CODE END cloudTimerFunc */
}

/* errorTimerFunc function */
void errorTimerFunc(void const * argument)
{
  /* USER CODE BEGIN errorTimerFunc */
 if(checkBatErr() == 1)
    {
      eventData = CLOUD_UPLOAD_BAT_ERR;
      osMessagePut(cloudQueueHandle,eventData,0); 
    }
   
   if(checkInverterErr() == 1)
   {
     eventData = CLOUD_UPLOAD_INVERTER_ERR;
     osMessagePut(cloudQueueHandle,eventData,0); 
   }
  /* USER CODE END errorTimerFunc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void stopAllTimer()
{
  

   osTimerStop(errorTimerHandle);
   osTimerStop(cloudTimerHandle);
   osTimerStop(gpsTimerHandle);
}





 

uint32_t event=0;
 void cloud_queue_exec(osEvent *osEvent)
 {
     
     event = (uint32_t)(osEvent->value.v);
 
     switch(event)
     {
        case CLOUD_UPLOAD_INFO:   
         pubLiveData();
         break;
        case CLOUD_UPLOAD_GPS:
         pubGPSData();
         break;
       case CLOUD_UPLOAD_INVERTER_ERR:
         pubInverterErrData();
         break;
       case CLOUD_UPLOAD_BAT_ERR:
         pubBatErrData();
         break;
       case READ_GPS_DATA:
          readGnssData();
          break;
       
       default:
       break;
    }
    
  
 }
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
