

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "OSAL_Timers.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "delay.h"
#include "GenericApp.h"
#include "DebugTrace.h"
#include "zcomdef.h"
#include "delay.h"
#include "zcl.h"
#include "zcl_general.h"




#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"
#include "hal_defs.h"
#include "IOT.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

/* RTOS */
#if defined( IAR_ARMCM3_LM )
#include "RTOS_App.h"
#endif  
extern IOT IOT_Status;
extern unsigned char isBUSY; //是否进入手动控制


extern void LCD_P8x16Str(unsigned char x, unsigned char y,unsigned char ch[]);
extern void HalLcdDisplayPercentBar( char *title, uint8 value );
extern void LCD_P16x16Ch(unsigned char x, unsigned char y, unsigned char N);
extern uint8 keysPressed;
uint8 keysPressed;
extern uint8 autoMode;
extern uint8_t mode;
uint8_t uartState = AUTO_MODE;
static uint8 manualMode = 0; // 0为自动模式，1为手动模式
static uint8 ledState = HAL_LED_MODE_OFF ; // LED灯状态，0为关闭，1为打开



/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
typedef struct {
  uint8 endpoint;
  uint16 profileId;
  uint16 deviceId;
  uint8 version;
} zclDeviceInfo_t;

typedef struct {
  zclDeviceInfo_t  deviceInfo;
  uint8             status;
} DeviceListItem_t;

/*********************************************************************
 * TYPEDEFS
 */
//#define LIGHT_SENSOR_PORT    P0
//#define LIGHT_SENSOR_PIN     BV(4)
//#define MOTION_SENSOR_PORT   P0
//#define MOTION_SENSOR_PIN    BV(7)
#define PHOTO_RESISTOR_PIN   P0_4
#define PIR_SENSOR_PIN       P0_7
#define KEYS_BLINK_LED      0x01
#define KEYS_SEND_MSG       0x02
#define KEYS_JOIN_REQ       0x04
#define KEYS_DISCOVERY_REQ  0x08
#define LED_OFF 0
#define LED_ON 1
#define ZCL_ON_OFF
#define COORDINATOR_PHOTOCELL_CLUSTERID   0x0001
#define COORDINATOR_PIR_CLUSTERID         0x0002
#define SAMPLE_SENSOR_EVT 0x8000
#define MAX_LEDS 3
bool leds[MAX_LEDS]; // 声明一个存储 LED 灯状态的布尔类型数组
static bool LEDState = false;
// 定义三个LED灯的开关状态数组
bool isLedOn[3] = {false, false, false}; // 默认都是关闭状态
char strTemp[32]; // Define strTemp as a character array of size 32






/*********************************************************************
 * GLOBAL VARIABLES
 */
// This list should be filled with Application specific Cluster IDs.
const cId_t GenericApp_ClusterList[GENERICAPP_MAX_CLUSTERS] =
{
  GENERICAPP_CLUSTERID
};

const SimpleDescriptionFormat_t GenericApp_SimpleDesc =
{
  GENERICAPP_ENDPOINT,              //  int Endpoint;
  GENERICAPP_PROFID,                //  uint16 AppProfId[2];
  GENERICAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  GENERICAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  GENERICAPP_FLAGS,                 //  int   AppFlags:4;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList,  //  byte *pAppInClusterList;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in GenericApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t GenericApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte GenericApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // GenericApp_Init() is called.
devStates_t GenericApp_NwkState;


byte GenericApp_TransID;  // This is the unique message ID (counter)

afAddrType_t GenericApp_DstAddr;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void GenericApp_HandleKeys( byte shift, byte keys );
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
static void GenericApp_SendTheMessage( void );
void ReSetMODULE(void);


#if defined( IAR_ARMCM3_LM )
static void GenericApp_ProcessRtosMessage( void );
#endif


#define MODULERESET       P0_6                            // P0.6口控制reset
/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */



/****************************************************************************
* 名    称: ReSetMODULE()
* 功    能: 低电平复位4g nbiot模块
* 入口参数: 无
* 出口参数: 无
****************************************************************************/
void ReSetMODULE(void)
{
  P0DIR |= 0x40;                  //P0.6定义为输出
   
  MODULERESET = 0;                  //低电平复位---------------------
  Delay_ms(1000);
  MODULERESET = 1;                  //高电平工作------------
  Delay_ms(2000);
}
void SendDeviceInfoToOtherDevices(uint8_t deviceInfo);

void SendDeviceInfoToOtherDevices(uint8_t deviceInfo) {
  // TODO: Implement the logic to send the device information to other devices
  // You can use Zigbee protocol or any other communication method here
  
  // Example implementation using Zigbee
  char buffer[10];
  sprintf(buffer, "%d", deviceInfo);
  afAddrType_t dstAddr;
  memset(&dstAddr, 0, sizeof(afAddrType_t));
  dstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;

  uint16_t len = strlen(buffer);
  uint8_t* buf = (uint8_t*)buffer;
  uint8_t handle = 0; // Replace with the correct handle value based on your actual implementation

//  AF_DataRequest(&dstAddr, &GenericApp_epDesc, GENERICAPP_CLUSTERID, len, buf, &GenericApp_TransID,
//                 AF_DISCV_ROUTE, AF_DEFAULT_RADIUS);
  AF_DataRequest(&GenericApp_DstAddr, &GenericApp_epDesc,GENERICAPP_CLUSTERID, len,buf,&GenericApp_TransID, AF_DISCV_ROUTE,
                     AF_DEFAULT_RADIUS);
      

  // Free the memory if dynamically allocated
  // osal_mem_free(buf);
}

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void GenericApp_Init( uint8 task_id )
{
  unsigned char tmp[10];
  GenericApp_TaskID = task_id;
  GenericApp_NwkState = DEV_INIT;
  GenericApp_TransID = 0;
// Configure P0_4 as input for light sensor and enable pull-up resistor
  P0SEL &= ~BV(4);  // Set as GPIO
  P0DIR &= ~BV(4);  // Set as input
  P0INP |= BV(4);   // Enable pull-up resistor

  // Configure P0_7 as input for PIR sensor and enable pull-up resistor
  P0SEL &= ~BV(7);  // Set as GPIO
  P0DIR &= ~BV(7);  // Set as input
  P0INP |= BV(7);   // Enable pull-up resistor

  // Configure P1_0-7 as output for LED lights
  P1SEL &= 0x00;
  P1DIR |= 0xFF;
  P1 |= 0xFF; // Turn off all LEDs
  HalUARTWrite(0,"初始化完毕\r\n", 13);

  // ...

  // Register GenericApp_Task task into the task list
  osal_set_event( task_id, START_DEVICE_EVT );
 
  

  
  ReSetMODULE();

  UartInitPort0();//初始化串口0打印调试信息
//  UartInitPort1(SerialCallBack);//初始化串口1对接4G或者NBIOT模块或者WiFi模块
//  IOT_Status.Status=0;
//  HalUARTWrite(0, "\r\n======CSTX-UART0 CSGSM======\r\n", 32);
//  osal_memset(tmp,0,10);
//  tmp[0] = HAL_UART_DMA+0x30;
//  tmp[1] = HAL_UART_ISR+0x30;
//  tmp[2] = HAL_UART_USB+0x30;
//  HalUARTWrite(0, tmp, 6);
//  HalUARTWrite(1, "AT+RST\r\n", 8);
  
  
  GenericApp_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  GenericApp_DstAddr.endPoint = GENERICAPP_ENDPOINT;
  GenericApp_DstAddr.addr.shortAddr = 0xFFFF;
  
  // Fill out the endpoint description.
  GenericApp_epDesc.endPoint = GENERICAPP_ENDPOINT;
  GenericApp_epDesc.task_id = &GenericApp_TaskID;
  GenericApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&GenericApp_SimpleDesc;
  GenericApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &GenericApp_epDesc );//这里是注册自身模块的端点与应用层任务进行联系

  // Register for all key events - This app will handle all key events
  RegisterForKeys( GenericApp_TaskID );



  ZDO_RegisterForZDOMsg( GenericApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( GenericApp_TaskID, Match_Desc_rsp );

#if defined( IAR_ARMCM3_LM )
  // Register this task with RTOS task initiator
  RTOS_RegisterApp( task_id, GENERICAPP_RTOS_MSG_EVT );
#endif
}
// 定义全局变量用于存储上一次的传感器状态
static uint8_t previous_pin4_state = 0;
static uint8_t previous_pin7_state = 0;
static void SampleSensor_CheckState(void)
{
  // 读取 P0_4 引脚的状态
  uint8_t pin4_state = (P0 & BV(4)) != 0;

  // 读取 P0_7 引脚的状态
  uint8_t pin7_state = (P0 & BV(7)) != 0;

  // 在这里进行传感器状态的检测和处理
  // ...

  // 如果 P0_4 或 P0_7 的电平发生变化，则触发 SAMPLE_SENSOR_EVT 事件
  if ((pin4_state != previous_pin4_state) || (pin7_state != previous_pin7_state))
  {
    previous_pin4_state = pin4_state;
    previous_pin7_state = pin7_state;
    osal_set_event(GenericApp_TaskID, SAMPLE_SENSOR_EVT);
  }
}

/*********************************************************************
 * @fn      GenericApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint8_t CalculateDistance(void);

uint8_t CalculateDistance(void) {
  // 在这里实现计算距离的逻辑
  // 根据与设备直接的距离确定LED灯数量
  // 返回LED灯数量

  // 示例：假设与设备直接的距离为distance，根据距离确定LED灯数量
  // 在此处编写距离计算的逻辑
  // 假设已知节点之间的连接关系，可以使用适当的数据结构（如邻接矩阵或邻接链表）表示网络拓扑结构

  // 假设网络拓扑关系如下：
  // 节点0：协调器
  // 节点1：终端1
  // 节点2：终端2
  // 节点3：终端3
  // 节点之间的连接关系为：0-1, 0-2, 2-3

  // 在这个示例中，距离定义为通过的节点数，而不考虑具体的物理距离
  // 可以根据实际情况修改距离的定义和计算逻辑

  // 节点之间的连接关系可以使用邻接矩阵或邻接链表表示
  // 这里使用一个简单的邻接矩阵来表示连接关系
  uint8_t distance = 0;
  uint8_t adjacencyMatrix[4][4] = {
    { 0, 1, 1, 0 },
    { 1, 0, 0, 0 },
    { 1, 0, 0, 1 },
    { 0, 0, 1, 0 }
  };

  // 遍历节点之间的连接关系，计算距离
  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = i + 1; j < 4; j++) {
      if (adjacencyMatrix[i][j] == 1) {
        distance++;
      }
    }
  }

  return distance;
}
uint16 GenericApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;
  zAddrType_t dstAddr;

  // Data Confirmation message fields
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          GenericApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          GenericApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
          sentTransID = afDataConfirm->transID;
          (void)sentEP;
          (void)sentTransID;

          // Action taken when confirmation is received.
          if ( sentStatus != ZSuccess )
          {
            // The data wasn't delivered -- Do something
          }
          break;

        case AF_INCOMING_MSG_CMD:
          GenericApp_MessageMSGCB( MSGpkt );//有数据包进来，会进入这个事件
          break;

        case ZDO_STATE_CHANGE:
          GenericApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (GenericApp_NwkState == DEV_ZB_COORD)
              || (GenericApp_NwkState == DEV_ROUTER)
              || (GenericApp_NwkState == DEV_END_DEVICE) )
          {
            // Start sending "the" message in a regular interval.
            osal_start_timerEx( GenericApp_TaskID,
                                GENERICAPP_SEND_MSG_EVT,
                                GENERICAPP_SEND_MSG_TIMEOUT );
          }
          break;
        
 

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
// 检测 SAMPLE_SENSOR_EVT 事件
  if (events & SAMPLE_SENSOR_EVT)
  {
    // 处理传感器状态的变化
    SampleSensor_CheckState();
    // 在手动模式下，当按键操作执行后，不执行其他操作
  if (manualMode == 1) {
    return (events ^ SAMPLE_SENSOR_EVT);
  }

  // 自动模式下，当光照超过阈值时，关闭所有LED灯
  else{
  if (P0_4 == TRUE) {
    HalLedSet(HAL_LED_ALL, HAL_LED_MODE_OFF);
    ledState = LED_OFF;
    HalUARTWrite(0,"关闭所有灯\r\n", 13);
  }

  // 自动模式下，当有人经过时，所有LED灯都会亮起并发送信息
  if (P0_4 == FALSE && P0_7 == TRUE) {
    HalLedSet(HAL_LED_ALL, HAL_LED_MODE_ON);

    dstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
    dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
    HalUARTWrite(0,"打开所有灯\r\n", 13);

    // 根据距离递减LED灯数量并发送信息
    uint8_t distance = CalculateDistance();
    uint8_t numLeds = MAX_LEDS - distance;
    if (numLeds > 0) {
      for (uint8_t i = 0; i < numLeds; i++) {
        HalLedSet(leds[i], HAL_LED_MODE_ON);
      }
    }

    // 向其他设备发送信息
    SendDeviceInfoToOtherDevices(numLeds);
  }
  }

    // 返回未处理的事件
    return (events ^ SAMPLE_SENSOR_EVT);
  }
  // Send a message out - This event is generated by a timer
  //  (setup in GenericApp_Init()).
//  if ( events & GENERICAPP_SEND_MSG_EVT )
//  {
//    
//    IOT_Init();                //IOT初始化获取链接卡号和注册状态
//    IOT_MQTTInit();            //IOT MQTT初始化 链接阿里云的初始化
//    isBUSY= !isBUSY;
//    GenericApp_SendTheMessage();//广播数据发送
//    
//    osal_start_timerEx( GenericApp_TaskID,
//                        GENERICAPP_SEND_MSG_EVT,
//                        GENERICAPP_SEND_MSG_TIMEOUT );
//
//    // return unprocessed events
//    return (events ^ GENERICAPP_SEND_MSG_EVT);
//  }

  
#if defined( IAR_ARMCM3_LM )
  // Receive a message from the RTOS queue
  if ( events & GENERICAPP_RTOS_MSG_EVT )
  {
    // Process message from RTOS queue
    GenericApp_ProcessRtosMessage();

    // return unprocessed events
    return (events ^ GENERICAPP_RTOS_MSG_EVT);
  }
#endif

  // Discard unknown events
  return 0;
}



/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      GenericApp_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */


static void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
      }
#if defined( BLINK_LEDS )
      else
      {
        // Flash LED to show failure
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
      }
#endif
      break;

    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            GenericApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            GenericApp_DstAddr.endPoint = pRsp->epList[0];

            // Light LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}

/*********************************************************************
 * @fn      GenericApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */


void GenericApp_HandleKeys( uint8 shift, uint8 keys )
{
  if (uartState != KEY_RELEASE) // 检查是否按下按键
  {
    return;
  }
  
  switch (keys)
  {
    case HAL_KEY_SW_1:
      if (manualMode == 1) // 处于手动模式
      {
        // 切换LED灯状态
        if (ledState == LED_OFF)
        {
          ledState = LED_ON;
          HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);
          HalLedSet(HAL_LED_2, HAL_LED_MODE_ON);
          HalLedSet(HAL_LED_3, HAL_LED_MODE_ON);
          HalUARTWrite(0,"打开所有灯\r\n", 13);
        }
        else
        {
          ledState = LED_OFF;
          HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
          HalLedSet(HAL_LED_2, HAL_LED_MODE_OFF);
          HalLedSet(HAL_LED_3, HAL_LED_MODE_OFF);
          HalUARTWrite(0,"关闭所有灯\r\n", 13);
        }
      }
      break;
    case HAL_KEY_SW_2:
      if (manualMode == 1) // 处于手动模式
      {
        // 切换手动模式和自动模式
        manualMode = 0;
        osal_start_timerEx(App_TaskID, GENERICAPP_AUTO_MODE_EVT, GENERICAPP_AUTO_MODE_TIMEOUT);
        HalLedSet(HAL_LED_2, HAL_LED_MODE_ON);
        HalUARTWrite(0,"切换自模式\r\n", 13);
        // 向其他设备发送自动模式状态信息
      }
      else // 处于自动模式
      {
        // 切换自动模式和手动模式
        manualMode = 1;
        osal_stop_timerEx(App_TaskID, GENERICAPP_AUTO_MODE_EVT);
        HalLedSet(HAL_LED_2, HAL_LED_MODE_OFF);
        HalUARTWrite(0,"切换手模式\r\n", 13);
        // 向其他设备发送手动模式状态信息
      }
      break;
    default:
      break;
  }
}


/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
static void sendData(uint16_t dstAddr, uint8_t* data, uint8_t len)
{
  afAddrType_t destAddr;
  destAddr.addr.shortAddr = dstAddr;
  destAddr.addrMode = afAddr16Bit;
  destAddr.endPoint = GENERICAPP_ENDPOINT;
  destAddr.panId = 0xFFFF;

  uint8_t transId = 0;
  AF_DataRequest(&destAddr, &GenericApp_epDesc, GENERICAPP_CLUSTERID, len, data,
                 &transId, AF_DISCV_ROUTE, AF_DEFAULT_RADIUS);

}

//int flagCount=0;//用来统计收到次数
//unsigned char  id=66,temp=0,humi=0,sensor1=0,sensor2=0,sensor3=0; //分别从终端发送过来的传感器数据
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
   if (pkt != NULL)
  {
    uint8_t* pMsg = pkt->cmd.Data;
    if (pMsg != NULL)
    {
      switch (pMsg[MSG_TYPE_IDX])
      {
        case GENERICAPP_MSG_TYPE_LIGHT:
          if (pMsg[MSG_DATA_IDX] == GENERICAPP_MSG_DATA_LIGHT_OFF)
          {
            // Turn off LED on this device
            HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);

            // Send message to other devices to turn off their LEDs
            uint8_t message[MSG_BUFFER_SIZE];
            message[MSG_TYPE_IDX] = GENERICAPP_MSG_TYPE_LIGHT;
            message[MSG_DATA_IDX] = GENERICAPP_MSG_DATA_LIGHT_OFF;
            message[MSG_ADDR_IDX] = BROADCAST_ADDRESS;
            message[MSG_LEN_IDX] = MSG_DATA_LEN;
            sendData(message[MSG_ADDR_IDX], message, message[MSG_LEN_IDX]);
          }
          break;

        case GENERICAPP_MSG_TYPE_MOTION:
          if (pMsg[MSG_DATA_IDX] == GENERICAPP_MSG_DATA_MOTION_DETECTED)
          {
            // Turn on LED on this device
            HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);

            // Calculate distance to other devices and turn off their LEDs one by one
            uint8_t message[MSG_BUFFER_SIZE];
            uint8_t distance = 1;
            message[MSG_TYPE_IDX] = GENERICAPP_MSG_TYPE_LIGHT;
            message[MSG_DATA_IDX] = GENERICAPP_MSG_DATA_LIGHT_ON;
            message[MSG_ADDR_IDX] = BROADCAST_ADDRESS;
            message[MSG_LEN_IDX] = MSG_DATA_LEN;
            while (distance < MAX_DISTANCE)
            {
              // Calculate destination address based on distance
              uint16_t destAddress = NLME_GetShortAddr() + distance;
              message[MSG_ADDR_IDX] = LO_UINT16(destAddress);
              message[MSG_ADDR_IDX + 1] = HI_UINT16(destAddress);

              // Send message to turn off LED on the device at the calculated destination address
              sendData(destAddress, message, message[MSG_LEN_IDX]);

              // Increment distance and wait for a short time
              distance++;
              Delay_ms(DELAY_TIME);
            }
          }
          break;

        default:
          break;
      }
    }
  }
}

/*********************************************************************
 * @fn      GenericApp_SendTheMessage
 *
 * @brief   Send "the" message.
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_SendTheMessage( void )
{
  //char theMessageData[] = "Hello World";

  if(IOT_Status.Recflag)
  {
      if ( AF_DataRequest( &GenericApp_DstAddr, &GenericApp_epDesc,
                           GENERICAPP_CLUSTERID,
                           (byte)2,
                           (byte *)&IOT_Status.Recbuffer,
                           &GenericApp_TransID,
                           AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
      {
        // Successfully requested to be sent.
        osal_memset(IOT_Status.Recbuffer,0,IOT_Status.Reclen);//将数组清空，进行下一次接收
        IOT_Status.Recflag=0;
 
      }
      
      else
      {
        // Error occurred in request to send.
      }
  }
}

#if defined( IAR_ARMCM3_LM )
/*********************************************************************
 * @fn      GenericApp_ProcessRtosMessage
 *
 * @brief   Receive message from RTOS queue, send response back.
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_ProcessRtosMessage( void )
{
  osalQueue_t inMsg;

  if ( osal_queue_receive( OsalQueue, &inMsg, 0 ) == pdPASS )
  {
    uint8 cmndId = inMsg.cmnd;
    uint32 counter = osal_build_uint32( inMsg.cbuf, 4 );

    switch ( cmndId )
    {
      case CMD_INCR:
        counter += 1;  /* Increment the incoming counter */
                       /* Intentionally fall through next case */

      case CMD_ECHO:
      {
        userQueue_t outMsg;

        outMsg.resp = RSP_CODE | cmndId;  /* Response ID */
        osal_buffer_uint32( outMsg.rbuf, counter );    /* Increment counter */
        osal_queue_send( UserQueue1, &outMsg, 0 );  /* Send back to UserTask */
        break;
      }
      
      default:
        break;  /* Ignore unknown command */    
    }
  }
}
#endif

/*********************************************************************
 */
