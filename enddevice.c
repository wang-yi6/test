/******************************************************************************
  Filename:       GenericApp.c
  Revised:        $Date: 2012-03-07 01:04:58 -0800 (Wed, 07 Mar 2012) $
  Revision:       $Revision: 29656 $

  Description:    Generic Application (no Profile).


  Copyright 2004-2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License"). You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
******************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends "Hello World" to another "Generic"
  application every 5 seconds.  The application will also
  receives "Hello World" packets.

  The "Hello World" messages are sent/received as MSG type message.

  This applications doesn't have a profile, so it handles everything
  directly - itself.

  Key control:
    SW1:
    SW2:  initiates end device binding
    SW3:
    SW4:  initiates a match description request
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "ZComDef.h"
#include "OSAL_Timers.h"
#include "delay.h"
#include "zcl.h"
#include "zcl_general.h"

#include "GenericApp.h"
#include "DebugTrace.h"
#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "MT_UART.h"
#include "MT_APP.h"
#include "MT.h"
#include "DHT11.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "IOT.h"
#include "hal_adc.h"
#include <stdlib.h>

extern void LCD_P8x16Str(unsigned char x, unsigned char y,unsigned char ch[]);
extern void HalLcdDisplayPercentBar( char *title, uint8 value );
extern void LCD_P16x16Ch(unsigned char x, unsigned char y, unsigned char N);

/* RTOS */
#if defined( IAR_ARMCM3_LM )
#include "RTOS_App.h"
#endif  

/*********************************************************************
 * MACROS
 */
static uint8 manualMode = 0; // 0为自动模式，1为手动模式
static uint8 ledState = HAL_LED_MODE_OFF ; // LED灯状态，0为关闭，1为打开

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */



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

// Pin assignments for sensors
#define PHOTO_RESISTOR_PIN   P0_3
#define PIR_SENSOR_PIN       P0_7
#define LED_OFF 0
#define LED_ON 1
#define CMD_TOGGLE_LED  0x03
static bool LEDState = false;
// 定义三个LED灯的开关状态数组
bool isLedOn[3] = {false, false, false}; // 默认都是关闭状态
char strTemp[32]; // Define strTemp as a character array of size 32



/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void GenericApp_HandleKeys( byte shift, byte keys );
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
static void GenericApp_SendTheMessage( void );
void UartInitPort0(void );
static void rxCB(uint8 port,uint8 event);
unsigned char GetPhotoResistorReading(void);
unsigned char GetPirSensorReading(void);


#if defined( IAR_ARMCM3_LM )
static void GenericApp_ProcessRtosMessage( void );
#endif

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */
extern uint8 autoMode;
extern uint8_t mode;
uint8_t uartState = AUTO_MODE;
/*********************************************************************
 * PUBLIC FUNCTIONS
 */


void UartInitPort0(void )
{

  halUARTCfg_t uartConfig; // 定义一个串口配置结构体
  uartConfig.configured = TRUE; // 串口配置为真
  uartConfig.baudRate = HAL_UART_BR_115200; // 波特率为115200
  uartConfig.flowControl = FALSE; // 流控制为假
  uartConfig.callBackFunc = rxCB; // 设置回调函数
  HalUARTOpen(HAL_UART_PORT_0, &uartConfig); // 打开串口0
  
  // 配置P0_3和P0_7引脚
  P0SEL &= ~(BV(3) | BV(7)); // 设置为GPIO模式
  P0DIR &= ~(BV(3) | BV(7)); // 设置为输入模式
  P0INP |= BV(3) | BV(7); // 设置为无上拉下拉
}

// 下面这个回调函数是我加的，从串口0读取n个字符，放进uartbuf
static void rxCB(uint8 port, uint8 event)
{
  // 定义一个数组，用于存储读取到的数据
  static uint8_t uartbuf[3];
  // 定义一个变量，用于记录当前读取到的数据个数
  static uint8_t count = 0;
  
  while (Hal_UART_RxBufLen(0)) //检测串口数据是否接收完成
  {
    // 读取一个字节
    HalUARTRead(0, &uartbuf[count], 1);
    count++;
    // 当读取到两个字节后，将其封装成一个消息并发送
    if (count == 2) {
      uint8_t data[3] = {0x00, 0x00, 0x00};
      data[0] = uartbuf[0];  // 第一个字节为传感器类型，0表示光敏电阻，1表示热释电红外
      data[1] = uartbuf[1];  // 第二个字节为传感器输出的电平信号
      
      // 封装成一个AF数据包并发送给协调器
      AF_DataRequest(&GenericApp_DstAddr, &GenericApp_epDesc,
                     GENERICAPP_CLUSTERID, 3, data,
                     &GenericApp_TransID, AF_DISCV_ROUTE,
                     AF_DEFAULT_RADIUS);
      
      // 重置计数器和缓存数组
      count = 0;
      memset(uartbuf, 0, sizeof(uartbuf));
    }
  }
}
  

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
 
    // Store the task ID
  GenericApp_TaskID = task_id;
  
  // Initialize LED pins
  P1SEL &= ~0x03; // Set P1.0 and P1.1 as GPIO
  P1DIR |= 0x03;  // Set P1.0 and P1.1 as outputs
  P1 &= ~0x03;    // Set P1.0 and P1.1 to low
  
  // Initialize UART for debugging
  UartInitPort0();
  

  
  // Set up endpoint address for communication with coordinator
  GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit; // Set to unicast
  GenericApp_DstAddr.endPoint = GENERICAPP_ENDPOINT;    // Set to endpoint of coordinator
  GenericApp_DstAddr.addr.shortAddr = 0x0000;           // Set coordinator short address
  
  // Fill out the endpoint description.
  GenericApp_epDesc.endPoint = GENERICAPP_ENDPOINT;
  GenericApp_epDesc.task_id = &GenericApp_TaskID;
  GenericApp_epDesc.simpleDesc = (SimpleDescriptionFormat_t *)&GenericApp_SimpleDesc;
  GenericApp_epDesc.latencyReq = noLatencyReqs;
  
  // Register the endpoint description with the AF
  afRegister( &GenericApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( GenericApp_TaskID );

  // Update the display
#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "GenericApp", HAL_LCD_LINE_1 );
#endif

  // Register for ZDO messages
  ZDO_RegisterForZDOMsg( GenericApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( GenericApp_TaskID, Match_Desc_rsp );

  // Set device state to initialized
  GenericApp_NwkState = DEV_INIT;
  
#if defined( IAR_ARMCM3_LM )
  // Register this task with RTOS task initiator
  RTOS_RegisterApp( task_id, GENERICAPP_RTOS_MSG_EVT );
#endif
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
uint16 GenericApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;

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
          GenericApp_MessageMSGCB( MSGpkt );
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

        case GENERICAPP_SEND_MSG_EVT: // 新增：处理GENERICAPP_SEND_MSG_EVT事件，调用数据采集发送函数
          GenericApp_SendTheMessage();
          osal_start_timerEx( GenericApp_TaskID,
                              GENERICAPP_SEND_MSG_EVT,
                              GENERICAPP_SEND_MSG_TIMEOUT + (osal_rand() & 0x00FF) * ENDNUM);
                              // 修改：将发送时间间隔增加至 5s
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
    case End_Device_Bind_rsp://请求是发给我们的 Coordinator
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )//請求绑定
      {
        // Light LED
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );//绑定成功
      }
#if defined( BLINK_LEDS )
      else
      {
        // Flash LED to show failure
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );//绑定失败
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

static void GenericApp_HandleKeys( uint8 shift, uint8 keys )
{
 zAddrType_t dstAddr;

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
        }
        else
        {
          ledState = LED_OFF;
          HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
          HalLedSet(HAL_LED_2, HAL_LED_MODE_OFF);
          HalLedSet(HAL_LED_3, HAL_LED_MODE_OFF);
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
        // 向其他设备发送自动模式状态信息
      }
      else // 处于自动模式
      {
        // 切换自动模式和手动模式
        manualMode = 1;
        osal_stop_timerEx(App_TaskID, GENERICAPP_AUTO_MODE_EVT);
        HalLedSet(HAL_LED_2, HAL_LED_MODE_OFF);
        // 向其他设备发送手动模式状态信息
      }
      break;
    default:
      break;
  }
 // 在手动模式下，当按键操作执行后，不执行其他操作
  if (manualMode == 1)
  {
    return;
  }
  // 自动模式下，当光照超过阈值时，关闭所有LED灯
  if (manualMode == 0)
  {
    if (P0_3 == TRUE) // 光敏电阻传感器输出高电平
    {
      HalLedSet(HAL_LED_ALL, HAL_LED_MODE_OFF);
      ledState = LED_OFF;
    }
  }

  // 自动模式下，当有人经过时，所有LED灯都会亮起并发送信息
  if (manualMode == 0 && P0_7 == TRUE) // 热释电红外传感器输出高电平
  {
    HalLedSet(HAL_LED_ALL, HAL_LED_MODE_ON); // 所有LED灯都会亮起

    // 发送信息给其他设备
    dstAddr.addrMode = AddrBroadcast;
    dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
    ZDP_MatchDescReq(&dstAddr, NWK_BROADCAST_SHORTADDR,
                     GENERICAPP_PROFID,
                     GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                     GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                     FALSE);
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
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{

 switch ( pkt->clusterId )
  {
    case GENERICAPP_CLUSTERID:
      HalUARTWrite(0, pkt->cmd.Data, pkt->cmd.DataLength);  //输出接收到的数据,通过串口发送
      if(pkt->cmd.Data[0]=='1')//1号灯
      {
         if(pkt->cmd.Data[1]=='1')//开灯
         { isLedOn[0] = true;
         HalLedSet(HAL_LED_1,HAL_LED_MODE_ON);}
         else
         { isLedOn[0] = false;
         HalLedSet(HAL_LED_1,HAL_LED_MODE_OFF);}
      }
         if(pkt->cmd.Data[0]=='2')//2号灯
      {
         if(pkt->cmd.Data[1]=='1')//开灯
         {isLedOn[1] = true;
         HalLedSet(HAL_LED_2,HAL_LED_MODE_ON);}
         else
         {isLedOn[1] = false;
         HalLedSet(HAL_LED_2,HAL_LED_MODE_OFF);}
      }
      if(pkt->cmd.Data[0]=='3')//1号灯
      {
         if(pkt->cmd.Data[1]=='1')//开灯
         {isLedOn[2] = true;
         HalLedSet(HAL_LED_3,HAL_LED_MODE_ON);}
         else
         {isLedOn[2] = false;
         HalLedSet(HAL_LED_3,HAL_LED_MODE_OFF);}
      }
     


      break;
  }


}

/*********************************************************************
 * @fn      GenericApp_Send_wenshidu_Message
 *
 * @brief   point to point.
 *
 * @param   none
 *
 * @return  none
 */



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
 unsigned char idstr[2], msg[30];

  memset(idstr, 0, 2);
  memset(msg, 0, 30);

  idstr[0] = ENDNUM + 0x30;
  idstr[1] = 0x00;

  // 根据LED灯状态设置消息内容
  int ledCount = 0;
  if (manualMode == 0) {
    // 如果是自动模式，发送LED灯状态和设备在线信息
    if (isLedOn[0]) {
      osal_memcpy(msg, "LED1:ON,", 8);
      ledCount++;
    } else {
      osal_memcpy(msg, "LED1:OFF,", 9);
    }
    if (isLedOn[1]) {
      osal_memcpy(&msg[9], "LED2:ON,", 8);
      ledCount++;
    } else {
      osal_memcpy(&msg[9], "LED2:OFF,", 9);
    }
    if (isLedOn[2]) {
      osal_memcpy(&msg[18], "LED3:ON,AUTO", 13);
      ledCount++;
    } else {
      osal_memcpy(&msg[18], "LED3:OFF,AUTO", 14);
    }
  } else {
    // 如果是手动模式，根据LED灯状态设置消息内容和亮起的LED数量
    if (isLedOn[0]) {
      osal_memcpy(msg, "LED1:ON,", 8);
      ledCount++;
    } else {
      osal_memcpy(msg, "LED1:OFF,", 9);
    }
    if (isLedOn[1]) {
      osal_memcpy(&msg[9], "LED2:ON,", 8);
      ledCount++;
    } else {
      osal_memcpy(&msg[9], "LED2:OFF,", 9);
    }
    osal_memcpy(&msg[18], "LED3:", 5);
    if (isLedOn[2]) {
      osal_memcpy(&msg[23], "ON,MANUAL", 9);
      ledCount++;
    } else {
      osal_memcpy(&msg[23], "OFF,MANUAL", 10);
    }
    // 根据LED状态设置消息内容
    if (manualMode == 0) {
      // 如果是自动模式，发送LED灯状态和设备在线信息
      if (LEDState) {
        osal_memcpy(msg, "LED:ON,AUTO", 11);
      } else {
        osal_memcpy(msg, "LED:OFF,AUTO", 12);
      }
    } else {
      // 如果是手动模式，根据LED状态设置消息内容
      if (LEDState) {
        osal_memcpy(msg, "LED:ON,MANUAL", 13);
      } else {
        osal_memcpy(msg, "LED:OFF,MANUAL", 14);
      }
}

  }

  // 将消息内容和设备ID组合成一个完整的消息
   osal_memcpy(strTemp, "id:", 3);
  osal_memcpy(&strTemp[3], idstr, 1);
  osal_memcpy(&strTemp[4], "msg:", 5);
  osal_memcpy(&strTemp[9], msg, osal_strlen((char *)msg));
  osal_memcpy(&strTemp[9 + osal_strlen((char *)msg)], ",LEDCount:", 10);
  osal_memcpy(&strTemp[19 + osal_strlen((char *)msg)], &ledCount, 1);
  osal_memcpy(&strTemp[20 + osal_strlen((char *)msg)], "\n", 1);

  // 将消息通过Zigbee协议发送出去
  afAddrType_t dstAddr;
memset(&dstAddr, 0, sizeof(afAddrType_t));
dstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;

uint16_t len = osal_strlen((char *)strTemp);
uint8_t *buf = strTemp;
uint8_t handle = 0; // 根据实际情况填写正确的handle
AF_DataRequest(&dstAddr, &GenericApp_epDesc, GENERICAPP_CLUSTERID, len, buf, &GenericApp_TransID,
               AF_DISCV_ROUTE, AF_DEFAULT_RADIUS);

osal_mem_free(buf);
 
}

/**********************************************************************/

//static void GenericApp_ProcessRtosMessage( void )
//{
//  osalQueue_t inMsg;
//
//  if ( osal_queue_receive( OsalQueue, &inMsg, 0 ) == pdPASS )
//  {
//    uint8 cmndId = inMsg.cmnd;
//    uint32 counter = osal_build_uint32( inMsg.cbuf, 4 );
//
//    switch ( cmndId )
//    {
//      case CMD_INCR:
//        counter += 1;  /* Increment the incoming counter */
//                       /* Intentionally fall through next case */
//
//      case CMD_ECHO:
//      {
//        userQueue_t outMsg;
//
//        outMsg.resp = RSP_CODE | cmndId;  /* Response ID */
//        osal_buffer_uint32( outMsg.rbuf, counter );    /* Increment counter */
//        osal_queue_send( UserQueue1, &outMsg, 0 );  /* Send back to UserTask */
//        break;
//      }
//      
//      case CMD_AUTO_MODE:
//      {
//        userQueue_t outMsg;
//        uint8_t status = inMsg.cbuf[0];
//
//        // 设置自动模式标志位
//        if (status == 1) {
//          isAutoMode = true;
//        } else {
//          isAutoMode = false;
//        }
//
//        // 向协调器发送设备状态信息
//        outMsg.resp = RSP_CODE | CMD_DEVICE_STATUS;
//        osal_buffer_uint32(outMsg.rbuf, status);
//        osal_queue_send(CoordQueue, &outMsg, 0);
//        break;
//      }
//      
//    case CMD_TOGGLE_LED:
//      {
//        // 切换LED状态
//        LEDState = !LEDState;
//        
//        // 向协调器发送设备状态信息
//        userQueue_t outMsg;
//        outMsg.resp = RSP_CODE | CMD_DEVICE_STATUS;
//        osal_buffer_uint32(outMsg.rbuf, LEDState);
//        osal_queue_send(CoordQueue, &outMsg, 0);
//        
//        // 发送更新后的LED状态到其他设备
//        GenericApp_SendTheMessage();
//        
//        break;
//      }
//      
//
//      default:
//        break;  /* Ignore unknown command */    
//    }
//  }
//}

/*********************************************************************
 */
