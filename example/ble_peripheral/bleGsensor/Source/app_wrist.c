/**************************************************************************************************

    Shanghai QST Corporation confidential and proprietary.
    All rights reserved.

    IMPORTANT: All rights of this software belong to Shanghai QST
    Corporation ("QST"). Your use of this Software is limited to those
    specific rights granted under  the terms of the business contract, the
    confidential agreement, the non-disclosure agreement and any other forms
    of agreements as a customer or a partner of QST. You may not use this
    Software unless you agree to abide by the terms of these agreements.
    You acknowledge that the Software may not be modified, copied,
    distributed or disclosed unless embedded on a QST Bluetooth Low Energy
    (BLE) integrated circuit, either as a product or is integrated into your
    products.  Other than for the aforementioned purposes, you may not use,
    reproduce, copy, prepare derivative works of, modify, distribute, perform,
    display or sell this Software and/or its documentation for any purposes.

    YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
    PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
    INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
    NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
    QST OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
    NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
    LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
    OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
    OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

**************************************************************************************************/

/**************************************************************************************************
    Filename:       wrist.c
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "ota_app_service.h"
#include "gatt_profile_uuid.h"
#include "wristservice.h"
#include "devinfoservice.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "app_wrist.h"
#include <string.h>
//#include "ui_display.h"
//#include "touch_key.h"
//#include "em70xx.h"
#include "qma6100.h"
//#include "battery.h"
#include "led_light.h"
#include "kscan.h"
#include "log.h"

/*********************************************************************
    MACROS
*/

#define DEF_BOARD_QST_EVK                     1

// Convert BPM to RR-Interval for data simulation purposes
#define HEARTRATE_BPM_TO_RR(bpm)              ((uint16) 60 * 1024 / (uint16) (bpm))

/*********************************************************************
    CONSTANTS
*/

// Fast advertising interval in 625us units
#define DEFAULT_FAST_ADV_INTERVAL             32

// Duration of fast advertising duration in ms
#define DEFAULT_FAST_ADV_DURATION             30000

// Slow advertising interval in 625us units
#define DEFAULT_SLOW_ADV_INTERVAL             1600

// Duration of slow advertising duration in ms (set to 0 for continuous advertising)
#define DEFAULT_SLOW_ADV_DURATION             0

// How often to perform heart rate periodic event
#define DEFAULT_HEARTRATE_PERIOD              2000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     200

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     1600

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         1

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

// Some values used to simulate measurements
#define BPM_DEFAULT                           73
#define BPM_MAX                               80
#define ENERGY_INCREMENT                      10
#define FLAGS_IDX_MAX                         7

/*********************************************************************
    TYPEDEFS
*/

/*********************************************************************
    GLOBAL VARIABLES
*/

/*********************************************************************
    EXTERNAL VARIABLES
*/

/*********************************************************************
    EXTERNAL FUNCTIONS
*/

/*********************************************************************
    LOCAL VARIABLES
*/
uint8 AppWrist_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

static uint8 scanData[] =
{
    0x0F,   // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'Q',
    'S',
    'T',
    's',
    'e',
    'n',
    's',
    'o',
    'r',
    '-',
    'X',
    'X',
    'X',
    'X',
};


static uint8 advertData[] =
{
    // flags
    0x02,
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
    // service UUIDs
    0x03,
    GAP_ADTYPE_16BIT_MORE,
    LO_UINT16(HEARTRATE_SERV_UUID),
    HI_UINT16(HEARTRATE_SERV_UUID),
};

// Device name attribute value
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "QSTsensor ";

// GAP connection handle
static uint16 gapConnHandle;

// Advertising user-cancelled state
static bool WristAdvCancelled = FALSE;
extern uint16_t crc16(uint16_t seed, const volatile void* p_data, uint32_t size);


/*********************************************************************
    LOCAL FUNCTIONS
*/
static void appWristProcOSALMsg( osal_event_hdr_t* pMsg );
static void WristGapStateCB( gaprole_States_t newState);
//static void HeartRateGapStateCB( gaprole_States_t newState );
//static void heartRatePeriodicTask( void );
//static void heartRateMeasNotify(void);
static void wristCB(uint8 event, uint8 param_size, uint8* param);
char* bdAddr2Str( uint8* pAddr );
/*********************************************************************
    PROFILE CALLBACKS
*/

// GAP Role Callbacks
static gapRolesCBs_t WristPeripheralCB =
{
    WristGapStateCB,  // Profile State Change Callbacks
    NULL                            // When a valid RSSI is read from controller
};

//// Bond Manager Callbacks
//static const gapBondCBs_t WristBondCB =
//{
//  NULL,                   // Passcode callback
//  NULL                    // Pairing state callback
//};

#ifdef DEF_BOARD_QST_EVK

  #define GPIO_GREEN    P0
  #define GPIO_BLUE     P1
  #define GPIO_RED      P27
  #define KEY1          P11
  #define KEY2          P14
  //char* ledStr[4] = {"led_none","led5","led3","led4"};
#else
  #define GPIO_GREEN    P31
  #define GPIO_BLUE     P33
  #define GPIO_RED      P32
  #define KEY1          P14
  #define KEY2          P15
  //char* ledStr[4] = {"led_none","red","green","blue"};
#endif
static gpio_pin_e led_pins[3] = {GPIO_GREEN,GPIO_BLUE,GPIO_RED};

static void light_timeout_handler(void)
{
#ifdef DEF_BOARD_QST_EVK
    static uint8_t led = 0;
    uint8_t i;
    for(i=0;i<sizeof(led_pins);i++)
    {
      hal_gpio_write(led_pins[i], (i==led?0:1));
    }
    if(++led == sizeof(led_pins)) {
      led = 0;
    }
#else
    static light_color_t color = LIGHT_COLOR_OFF;

    light_color_quickSet(color);
    if(++color == LIGHT_COLOR_NUM) {
      color = LIGHT_COLOR_OFF;
    }
#endif
}

void acc_event_handler(qma6100_ev_t *pev)
{
    if (pev->ev == rawdata_event) {
        int gx, gy, gz;
        int16_t *acc_data = (int16_t *)pev->data;
        gx = acc_data[0];
        gy = acc_data[1];
        gz = acc_data[2];
        //LOG("acc report: %d, %d, %d\n", gx, gy, gz);
        wristProfileResponseAccelerationData(gx,gy,gz);
    }
}

void appWristInit( uint8 task_id )
{
    AppWrist_TaskID = task_id;
    LOG("\n\n\nQST-Gsensor Demo\n\n\n");

    // Setup the GAP Peripheral Role Profile
    {
        uint8 initial_advertising_enable = FALSE;
        uint8 advChnMap = GAP_ADVCHAN_37 | GAP_ADVCHAN_38 | GAP_ADVCHAN_39;
        // By setting this to zero, the device will go into the waiting state after
        // being discoverable for 30.72 second, and will not being advertising again
        // until the enabler is set back to TRUE
        uint16 gapRole_AdvertOffTime = 0;
        uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
        uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
        uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
        uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
        uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;
        uint8 peerPublicAddr[] =
        {
            0x01,
            0x02,
            0x03,
            0x04,
            0x05,
            0x06
        };
        GAPRole_SetParameter(GAPROLE_ADV_DIRECT_ADDR, sizeof(peerPublicAddr), peerPublicAddr);
        // set adv channel map
        GAPRole_SetParameter(GAPROLE_ADV_CHANNEL_MAP, sizeof(uint8), &advChnMap);
        // Set the GAP Role Parameters
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
        GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
        GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanData ), scanData );
        GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
        GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
        GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
        GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
        GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
        GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
    }
    // Set the GAP Characteristics
    GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );
    // Set advertising interval
    {
        uint16 advInt = 400;   // actual time = advInt * 625us
        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
    }
    // Initialize GATT attributes
    GGS_AddService( GATT_ALL_SERVICES );         // GAP
    GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes
    DevInfo_AddService( );
    ota_app_AddService();
    wristProfile_AddService(wristCB);
    //app_datetime_init();

    // initial QST g-sensor
    //qma6100_demo();

    // Setup a delayed profile startup
    osal_set_event( AppWrist_TaskID, START_DEVICE_EVT );
    //light_init(led_pins,3);
    //osal_start_reload_timer(AppWrist_TaskID, TIMER_LIGHT_EVT, 1000);
    osal_start_timerEx(AppWrist_TaskID, ACC_INIT_EVT, 500);
}

/*********************************************************************
    @fn      HeartRate_ProcessEvent

    @brief   Heart Rate Application Task event processor.  This function
            is called to process all events for the task.  Events
            include timers, messages and any other user defined events.

    @param   task_id  - The OSAL assigned task ID.
    @param   events - events to process.  This is a bit map and can
                     contain more than one event.

    @return  events not processed
*/
uint16 appWristProcEvt( uint8 task_id, uint16 events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function

    if ( events & SYS_EVENT_MSG )
    {
        uint8* pMsg;

        if ( (pMsg = osal_msg_receive( AppWrist_TaskID )) != NULL )
        {
            appWristProcOSALMsg( (osal_event_hdr_t*)pMsg );
            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & START_DEVICE_EVT )
    {
        // Start the Device
        VOID GAPRole_StartDevice( &WristPeripheralCB );
        // Register with bond manager after starting device
        //GAPBondMgr_Register( (gapBondCBs_t *) &WristBondCB );
        return ( events ^ START_DEVICE_EVT );
    }

    if( events & TIMER_DT_EVT)
    {
        app_datetime_sync_handler();
        return ( events ^ TIMER_DT_EVT );
    }

    if( events & TIMER_LIGHT_EVT)
    {
        light_timeout_handler();
        return ( events ^ TIMER_LIGHT_EVT);
    }

    if( events & ACC_INIT_EVT)
    {
        static uint16_t initCnt = 0;
        if (qma6100_demo(acc_event_handler) != QMA_SUCCESS) {
          osal_start_timerEx(AppWrist_TaskID, ACC_INIT_EVT, 1000);
          LOG("ReInit Acc %d\n", ++initCnt);
        }
        else {
          osal_start_reload_timer(AppWrist_TaskID, ACC_DATA_EVT, 2000);
        } 
        return ( events ^ ACC_INIT_EVT);
    }

    if( events & ACC_DATA_EVT)
    {
        int16_t Acc[3] = {0x00,};
        if (qma6100_data_read(Acc, sizeof(Acc)) == QMA_SUCCESS) {
          LOG("X %d, Y %d, Z %d\n", Acc[0], Acc[1], Acc[2]);
        }
        return ( events ^ ACC_DATA_EVT);
    }

    if( events & ACC_INT1_EVT)
    {
        qma6100_int1_handler();
        return ( events ^ ACC_INT1_EVT);
    }

    if( events & ACC_INT2_EVT)
    {
        qma6100_int2_handler();
        return ( events ^ ACC_INT2_EVT);
    }

    if ( events & RESET_ADV_EVT )
    {
        uint8 initial_advertising_enable = TRUE;
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
        return ( events ^ RESET_ADV_EVT );
    }

    return 0;
}


/*********************************************************************
    @fn      appWristProcOSALMsg

    @brief   Process an incoming task message.

    @param   pMsg - message to process

    @return  none
*/
static void appWristProcOSALMsg( osal_event_hdr_t* pMsg )
{
}


/*********************************************************************
    @fn      WristGapStateCB

    @brief   Notification from the profile of a state change.

    @param   newState - new state

    @return  none
*/
static void WristGapStateCB( gaprole_States_t newState )
{
    LOG("WristGapStateCB: %d\n", newState);

    // if connected
    if (newState == GAPROLE_CONNECTED)
    {
        // get connection handle
        GAPRole_GetParameter(GAPROLE_CONNHANDLE, &gapConnHandle);
    }
    // if disconnected
    else if (gapProfileState == GAPROLE_CONNECTED &&
             newState != GAPROLE_CONNECTED)
    {
        uint8 advState = TRUE;
        // reset client characteristic configuration descriptors
        wristProfile_HandleConnStatusCB( gapConnHandle, LINKDB_STATUS_UPDATE_REMOVED );

        if ( newState == GAPROLE_WAITING_AFTER_TIMEOUT )
        {
            // link loss timeout-- use fast advertising
            GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_FAST_ADV_INTERVAL );
            GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_FAST_ADV_INTERVAL );
            GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_FAST_ADV_DURATION );
        }
        else
        {
            // Else use slow advertising
            GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_SLOW_ADV_INTERVAL );
            GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_SLOW_ADV_INTERVAL );
            GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_SLOW_ADV_DURATION );
        }

        // Enable advertising
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advState );
    }
    // if advertising stopped
    else if ( gapProfileState == GAPROLE_ADVERTISING &&
              newState == GAPROLE_WAITING )
    {
        // if advertising stopped by user
        if ( WristAdvCancelled )
        {
            WristAdvCancelled = FALSE;
        }
        // if fast advertising switch to slow
        else if ( GAP_GetParamValue( TGAP_GEN_DISC_ADV_INT_MIN ) == DEFAULT_FAST_ADV_INTERVAL )
        {
            uint8 advState = TRUE;
            GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_SLOW_ADV_INTERVAL );
            GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_SLOW_ADV_INTERVAL );
            GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_SLOW_ADV_DURATION );
            GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advState );
        }
    }
    // if started
    else if (newState == GAPROLE_STARTED)
    {
        // Set the system ID from the bd addr
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
        uint8 ownAddress[B_ADDR_LEN];
        uint8 str_addr[14]= {0};
        uint8 initial_advertising_enable = FALSE;//true
        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];
        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;
        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];
        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
        osal_memcpy(&str_addr[0],bdAddr2Str(ownAddress),14);
        osal_memcpy(&scanData[12],&str_addr[10],4);
        osal_memcpy(&attDeviceName[9],&str_addr[10],4);
        GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanData ), scanData );
        // Set the GAP Characteristics
        GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
        osal_set_event(AppWrist_TaskID, RESET_ADV_EVT);
    }

    gapProfileState = newState;
}

/*********************************************************************
    @fn      heartRateCB

    @brief   Callback function for Wrist service.

    @param   event - service event

    @return  none
*/
static void wristCB(uint8 event, uint8 param_size, uint8* param)
{
    switch(event)
    {
    case WRIST_NOTI_ENABLED:
    {
        // if connected start periodic measurement
        if (gapProfileState == GAPROLE_CONNECTED)
        {
            //osal_start_timerEx( AppWrist_TaskID, HEART_PERIODIC_EVT, DEFAULT_HEARTRATE_PERIOD );
        }

        break;
    }

    case WRIST_NOTI_DISABLED:
    {
        // stop periodic measurement
        //osal_stop_timerEx( AppWrist_TaskID, HEART_PERIODIC_EVT );
        break;
    }
    }
}


/*********************************************************************
    @fn      bdAddr2Str

    @brief   Convert Bluetooth address to string. Only needed when
           LCD display is used.

    @return  none
*/
char* bdAddr2Str( uint8* pAddr )
{
    uint8       i;
    char        hex[] = "0123456789ABCDEF";
    static char str[B_ADDR_STR_LEN];
    char*        pStr = str;
    *pStr++ = '0';
    *pStr++ = 'x';
    // Start from end of addr
    pAddr += B_ADDR_LEN;

    for ( i = B_ADDR_LEN; i > 0; i-- )
    {
        *pStr++ = hex[*--pAddr >> 4];
        *pStr++ = hex[*pAddr & 0x0F];
    }

    *pStr = 0;
    return str;
}

/*********************************************************************
*********************************************************************/
