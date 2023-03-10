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
    Filename:       qst_evk_demo.c
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/

#include "OSAL.h"
#include "qst_evk_demo.h"
#include "log.h"

#include "gpio.h"
#include "clock.h"

#include "pwrmgr.h"
#include "error.h"
#include "key.h"
#include "led_light.h"
#include "qma6100.h"


#ifdef DEF_BOARD_QST_EVK

  #define GPIO_GREEN    P0
  #define GPIO_BLUE     P1
  #define GPIO_RED      P27
  #define KEY1          P11
  #define KEY2          P14
  char* ledStr[4] = {"led_none","led5","led3","led4"};
#else
  #define GPIO_GREEN    P31
  #define GPIO_BLUE     P33
  #define GPIO_RED      P32
  #define KEY1          P14
  #define KEY2          P15
  char* ledStr[4] = {"led_none","red","green","blue"};
#endif
static gpio_pin_e led_pins[3] = {GPIO_GREEN,GPIO_BLUE,GPIO_RED};

/*********************************************************************
    pulseMeasure_Task
    Task pulseMeasure sample code,we can use p04~p07 and p11~p15 easily.
*/
static uint8 pulseMeasure_TaskID;

typedef struct
{
    bool          enable;
    bool          pinstate;
    uint32_t      edge_tick;
} gpioin_Trig_t;

typedef struct
{
    GPIO_Pin_e    pin;
    bool          type;
    uint32_t      ticks;
} gpioin_pulse_Width_measure_t;

gpioin_pulse_Width_measure_t measureResult =
{
    .pin = GPIO_P14,
};

static gpioin_Trig_t gpioTrig =
{
    .enable = FALSE,
    .edge_tick = 0,
};

void plus_edge_callback(void)
{
    LOG("pulse:%d %d\n",measureResult.type,measureResult.ticks);
}

void pulse_measure_callback(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
{
    if(gpioTrig.enable == FALSE)
    {
        gpioTrig.enable = TRUE;
        gpioTrig.edge_tick = hal_systick();
        return;
    }

    measureResult.type = type;
    measureResult.ticks = hal_ms_intv(gpioTrig.edge_tick);
    plus_edge_callback();
    gpioTrig.edge_tick = hal_systick();
}


void Pulse_Measure_Init( uint8 task_id )
{
    pulseMeasure_TaskID = task_id;
    hal_gpio_init();
    hal_gpioin_register(measureResult.pin,pulse_measure_callback,pulse_measure_callback);
    gpioTrig.pinstate = hal_gpio_read(measureResult.pin);
}

uint16 Pulse_Measure_ProcessEvent( uint8 task_id, uint16 events )
{
    if(task_id != pulseMeasure_TaskID)
    {
        return 0;
    }

    // Discard unknown events
    return 0;
}

/*********************************************************************
    gpio_wakeup_Task
    Task gpio wakeup sample code
    The followinng code shows P14 wakeup the system when there is a posedge or negedge.
*/
static uint8 gpio_wakeup_TaskID;
void posedge_callback_wakeup(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
{
    if(type == POSEDGE)
    {
        LOG("wakeup(pos):gpio:%d type:%d\n",pin,type);
    }
    else
    {
        LOG("error\n");
    }
}

void negedge_callback_wakeup(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
{
    if(type == NEGEDGE)
    {
        LOG("wakeup(neg):gpio:%d type:%d\n",pin,type);
    }
    else
    {
        LOG("wakeup(pos):gpio:%d type:%d\n",pin,type);
    }
}

/*
     P00~P03:default jtag,we can use it as wakeup pin when no debug.
     P04~P07,P11~P15,P18~P30:default gpio,use it easily.
     P08:mode select pin,cannot used as other usage.
     P09~P10,it is uart in burn mode which cannot config.it is configable when in debug mode.
     P16~P17:xtal pin,when use this pins,please use rc as system frequency.config hal_rtc_clock_config(CLK_32K_RCOSC) in hal_init first.
     P31~P34:default spif,we can use it as wakeup pin directly,we driver have completed its multiplex config.
*/
typedef struct gpioin_wakeup_t
{
    GPIO_Pin_e pin;
    gpioin_Hdl_t posedgeHdl;
    gpioin_Hdl_t negedgeHdl;
} gpioin_wakeup;

gpioin_wakeup gpiodemo[GPIO_WAKEUP_PIN_NUM] =
{
    GPIO_P11,posedge_callback_wakeup,negedge_callback_wakeup,
    GPIO_P14,posedge_callback_wakeup,negedge_callback_wakeup,
    GPIO_P23,posedge_callback_wakeup,negedge_callback_wakeup,
};

void GPIO_Wakeup_Init(uint8 task_id )
{
    uint8_t i = 0;
    static bool gpioin_state[GPIO_WAKEUP_PIN_NUM];
    hal_gpio_init();
    gpio_wakeup_TaskID = task_id;
    LOG("gpio wakeup demo start...\n");

    //hal_gpio_pull_set(P14,WEAK_PULL_UP);

    for(i = 0; i<GPIO_WAKEUP_PIN_NUM; i++)
    {
        hal_gpioin_register(gpiodemo[i].pin,gpiodemo[i].posedgeHdl,gpiodemo[i].negedgeHdl);
        gpioin_state[i] = hal_gpio_read(gpiodemo[i].pin);
        LOG("gpioin_state:%d %d\n",i,gpioin_state[i]);
    }
}

uint16 GPIO_Wakeup_ProcessEvent( uint8 task_id, uint16 events )
{
    if(task_id != gpio_wakeup_TaskID)
    {
        return 0;
    }

    return 0;
}

/*********************************************************************
    key_Task:gpio config as key

*/
static uint8 EvkDemo_TaskID;

#define KEY_DEMO_ONCE_TIMER      0x0001
#define KEY_DEMO_CYCLE_TIMER     0x0002
#define ACC_INIT_EVT             0x0004
//#define HAL_KEY_EVENT            0x0100//assign short key event in your app event process

#ifdef HAL_KEY_SUPPORT_LONG_PRESS
    //    #define KEY_DEMO_LONG_PRESS_EVT   0x0200 //if use long key,assign long key event in your app process
#endif

static void led_toggle(light_color_t color)
{
#ifdef DEF_BOARD_QST_EVK
    static bool redIsOn = TRUE;
    static bool greenIsOn = TRUE;
    static bool blueIsOn = TRUE;
#else
    static bool redIsOn = FALSE;
    static bool greenIsOn = FALSE;
    static bool blueIsOn = FALSE;
#endif
    uint8_t led = LIGHT_RED;
    uint16_t led_value = LIGHT_TURN_OFF;

    if (color == LIGHT_COLOR_RED) {
      led = LIGHT_RED;
      led_value = redIsOn?LIGHT_TURN_OFF:LIGHT_TURN_ON;
      redIsOn = !redIsOn;
    } else if (color == LIGHT_COLOR_GREEN) {
      led = LIGHT_GREEN;
      led_value = greenIsOn?LIGHT_TURN_OFF:LIGHT_TURN_ON;
      greenIsOn = !greenIsOn;
    } else if (color == LIGHT_COLOR_BLUE) {
      led = LIGHT_BLUE;
      led_value = blueIsOn?LIGHT_TURN_OFF:LIGHT_TURN_ON;
      blueIsOn = !blueIsOn;
    }

#ifdef DEF_BOARD_QST_EVK
    hal_gpio_write(led_pins[led], led_value);
#else
    light_config(led, led_value);
    light_reflash();
#endif
}

static void key_ctl_led(gpio_pin_e key_pin)
{
    if(key_pin == KEY1) {
        led_toggle(LIGHT_COLOR_RED);
        LOG("%s\n", ledStr[LIGHT_COLOR_RED]);
    } else if(key_pin == KEY2) {
        led_toggle(LIGHT_COLOR_GREEN);
        LOG("%s\n", ledStr[LIGHT_COLOR_GREEN]);
    } else if(key_pin == GPIO_DUMMY) {
        led_toggle(LIGHT_COLOR_BLUE);
        LOG("%s\n", ledStr[LIGHT_COLOR_BLUE]);
    }
}

static void key_press_evt(uint8_t i,key_evt_t key_evt)
{
    //LOG("\nkey index:%d gpio:%d ",i,key_state.key[i].pin);

    switch(key_evt)
    {
    case HAL_KEY_EVT_PRESS:
        LOG("key:%d(press down) ", key_state.key[i].pin);
        key_ctl_led(key_state.key[i].pin);
        break;

    case HAL_KEY_EVT_RELEASE:
        //LOG("key(press release)\n");
        break;
        #ifdef HAL_KEY_SUPPORT_LONG_PRESS

    case HAL_KEY_EVT_LONG_RELEASE:
        hal_pwrmgr_unlock(MOD_USR1);
        LOG("key(long press release)\n");
        break;
        #endif

    default:
        LOG("unexpect\n");
        break;
    }
}


void EVK_Demo_Init(uint8 task_id)
{
    uint8_t i = 0;
    EvkDemo_TaskID = task_id;
    LOG("EVK demo start...\n");
    hal_gpio_init();
    //hal_gpioretention_register(P20);
    //hal_gpio_write(P20,1);
//    hal_gpio_pin2pin3_control(P2,1);
    /*
        when use key,please set the following parameters:
        1.key number,config KEY_NUM in key.h
        2.gpio used,config key_state.pin
             P00~P03:default jtag,we can use it as key when no debug.
             P04~P07,P11~P15:default gpio,use it easily.
             P08:mode select pin,cannot used as other usage.
             P09~P10,it is uart in burn mode which cannot config.it is configable when in debug mode.
             P16~P17:xtal pin,when use this pins,please use rc as system frequency.config hal_rtc_clock_config(CLK_32K_RCOSC) in hal_init first.
             P18~P34:wakeup is supported,but interrupt is not supported,so config it as key is not suggested.
        3.idle level,config key_state.idle_level
        4.key type,if only use press and release,ignore the long press and release code
        5.taskID and callback function
    */
    key_state.key[0].pin = KEY1;//default gpio
    key_state.key[1].pin = KEY2;
//  key_state.key[2].pin = GPIO_P00;//default jtag
//  key_state.key[3].pin = GPIO_P01;
//  key_state.key[4].pin = GPIO_P02;
//  key_state.key[5].pin = GPIO_P03;


    for(i = 0; i < HAL_KEY_NUM; ++i)
    {
        key_state.key[i].state = HAL_STATE_KEY_IDLE;
        key_state.key[i].idle_level = HAL_HIGH_IDLE;//HAL_LOW_IDLE;
    }

//  key_state.key[0].idle_level = HAL_LOW_IDLE;
//  key_state.key[1].idle_level = HAL_HIGH_IDLE;
    key_state.task_id = EvkDemo_TaskID;
    key_state.key_callbank = key_press_evt;
    key_init();
    light_init(led_pins,3);
    osal_start_timerEx(EvkDemo_TaskID, KEY_DEMO_ONCE_TIMER, 1000);
    osal_start_reload_timer(EvkDemo_TaskID, KEY_DEMO_CYCLE_TIMER, 1000);
#ifdef DEF_BOARD_QST_EVK
    osal_start_timerEx(EvkDemo_TaskID, ACC_INIT_EVT, 500);
#endif
}

uint16 EVK_ProcessEvent( uint8 task_id, uint16 events )
{
    if(task_id != EvkDemo_TaskID)
    {
        return 0;
    }

    if( events & KEY_DEMO_ONCE_TIMER)
    {
        //LOG("once timer\n");
        osal_start_timerEx( EvkDemo_TaskID, KEY_DEMO_ONCE_TIMER, 1000);
        return (events ^ KEY_DEMO_ONCE_TIMER);
    }

    if( events & KEY_DEMO_CYCLE_TIMER)
    {
        //LOG("recycle timer\n");
        return (events ^ KEY_DEMO_CYCLE_TIMER);
    }

    if( events & ACC_INIT_EVT)
    {
        static uint16_t initCnt = 0;
        if (qma6100_demo() != QMA_SUCCESS) {
          osal_start_timerEx(EvkDemo_TaskID, ACC_INIT_EVT, 1000);
          LOG("ReInit Acc %d\n", ++initCnt);
        }
        return ( events ^ ACC_INIT_EVT);
    }

    if( events & HAL_KEY_EVENT)                                                     //do not modify,key will use it
    {
        for (uint8 i = 0; i < HAL_KEY_NUM; ++i)
        {
            if ((key_state.temp[i].in_enable == TRUE)||
                    (key_state.key[i].state == HAL_STATE_KEY_RELEASE_DEBOUNCE))
            {
                gpio_key_timer_handler(i);
            }
        }

        return (events ^ HAL_KEY_EVENT);
    }

    #ifdef HAL_KEY_SUPPORT_LONG_PRESS

    if( events & KEY_DEMO_LONG_PRESS_EVT)
    {
        for (int i = 0; i < HAL_KEY_NUM; ++i)
        {
            if(key_state.key[i].state == HAL_KEY_EVT_PRESS)
            {
                LOG("key:%d gpio:%d	",i,key_state.key[i].pin);
                LOG("key(long press down)");
                key_ctl_led(GPIO_DUMMY);
                //user app code long press down process
            }
        }

        return (events ^ KEY_DEMO_LONG_PRESS_EVT);
    }

    #endif
    return 0;
}

/*********************************************************************
*********************************************************************/
