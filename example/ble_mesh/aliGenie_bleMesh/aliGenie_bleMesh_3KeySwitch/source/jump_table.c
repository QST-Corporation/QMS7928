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


// this file define the call proxy of ROM function, not change by user
//

/*******************************************************************************
    INCLUDES
*/
#include "jump_function.h"
#include "global_config.h"
#include "aliGenie_bleMesh.h"
#include "OSAL_Tasks.h"
#include "rf_phy_driver.h"
#include "pwrmgr.h"
#include "gpio.h"
#include "timer.h"
#include "uart.h"
#include "log.h"

/*******************************************************************************
    MACROS
*/
void (*trap_c_callback)(void);

extern void log_printf(const char* format, ...);
void _hard_fault(uint32_t* arg)
{
    uint32_t* stk = (uint32_t*)((uint32_t)arg);
    log_printf("[Hard fault handler]\n");
    log_printf("R0   = 0x%08x\n", stk[9]);
    log_printf("R1   = 0x%08x\n", stk[10]);
    log_printf("R2   = 0x%08x\n", stk[11]);
    log_printf("R3   = 0x%08x\n", stk[12]);
    log_printf("R4   = 0x%08x\n", stk[1]);
    log_printf("R5   = 0x%08x\n", stk[2]);
    log_printf("R6   = 0x%08x\n", stk[3]);
    log_printf("R7   = 0x%08x\n", stk[4]);
    log_printf("R8   = 0x%08x\n", stk[5]);
    log_printf("R9   = 0x%08x\n", stk[6]);
    log_printf("R10  = 0x%08x\n", stk[7]);
    log_printf("R11  = 0x%08x\n", stk[8]);
    log_printf("R12  = 0x%08x\n", stk[13]);
    log_printf("SP   = 0x%08x\n", stk[0]);
    log_printf("LR   = 0x%08x\n", stk[14]);
    log_printf("PC   = 0x%08x\n", stk[15]);
    log_printf("PSR  = 0x%08x\n", stk[16]);
    log_printf("ICSR = 0x%08x\n", *(volatile uint32_t*)0xE000ED04);

    if (trap_c_callback)
    {
        trap_c_callback();
    }

    while (1);
}
// *INDENT-OFF*
__asm void hard_fault(void)
{
    PRESERVE8
    IMPORT  _hard_fault
    ldr     r0, = 0x1FFF0400 /*store in global config 0x1fff0000 0x1fff0400*/
    subs    r0, r0, #72
    mov     r1, sp
    str     r1, [r0]
    adds    r0, #4
    stmia   r0!, {r4 - r7}
    mov     r4, r8
    mov     r5, r9
    mov     r6, r10
    mov     r7, r11
    stmia   r0!, {r4 - r7}
    pop     {r4 - r5} /* pop rom Hardfault stack*/
    pop     {r4 - r7} /* pop exception entry R0-R1*/
    stmia   r0!, {r4 - r7}
    pop     {r4 - r7}/* pop exception entry R12 LR PC xPSR*/
    stmia   r0!, {r4 - r7}
    subs    r0, r0, #68
    ldr     r1, = _hard_fault
    bx      r1
    ALIGN   4
}
// *INDENT-ON*

/*******************************************************************************
    CONSTANTS
*/
const uint32_t* const jump_table_base[256] __attribute__((section("jump_table_mem_area"))) =
{
    (const uint32_t*)0,                         // 0. write Log
    (const uint32_t*)osalInitTasks,             // 1. init entry of app
    (const uint32_t*)tasksArr,                  // 2. task list
    (const uint32_t*)& tasksCnt,                // 3. task count
    (const uint32_t*)& tasksEvents,             // 4. task events
    0, 0, 0, 0, 0,                              // 5 - 9, reserved by QST
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,               // 10 - 19, reserved by QST
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,               // 20 - 29, reserved by QST
    0, 0, 0, 0, 0, 0, 0, 0,                     // <30 - - 37>
    0, 0,
    0, 0, 0, 0, 0, 0, //40 - 45
    0, 0, 0, 0,                                 //46 - 49
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,               // 50 - 59, reserved by QST
    0,   // < 60 -
    0,
    0,
    0,
    0, 0, 0, 0, 0, 0,                           //  -69>, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,               // 70 -79, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,               // 80 - 89, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,               // 90 - 99, reserved for rom patch
    (const uint32_t*)hal_pwrmgr_sleep_process,         // <100 -
    (const uint32_t*)hal_pwrmgr_wakeup_process,
    (const uint32_t*)rf_phy_ini,
    0,
    0,
    0,
    0, 0, 0, 0,                       // - 109, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 110 -119, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 120 -129, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 130 -139, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 140 -149, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 150 -159, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 160 -169, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 170 -179, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 180 -189, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 190 -199, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 200 - 209, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 210 - 219, reserved for rom patch
    (const uint32_t*)hard_fault, 0, 0, 0, 0, 0, 0, 0,           // 220 - 227
    0, 0,       // 228 - 229
    0, 0, 0, 0, 0,  // 230 - 234
    (const uint32_t*)hal_UART0_IRQHandler,      // 235 uart irq handler
    0, 0, 0, 0, 0,    // 236 - 240
    0, 0, 0, 0, 0, 0, 0, 0, 0,     // 241 - 249, for ISR entry
    0, 0, 0, 0, 0, 0                  // 250 - 255, for ISR entry
};



/*******************************************************************************
    Prototypes
*/


/*******************************************************************************
    LOCAL VARIABLES
*/


/*********************************************************************
    EXTERNAL VARIABLES
*/
uint32 global_config[SOFT_PARAMETER_NUM] __attribute__((section("global_config_area")));
