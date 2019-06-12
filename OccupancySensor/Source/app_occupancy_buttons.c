/*****************************************************************************
 *
 * MODULE:             JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:          app_occupancy_buttons.c
 *
 * DESCRIPTION:        DK4 (DR1175/DR1199) Button Press detection (Implementation)
 *
 ****************************************************************************
 *
 * This software is owned by NXP B.V. and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on NXP products [NXP Microcontrollers such as JN5168, JN5179].
 * You, and any third parties must reproduce the copyright and warranty notice
 * and any other legend of ownership on each copy or partial copy of the
 * software.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Copyright NXP B.V. 2017. All rights reserved
 *
 ***************************************************************************/


#include <jendefs.h>
#include "ZTimer.h"
#include "ZQueue.h"
#include "app_main.h"
#include "dbg.h"
#include "AppHardwareApi.h"
#include "app_events.h"
#include "app_main.h"

#include "app_reporting.h"
#include "App_OccupancySensor.h"

#include "pwrm.h"
#include "app_occupancy_buttons.h"
#include "app_occupancy_sensor_state_machine.h"
/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifndef DEBUG_APP_BUTTON
    #define TRACE_APP_BUTTON               FALSE
#else
    #define TRACE_APP_BUTTON               TRUE
#endif

#define WAKE_FROM_DEEP_SLEEP     (1<<11)
#define DIO_STATE_NVM_LOCATION     0


PUBLIC void APP_bButtonInitialise(void) {
    // Set DIO lines to inputs with buttons connected
    vAHI_DioSetDirection(APP_BUTTONS_DIO_MASK, 0);

    // Turn off pull-ups for DIO lines with buttons connected
    vAHI_DioSetPullup(0, APP_BUTTONS_DIO_MASK);

    // Wake on both rising and falling edges
    vAHI_DioWakeEdge(APP_BUTTONS_DIO_MASK_RISING, APP_BUTTONS_DIO_MASK_FALLING);

    /*
    if (FALSE == (u16AHI_PowerStatus() & WAKE_FROM_DEEP_SLEEP))
    {
        // Set the edge detection for falling edges
        vAHI_DioWakeEdge(0, APP_BUTTONS_DIO_MASK);
    }
    else
    {
        u32PreviousDioState = u32AHI_ReadNVData(DIO_STATE_NVM_LOCATION);
        DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Woke from deep sleep, previous Dio State = %08x", u32PreviousDioState);
    }
    */

    // Enable wakeups
    vAHI_DioWakeEnable(APP_BUTTONS_DIO_MASK, 0);

    // Enable interrupts
    vAHI_DioInterruptEnable(APP_BUTTONS_DIO_MASK, 0);
}





PUBLIC void APP_cbTimerLeftDebounce( void * pvParam) {
	bool pressed = ((u32AHI_DioReadInput() & MASK_LEFT_BUTTON) != 0);
	DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: LEFT BUTTON DEBOUNCED: state is 0x%x", pressed);

	sSensor.sOnOffServerCluster.bOnOff = pressed;
	vSendImmediateReport();
}


PUBLIC void vISR_Timer0(uint32 u32Device, uint32 u32ItemBitmap) {
	// This does nothing. Remove later.
}




/*
 * ISR called on system controller interrupt.
 */
PUBLIC void vISR_SystemController(void) {
    // clear pending DIO changed bits by reading register
    uint8 u8WakeInt = u8AHI_WakeTimerFiredStatus();
    uint32_t dio_interrupts = u32AHI_DioInterruptStatus();

    DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: In vISR_SystemController: 0x%x", dio_interrupts);

    if (u8WakeInt & E_AHI_WAKE_TIMER_MASK_0) {
        APP_tsEvent sButtonEvent;

        // wake timer interrupt got us here
        DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Wake Timer 0 Interrupt");
        vAHI_WakeTimerStop(E_AHI_WAKE_TIMER_0);

        // Post a message to the stack so we aren't handling events in interrupt context
        sButtonEvent.eType = APP_E_EVENT_WAKE_TIMER;
        ZQ_bQueueSend(&APP_msgAppEvents, &sButtonEvent);
    }

    if (dio_interrupts & ((1 << DIO_LEFT_BUTTON_FALLING) | (1 << DIO_LEFT_BUTTON_RISING))) {
    	DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Debouncing left button");

        if (ZTIMER_eGetState(u8TimerLeftDebounce) == E_ZTIMER_STATE_RUNNING) {
        	DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Cancelling existing debounce event!");
        	ZTIMER_eStop(u8TimerLeftDebounce);
        }

        ZTIMER_eStart(u8TimerLeftDebounce, ZTIMER_TIME_MSEC(100));

    }

    if (u8WakeInt & E_AHI_WAKE_TIMER_MASK_1) {
        APP_tsEvent sButtonEvent;

        // wake timer interrupt got us here
        DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Wake Timer 1 Interrupt");

        PWRM_vWakeInterruptCallback();

        // Post a message to the stack so we aren't handling events in interrupt context
        sButtonEvent.eType = APP_E_EVENT_PERIODIC_REPORT;
        ZQ_bQueueSend(&APP_msgAppEvents, &sButtonEvent);
    }

}

/*
 * When we wake up, we have restarted so we need to manually check to see
 * what Dio woke us. Start the ButtonScanTask and disable wake interrupts
 */
PUBLIC void vActionOnButtonActivationAfterDeepSleep(void)
{
    DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Interrupt Status = %08x", u32AHI_DioInterruptStatus());
    //u32DioInterrupts |= APP_BUTTONS_DIO_MASK;
    vAHI_DioWakeEnable(APP_BUTTONS_DIO_MASK, 0);

    /* Begin debouncing the buttons */
    //bDebouncing = TRUE;
    //vAHI_TimerEnable(E_AHI_TIMER_0, 0, FALSE, TRUE, FALSE);
    //vAHI_Timer0RegisterCallback(vISR_Timer0);
    //vAHI_TimerStartSingleShot(E_AHI_TIMER_0, 0, 1);
}

/****************************************************************************
 *
 * NAME: vSaveDioStateBeforeDeepSleep
 *
 * DESCRIPTION:
 * Due to us going to sleep on a falling edge as well as a rising edge, we need
 * to save the Dio state into NVM so when we wake back up we know what edge we
 * had configured to wake us up.
 *
 ****************************************************************************/
PUBLIC void vSaveDioStateBeforeDeepSleep(void)
{
	/*
    DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Writing %08x to NVM", u32PreviousDioState);
    vAHI_WriteNVData(DIO_STATE_NVM_LOCATION, u32PreviousDioState);
    DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Written %08x to NVM", u32AHI_ReadNVData(DIO_STATE_NVM_LOCATION));
    */
}

/****************************************************************************
 *
 * NAME: bGetPreSleepOccupancyState
 *
 * DESCRIPTION:
 * This function returns the last Dio edge of Switch 1 which determines what the last
 * Occupancy state was.
 *
 ****************************************************************************/
PUBLIC bool_t bGetPreSleepOccupancyState(void) {
	/*
    DBG_vPrintf(TRACE_APP_BUTTON, "\nAPP Button: Occupancy = %08x: ", u32PreviousDioState & APP_BUTTONS_BUTTON_SW1);

    // If previously the occupied DIO pin was low, return occupied, else unoccupied
    if((u32PreviousDioState & APP_BUTTONS_BUTTON_SW1) == 0)
    {
        DBG_vPrintf(TRACE_APP_BUTTON, "Occupied");
        return TRUE;
    }
    else
    {
        DBG_vPrintf(TRACE_APP_BUTTON, "Unoccupied");
        return FALSE;
    }
    */
	return FALSE;

}
