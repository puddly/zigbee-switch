/****************************************************************************
 *
 * MODULE:             JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:          app_start_sensot.c
 *
 * DESCRIPTION:        ZLO Sensor Application Initialisation and Startup
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

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include "pwrm.h"
#include "pdum_nwk.h"
#include "pdum_apl.h"
#include "PDM.h"
#include "dbg.h"
#include "dbg_uart.h"
#include "pdum_gen.h"
#include "zps_gen.h"
#include "zps_apl_af.h"
#include "AppApi.h"
#include "app_zlo_sensor_node.h"
#include "app_main.h"
#include "app_blink_led.h"
#include "app_occupancy_buttons.h"
#include <string.h>
#include "bdb_api.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifndef DEBUG_START_UP
    #define TRACE_START FALSE
#else
    #define TRACE_START TRUE
#endif

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

PRIVATE void vInitialiseApp(void);
#ifdef SLEEP_ENABLE
    PRIVATE void vSetUpWakeUpConditions(void);
#endif

PRIVATE void vfExtendedStatusCallBack (ZPS_teExtendedStatus eExtendedStatus);
/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
extern void *_stack_low_water_mark;

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
/**
 * Power manager Callback.
 * Called just before the device is put to sleep
 */

static PWRM_DECLARE_CALLBACK_DESCRIPTOR(PreSleep);
/**
 * Power manager Callback.
 * Called just after the device wakes up from sleep
 */
static PWRM_DECLARE_CALLBACK_DESCRIPTOR(Wakeup);
/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/


/****************************************************************************
 *
 * NAME: vAppMain
 *
 * DESCRIPTION:
 * Entry point for application from a cold start.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vAppMain(void) {
	// Wait until FALSE i.e. on XTAL  - otherwise uart data will be at wrong speed
	while (bAHI_GetClkSource() == TRUE);
	// Now we are running on the XTAL, optimise the flash memory wait states
	vAHI_OptimiseWaitStates();

    // Don't use RTS/CTS pins on UART0 as they are used for buttons
    vAHI_UartSetRTSCTS(E_AHI_UART_0, FALSE);

    /*
     * Initialise the debug diagnostics module to use UART0 at 115K Baud;
     * Do not use UART 1 if LEDs are used, as it shares DIO with the LEDS
     */
    DBG_vUartInit(DBG_E_UART_0, DBG_E_UART_BAUD_RATE_115200);
    DBG_vPrintf(TRACE_START, "\nAPP Start: Switch Power Up");

    /*
     * Initialise the stack overflow exception to trigger if the end of the
     * stack is reached. See the linker command file to adjust the allocated
     * stack size.
     */
    vAHI_SetStackOverflow(TRUE, (uint32)&_stack_low_water_mark);

    /*
     * Catch resets due to watchdog timer expiry. Comment out to harden code.
     */
    if (bAHI_WatchdogResetEvent()) {
        DBG_vPrintf(TRACE_START, "\nAPP Start: Watchdog timer has reset device!");
        DBG_vDumpStack();
        #if HALT_ON_EXCEPTION
            vAHI_WatchdogStop();
            while (1);
        #endif
    }

    // initialise ROM based software modules
    #ifndef JENNIC_MAC_MiniMacShim
    u32AppApiInit(NULL, NULL, NULL, NULL, NULL, NULL);
    #endif

    // Define HIGH_POWER_ENABLE to enable high power module
    #ifdef HIGH_POWER_ENABLE
        vAHI_HighPowerModuleEnable(TRUE, TRUE);
    #endif


    DBG_vPrintf(TRACE_START, "\nAPP Entering APP_vSetUpHardware()");
    APP_vSetUpHardware();

    DBG_vPrintf(TRACE_START, "\nAPP Entering APP_vInitResources()");
    APP_vInitResources();

    DBG_vPrintf(TRACE_START, "\nAPP Entering APP_vInitialise()");
    vInitialiseApp();

	if (!APP_bNodeIsInRunningState()) {
		BDB_eNsStartNwkSteering();
	}

	DBG_vPrintf(TRACE_START, "APP: Entering BDB_vStart()\n");
	BDB_vStart();

    DBG_vPrintf(TRACE_START, "\nAPP Entering APP_vMainLoop()");

    /* no return from this */
    APP_vMainLoop();
}

/****************************************************************************
 *
 * NAME: vAppRegisterPWRMCallbacks
 *
 * DESCRIPTION:
 * Power manager callback.
 * Called to allow the application to register  sleep and wake callbacks.
 *
 * PARAMETERS:      Name            RW  Usage
 *
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
void vAppRegisterPWRMCallbacks(void) {
    PWRM_vRegisterPreSleepCallback(PreSleep);
    PWRM_vRegisterWakeupCallback(Wakeup);
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: vInitialiseApp
 *
 * DESCRIPTION:
 * Initialises Zigbee stack, hardware and application.
 *
 *
 * RETURNS:
 * void
 ****************************************************************************/
PRIVATE void vInitialiseApp(void) {
    /*
     * Initialise JenOS modules. Initialise Power Manager even on non-sleeping nodes
     * as it allows the device to doze when in the idle task.
     * Parameter options: E_AHI_SLEEP_OSCON_RAMON or E_AHI_SLEEP_DEEP or ...
     */
    PWRM_vInit(E_AHI_SLEEP_OSCON_RAMON);

    PDM_eInitialise(63);

    // Initialise Protocol Data Unit Manager
    PDUM_vInit();

    ZPS_vExtendedStatusSetCallback(vfExtendedStatusCallBack);

    // Initialise application
    APP_vInitialiseNode();

    DBG_vPrintf(TRACE_START, "\nAPP Start: Tick Timer = %d", u32AHI_TickTimerRead());
    DBG_vPrintf(TRACE_START,"\nAPP Start: Initialised");

}

/****************************************************************************
 *
 * NAME: vfExtendedStatusCallBack
 *
 * DESCRIPTION:
 *
 * ZPS extended error callback .
 *
 * PARAMETERS:      Name            RW  Usage
 *
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vfExtendedStatusCallBack (ZPS_teExtendedStatus eExtendedStatus) {
    DBG_vPrintf(TRACE_START,"ERROR: Extended status %x\n", eExtendedStatus);
}

/****************************************************************************
 *
 * NAME: vSetUpWakeUpConditions
 *
 * DESCRIPTION:
 *
 * Set up the wake up inputs while going to sleep.
 *
 * PARAMETERS:      Name            RW  Usage
 *
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vSetUpWakeUpConditions(void) {
	// Set the LED to inputs to reduce power consumption
    APP_vSetLED(LED1, FALSE);
    APP_vSetLED(LED2, FALSE);

    vAHI_DioSetDirection(APP_BUTTONS_DIO_MASK, 0x00000000);
    vSaveDioStateBeforeDeepSleep();
}

/****************************************************************************
 *
 * NAME: PreSleep
 *
 * DESCRIPTION:
 *
 * PreSleep call back by the power manager before the controller put into sleep.
 *
 * PARAMETERS:      Name            RW  Usage
 *
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PWRM_CALLBACK(PreSleep) {
    DBG_vPrintf(TRACE_START,"\nAPP Start: Sleeping, Tick Timer = %d\n", u32AHI_TickTimerRead());
    // Put off the LEDs Indicators to All off if the device is sleeping while the state is not running,
    // In running the state LED will be used to retain the occupied - unoccupied state.

    APP_vSetLED(LED2, FALSE);

    // Set up wake up input
    vSetUpWakeUpConditions();

    // Save the MAC settings (will get lost though if we don't preserve RAM)
    vAppApiSaveMacSettings();

    // Put ZTimer module to sleep (stop tick timer)
    ZTIMER_vSleep();

    // Wait for any characters in the UART to be transmitted
    DBG_vUartFlush();

    // Disable UART
    vAHI_UartDisable(E_AHI_UART_0);

}

/****************************************************************************
 *
 * NAME: Wakeup
 *
 * DESCRIPTION:
 *
 * Wakeup call back by  power manager after the controller wakes up from sleep.
 *
 ****************************************************************************/
PWRM_CALLBACK(Wakeup) {
	/* Wait until FALSE i.e. on XTAL  - otherwise uart data will be at wrong speed */
	while (bAHI_GetClkSource() == TRUE);
	/* Now we are running on the XTAL, optimise the flash memory wait states */
	vAHI_OptimiseWaitStates();
	#ifndef PDM_EEPROM
		PDM_vWarmInitHW();
	#endif

    // Don't use RTS/CTS pins on UART0 as they are used for buttons
    vAHI_UartSetRTSCTS(E_AHI_UART_0, FALSE);
    DBG_vUartInit(DBG_E_UART_0, DBG_E_UART_BAUD_RATE_115200);

    DBG_vPrintf(TRACE_START, "\nAPP Start: APP: Woken up (CB)");
    DBG_vPrintf(TRACE_START, "\nAPP Start: APP: Warm Waking powerStatus = 0x%04x", u16AHI_PowerStatus());

    // If the power status is OK and RAM held while sleeping, restore the MAC settings
    if(u16AHI_PowerStatus() & (1 << 1)) {
        // Restore MAC settings (turns radio on)
        vMAC_RestoreSettings();
        DBG_vPrintf(TRACE_START, "\nAPP Start: APP: MAC settings restored");

        // Define HIGH_POWER_ENABLE to enable high power module
        #ifdef HIGH_POWER_ENABLE
            vAHI_HighPowerModuleEnable(TRUE, TRUE);
        #endif

        DBG_vPrintf(TRACE_START, "\nAPP Restarting..., Tick Timer = %d", u32AHI_TickTimerRead());
        APP_vSetUpHardware();

		#ifdef CLD_OTA
        	vWarmStartOTA();
		#endif
        ZTIMER_vWake();

    }

    // Ensure we stay awake long enough to handle any button events
    ZTIMER_eStart(u8TimerMinWake, ZTIMER_TIME_MSEC(50));
}
