/*****************************************************************************
 *
 * MODULE:          JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:       app_PIR_OpenCollector_events.c
 *
 * DESCRIPTION:     ZLO Demo: Driver behaviour of the Open Collector PIR Sensor
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
 * Copyright NXP B.V. 2016. All rights reserved
 *
 ***************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include "dbg.h"
#include "app_sleep_handler.h"
#include "app_PIR_events.h"
#include "app_occupancy_sensor_state_machine.h"
#include "app_blink_led.h"
#include "AppHardwareApi.h"
#include "app_zlo_sensor_node.h"
#include "app_reporting.h"
/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifdef DEBUG_PIR_EVENTS
    #define TRACE_PIR_EVENTS   TRUE
#else
    #define TRACE_PIR_EVENTS  FALSE
#endif
/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void vHandlePIROpenCollectorOccupiedToUnoccupiedTimerExpired(void);
/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PRIVATE teRunningTimer u8RunningTimerType = E_APP_TIMER_NONE;
/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
/****************************************************************************
 *
 * NAME: vHandlePIROpenCollectorFallingEdgeEvent
 *
 * DESCRIPTION:
 * We have received a falling edge event which means we are now occupied.
 * Change the occupancy to occupied, turn the LED on, send the report (if we
 * were previously unoccupied. If we were already occupied, stop the occupied
 * to unoccupied timer.
 *
 ****************************************************************************/
PUBLIC void vHandleFallingEdgeEvent(void)
{
    DBG_vPrintf(TRACE_PIR_EVENTS,"\nAPP PIR Event: Falling Edge Event");
	/* We were previously unoccupied*/
	DBG_vPrintf(TRACE_PIR_EVENTS,"\nAPP PIR Event: Occupied");
	sSensor.sOnOffServerCluster.bOnOff = FALSE;
	vSendImmediateReport();
	APP_vSetLED(LED1, sSensor.sOnOffServerCluster.bOnOff);
}

/****************************************************************************
 *
 * NAME: vHandlePIROpenCollectorRisingEdgeEvent
 *
 * DESCRIPTION:
 * We have received a rising edge event which means we are now unoccupied.
 * Start the occupied to unoccupied timer and attempt to go to sleep.
 *
 ****************************************************************************/
PUBLIC void vHandleRisingEdgeEvent(void)
{
    DBG_vPrintf(TRACE_PIR_EVENTS,"\nAPP PIR Event: Falling Edge Event");
	/* We were previously unoccupied*/
	DBG_vPrintf(TRACE_PIR_EVENTS,"\nAPP PIR Event: Occupied");
	sSensor.sOnOffServerCluster.bOnOff = TRUE;
	vSendImmediateReport();
	APP_vSetLED(LED1, sSensor.sOnOffServerCluster.bOnOff);
}

/****************************************************************************
 *
 * NAME: vHandlePIROpenCollectorNewJoinEvent
 *
 * DESCRIPTION:
 * This method is called when we have just joined a new network, rejoined
 * a network or we have just come out of find and bind. If we are occupied,
 * send a report out, if we are unoccupied, just start the occupied to unoccupied
 * delay timer so we aren't potentially turning lights off prematurely.
 *
 ****************************************************************************/
PUBLIC void vHandleNewJoinEvent(void)
{

    DBG_vPrintf(TRACE_PIR_EVENTS,"\nAPP PIR Event: New join (RunningTimerType=%d) (Occupancy=%02x)", u8RunningTimerType, sSensor.sOnOffServerCluster.bOnOff);
    switch (u8RunningTimerType)
    {

    case E_APP_TIMER_NONE:
    case E_APP_TIMER_OCCUPIED_TO_UNOCCUPIED:
        DBG_vPrintf(TRACE_PIR_EVENTS,"\nAPP PIR Event: Sending a report");
        vSendImmediateReport();
        DBG_vPrintf(TRACE_PIR_EVENTS,"\nAPP PIR Event: Starting Occupied to Unoccupied timer");
        u8RunningTimerType = E_APP_TIMER_OCCUPIED_TO_UNOCCUPIED;
        break;

    case E_APP_TIMER_UNOCCUPIED_TO_OCCUPIED:
    default:
        break;

    }
}

/****************************************************************************
 *
 * NAME: vHandlePIROpenCollectorWakeTimerExpired
 *
 * DESCRIPTION:
 * Handles a wake timer expired event. We check to see what type of timer
 * event we had running and processes it accordingly.
 *
 ****************************************************************************/
PUBLIC void vHandleWakeTimeoutEvent(void)
{
    switch (u8RunningTimerType)
    {

    case E_APP_TIMER_OCCUPIED_TO_UNOCCUPIED:
        vHandlePIROpenCollectorOccupiedToUnoccupiedTimerExpired();
        break;

    case E_APP_TIMER_UNOCCUPIED_TO_OCCUPIED:
    default:
        break;

    }

}
/****************************************************************************/
/***        Local Functions			                                      ***/
/****************************************************************************/
/****************************************************************************
 *
 * NAME: vHandlePIROpenCollectorOccupiedToUnoccupiedTimerExpired
 *
 * DESCRIPTION:
 * When a timer goes from occupied to unoccupied this method changes it to
 * unoccupied, turns the LED on and sends out a report.
 *
 ****************************************************************************/
PRIVATE void vHandlePIROpenCollectorOccupiedToUnoccupiedTimerExpired(void)
{
    DBG_vPrintf(TRACE_PIR_EVENTS,"\nAPP PIR Event timer expired: Unoccupied = %d", sSensor.sOnOffServerCluster.bOnOff);
}
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
