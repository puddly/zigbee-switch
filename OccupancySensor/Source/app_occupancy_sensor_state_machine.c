/*****************************************************************************
 *
 * MODULE:          JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:       app_sensor_state_machine.c
 *
 * DESCRIPTION:     ZLO Demo: Processes all stack events depending on it's state
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
#include "dbg.h"
#include "app_occupancy_sensor_state_machine.h"
#include "pwrm.h"
#include "App_OccupancySensor.h"
#include "app_nwk_event_handler.h"
#include "AppHardwareApi.h"
#include "app_zbp_utilities.h"
#include "pdum_gen.h"
#include "PDM.h"
#include "bdb_api.h"
#include "app_common.h"
#include "app_zlo_sensor_node.h"
#include "zps_gen.h"
#ifdef APP_NTAG_ICODE
#include "app_ntag_icode.h"
#endif

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifdef DEBUG_SENSOR_STATE
    #define TRACE_SENSOR_STATE   TRUE
#else
    #define TRACE_SENSOR_STATE   FALSE
#endif
/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PUBLIC PDM_tsRecordDescriptor sDevicePDDesc;
/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
/****************************************************************************
 *
 * NAME: vAppHandleRunning
 *
 * DESCRIPTION:
 * Is called to handle stack events when we are currently in the network
 *
 ****************************************************************************/
PUBLIC void vAppHandleRunning(ZPS_tsAfEvent* psStackEvent)
{

    DBG_vPrintf(TRACE_SENSOR_STATE, "\nAPP State Event: Running: Event %d", psStackEvent->eType);

    switch(psStackEvent->eType)
    {

    case ZPS_EVENT_NWK_JOINED_AS_ENDDEVICE:
        vHandleNetworkJoinEndDevice();
        #ifdef APP_NTAG_ICODE
        {
            /* Not a rejoin ? */
            if (FALSE == psStackEvent->uEvent.sNwkJoinedEvent.bRejoin)
            {
                /* Write network data to tag */
                APP_vNtagStart(LEFT_BUTTON_ENDPOINT);
            }
        }
        #endif
        break;

    case ZPS_EVENT_NWK_LEAVE_INDICATION:
        vHandleNetworkLeave( psStackEvent);
        break;

    case ZPS_EVENT_NWK_POLL_CONFIRM:
        vHandlePollResponse( psStackEvent);
        break;

    case ZPS_EVENT_NWK_LEAVE_CONFIRM:
        vHandleNetworkLeaveConfirm( psStackEvent);
        break;

#ifdef CLD_OTA
    case ZPS_EVENT_APS_DATA_INDICATION:
    	if ((psStackEvent->uEvent.sApsDataIndEvent.eStatus == ZPS_E_SUCCESS) &&
    	                (psStackEvent->uEvent.sApsDataIndEvent.u8DstEndpoint == 0))
    	 	 {
    	            // Data Ind for ZDp Ep
    	            if (ZPS_ZDP_MATCH_DESC_RSP_CLUSTER_ID == psStackEvent->uEvent.sApsDataIndEvent.u16ClusterId)
    	            {
    	                DBG_vPrintf(TRACE_SENSOR_STATE, "\r\nNODE: ZPS_ZDP_MATCH_DESC_RSP_CLUSTER_ID");
    	                vHandleMatchDescriptor(psStackEvent);
    	                if(APP_bPersistantPolling != TRUE)
    	                {
    	                    ZTIMER_eStop(u8TimerPoll);
    	                }
    	            } else if (ZPS_ZDP_IEEE_ADDR_RSP_CLUSTER_ID == psStackEvent->uEvent.sApsDataIndEvent.u16ClusterId) {
    	                vHandleIeeeAddressRsp(psStackEvent);
    	                if(APP_bPersistantPolling != TRUE)
    	                {
    	                    ZTIMER_eStop(u8TimerPoll);
    	                }
    	            }
    	        }
    	break;
#endif
    default:
        break;

    }

}

/****************************************************************************
 *
 * NAME: vAppHandleStartup
 *
 * DESCRIPTION:
 * Is called to handle stack events when we are currently not in the network
 *
 ****************************************************************************/
PUBLIC void vAppHandleStartup(void)
{
    DBG_vPrintf(TRACE_SENSOR_STATE, "\nAPP State Event: StartUp");
    BDB_eNsStartNwkSteering();
    sDeviceDesc.eNodeState = E_JOINING_NETWORK;
}
/****************************************************************************
 *
 * NAME: vStartWakeTimer
 *
 * DESCRIPTION:
 * Starts wake timer 0
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vStartWakeTimer(uint16 u16Tick)
{
    uint32 u32Ticks = 0;
    uint64 u64AdjustedTicks = 0;
    /* Disable Wake timer interrupt to use for calibration */
    vAHI_WakeTimerEnable(E_AHI_WAKE_TIMER_0, FALSE);
    uint32 u32CalibrationValue = u32AHI_WakeTimerCalibrate();
    (void)u8AHI_WakeTimerFiredStatus();
    /* Enable timer to use for sleeping */
    vAHI_WakeTimerEnable(E_AHI_WAKE_TIMER_0, TRUE);
    u64AdjustedTicks = u16Tick * APP_TICKS_PER_SECOND ;
    u64AdjustedTicks = u64AdjustedTicks * 10000 / u32CalibrationValue;
    if (u64AdjustedTicks > 0xffffffff)
    {
        /* overflowed os limit to max uint32*/
        u32Ticks = 0xfffffff;
    } else {
        u32Ticks = (uint32)u64AdjustedTicks;
    }
    DBG_vPrintf(TRACE_SENSOR_STATE, "\nAPP Sensor State: WakeTimer : u32CalibrationValue = %d u32Ticks %08x", u32CalibrationValue,u32Ticks);
    vAHI_WakeTimerStop(E_AHI_WAKE_TIMER_0);
    vAHI_WakeTimerStart(E_AHI_WAKE_TIMER_0, u32Ticks);
}


/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
