/*****************************************************************************
 *
 * MODULE:             JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:          App_OccupancySensor.c
 *
 * DESCRIPTION:        ZLO Demo Occupancy Sensor -Implementation
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
#include <jendefs.h>
#include "zps_gen.h"
#include "App_OccupancySensor.h"
#include "app_blink_led.h"
#include "dbg.h"
#include <string.h>

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
const uint8 LEFT_BUTTON_ENDPOINT = OCCUPANCYSENSOR_LEFT_BUTTON_ENDPOINT;
tsHA_OnOffOutputDevice sSensor;

/* define the default reports */
tsReports asDefaultReports[ZCL_NUMBER_OF_REPORTS] = \
{\
    {GENERAL_CLUSTER_ID_ONOFF,{0, E_ZCL_BOOL, E_CLD_ONOFF_ATTR_ID_ONOFF, ZLO_MIN_REPORT_INTERVAL, ZLO_MAX_REPORT_INTERVAL, 0, {0}}},\
};
/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: eApp_HA_RegisterEndpoint
 *
 * DESCRIPTION:
 * Register ZLO endpoints
 *
 * PARAMETER
 * Type                        Name                  Descirption
 * tfpZCL_ZCLCallBackFunction  fptr                  Pointer to ZCL Callback function
 *
 * RETURNS:
 * teZCL_Status
 *
 ****************************************************************************/
PUBLIC teZCL_Status eApp_ZLO_RegisterEndpoint(tfpZCL_ZCLCallBackFunction fptr)
{
    return eHA_RegisterOnOffOutputEndPoint(OCCUPANCYSENSOR_LEFT_BUTTON_ENDPOINT,
                                              fptr,
                                              &sSensor);
}

/****************************************************************************
 *
 * NAME: vAPP_ZCL_DeviceSpecific_Init
 *
 * DESCRIPTION:
 * ZCL Device Specific initialization
 *
 * PARAMETER: void
 *
 * RETURNS: void
 *
 ****************************************************************************/
PUBLIC void vAPP_ZCL_DeviceSpecific_Init(void)
{
    /* Initialise the strings in Basic */
    memcpy(sSensor.sBasicServerCluster.au8ManufacturerName, "NXP", CLD_BAS_MANUF_NAME_SIZE);
    memcpy(sSensor.sBasicServerCluster.au8ModelIdentifier, "ZLO-OccupancySensor", CLD_BAS_MODEL_ID_SIZE);
    memcpy(sSensor.sBasicServerCluster.au8DateCode, "20160210", CLD_BAS_DATE_SIZE);
    memcpy(sSensor.sBasicServerCluster.au8SWBuildID, "4000-0001", CLD_BAS_SW_BUILD_SIZE);
    memcpy(sSensor.sBasicServerCluster.au8ProductURL, "www.nxp.com", CLD_BAS_URL_SIZE);
    memcpy(sSensor.sBasicServerCluster.au8ProductCode, "1234", CLD_BAS_PCODE_SIZE);
    sSensor.sBasicServerCluster.eGenericDeviceType = E_CLD_BAS_GENERIC_DEVICE_TYPE_WALL_SWITCH;

    /* Initialise the strings in Occupancy Cluster */
    //sSensor.sOccupancySensingServerCluster.eOccupancySensorType = E_CLD_OS_SENSOR_TYPE_PIR;
    //sSensor.sOccupancySensingServerCluster.u8Occupancy = 0;
}

/****************************************************************************
 *
 * NAME: vAPP_ZCL_DeviceSpecific_SetIdentifyTime
 *
 * DESCRIPTION:
 * ZCL Device Specific setting of identify time
 *
 * PARAMETER:
 * uint16 u16Time Identify time duration
 *
 * RETURNS: void
 *
 ****************************************************************************/
PUBLIC void vAPP_ZCL_DeviceSpecific_SetIdentifyTime(uint16 u16Time)
{
    sSensor.sIdentifyServerCluster.u16IdentifyTime=u16Time;
}
/****************************************************************************
 *
 * NAME: vAPP_ZCL_DeviceSpecific_UpdateIdentify
 *
 * DESCRIPTION:
 * ZCL Device Specific Identify Updates
 *
 * PARAMETER: void
 *
 * RETURNS: void
 *
 ****************************************************************************/
PUBLIC void vAPP_ZCL_DeviceSpecific_UpdateIdentify(void)
{
    APP_vSetLED(LED2, sSensor.sIdentifyServerCluster.u16IdentifyTime%2);
}
/****************************************************************************
 *
 * NAME: vAPP_ZCL_DeviceSpecific_IdentifyOff
 *
 * DESCRIPTION:
 * ZCL Device Specific stop identify
 *
 * PARAMETER: void
 *
 * RETURNS: void
 *
 ****************************************************************************/
PUBLIC void vAPP_ZCL_DeviceSpecific_IdentifyOff(void)
{
    vAPP_ZCL_DeviceSpecific_SetIdentifyTime(0);
    APP_vSetLED(LED2, 0);
}

/****************************************************************************
 *
 * NAME: app_u8GetDeviceEndpoint
 *
 * DESCRIPTION:
 * Returns the application endpoint
 *
 * PARAMETER: void
 *
 * RETURNS: void
 *
 ****************************************************************************/
PUBLIC uint8 app_u8GetDeviceEndpoint( void)
{
    return OCCUPANCYSENSOR_LEFT_BUTTON_ENDPOINT;
}
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
