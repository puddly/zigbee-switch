/*****************************************************************************
 *
 * MODULE:             JN-AN-1220 ZLO Sensor Demo
 *
 * COMPONENT:          app_reporting.c
 *
 * DESCRIPTION:        ZLO Demo : OccupancySensor Report (Implementation)
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
#include <string.h>
#include "dbg.h"
#include "ZTimer.h"
#include "app_main.h"
#include "pdum_gen.h"
#include "PDM.h"
#include "pdum_gen.h"
#include "app_common.h"
#include "PDM_IDs.h"
#include "zcl_options.h"
#include "app_zbp_utilities.h"
#include "zcl_common.h"
#include "app_reporting.h"
#include "App_OccupancySensor.h"
/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifndef DEBUG_REPORT
    #define TRACE_REPORT   FALSE
#else
    #define TRACE_REPORT   TRUE
#endif
/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/*There is just one attributes at this point - Occupancy Attribute */

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
extern const uint8 LEFT_BUTTON_ENDPOINT;
/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

#define NUMBER_OF_REPORTABLE_ATTRIBUTES  (1)

/*Just one reports for time being*/
PRIVATE tsReports asSavedReports[NUMBER_OF_REPORTABLE_ATTRIBUTES];
/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/


/****************************************************************************
 *
 * NAME: eRestoreReports
 *
 * DESCRIPTION:
 * Loads the reporting information from the EEPROM/PDM
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC PDM_teStatus eRestoreReports( void )
{
    /* Restore any report data that is previously saved to flash */
    uint16 u16ByteRead;
    PDM_teStatus eStatusReportReload = PDM_eReadDataFromRecord(PDM_ID_APP_REPORTS,
                                                              asSavedReports,
                                                              sizeof(asSavedReports),
                                                              &u16ByteRead);

    DBG_vPrintf(TRACE_REPORT,"\nAPP Report: eStatusReportReload=%d",eStatusReportReload);
    /* Restore any application data previously saved to flash */

    return  (eStatusReportReload);
}

/****************************************************************************
 *
 * NAME: vMakeSupportedAttributesReportable
 *
 * DESCRIPTION:
 * Makes the attributes reportable for Occupancy attribute
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vMakeSupportedAttributesReportable(void) {
	int i;
    for (i = 0; i < NUMBER_OF_REPORTABLE_ATTRIBUTES; i++) {
        uint16_t cluster_id = asSavedReports[i].u16ClusterID;
        tsZCL_AttributeReportingConfigurationRecord* record = &(asSavedReports[i].sAttributeReportingConfigurationRecord);

        eZCL_CreateLocalReport(LEFT_BUTTON_ENDPOINT, cluster_id, 0, TRUE, record);
    }
}

/****************************************************************************
 *
 * NAME: vLoadDefaultConfigForReportable
 *
 * DESCRIPTION:
 * Loads a default configuration
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vLoadDefaultConfigForReportable(void) {
    memset(asSavedReports, 0x00, sizeof(asSavedReports));

    int i;

    for (i = 0; i < NUMBER_OF_REPORTABLE_ATTRIBUTES; i++) {
        asSavedReports[i] = asDefaultReports[i];
    }

    PDM_eSaveRecordData(PDM_ID_APP_REPORTS, asSavedReports, sizeof(asSavedReports));

}


/****************************************************************************
 *
 * NAME: vSaveReportableRecord
 *
 * DESCRIPTION:
 * Loads a default configuration
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void vSaveReportableRecord(uint16 u16ClusterID, tsZCL_AttributeReportingConfigurationRecord* psAttributeReportingConfigurationRecord) {
    asSavedReports[0].u16ClusterID = u16ClusterID;

    memcpy( &(asSavedReports[0].sAttributeReportingConfigurationRecord),
            psAttributeReportingConfigurationRecord,
            sizeof(tsZCL_AttributeReportingConfigurationRecord));

    PDM_eSaveRecordData(PDM_ID_APP_REPORTS, asSavedReports, sizeof(asSavedReports));
}

/****************************************************************************
 *
 * NAME: vSendImmediateReport
 *
 * DESCRIPTION:
 * Method to send a report to all bound nodes.
 *
 ****************************************************************************/
PUBLIC void vSendImmediateReport(void) {
    DBG_vPrintf(TRACE_REPORT, "\nAPP Report: Sending Report");

    PDUM_thAPduInstance apdu = hZCL_AllocateAPduInstance();

    if (apdu == PDUM_INVALID_HANDLE) {
        DBG_vPrintf(TRACE_REPORT, "\nAPP Report: PDUM_INVALID_HANDLE");
    }

    tsZCL_Address sDestinationAddress;
    sDestinationAddress.eAddressMode = E_ZCL_AM_BOUND_NO_ACK;

    teZCL_Status report_status = eZCL_ReportAllAttributes(&sDestinationAddress, GENERAL_CLUSTER_ID_ONOFF, 1, 0, apdu);

    // Send the report with all attributes.
    if (report_status != E_ZCL_SUCCESS) {
    	DBG_vPrintf(TRACE_REPORT, "\nAPP Report: Error Sending Report: %x", report_status);
    }

    PDUM_eAPduFreeAPduInstance(apdu);

    DBG_vPrintf(TRACE_REPORT, "\nAPP Report: Report Sent");
}
