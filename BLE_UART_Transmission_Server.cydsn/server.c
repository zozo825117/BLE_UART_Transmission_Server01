/*******************************************************************************
* File Name: server.c
*
* Version: 1.0
*
* Description:
*  Common BLE application code for server devices.
*
*******************************************************************************
* Copyright 2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include <project.h>
#include <stdio.h>
#include "common.h"

CYBLE_API_RESULT_T apiResult;

void StartAdvertisement(void)
{
    uint16 i;
    CYBLE_GAP_BD_ADDR_T localAddr;
    apiResult = CyBle_GappStartAdvertisement(CYBLE_ADVERTISING_FAST);
    if(apiResult != CYBLE_ERROR_OK)
    {
        printf("StartAdvertisement API Error: %x \r\n", (int) apiResult);
    }
    else
    {
        printf("Start Advertisement with addr: ");
        CyBle_GetDeviceAddress(&localAddr);
        for(i = CYBLE_GAP_BD_ADDR_SIZE; i > 0u; i--)
        {
            printf("%2.2x", localAddr.bdAddr[i-1]);
        }
        printf("\r\n");
    }
}

void ServerDebugOut(uint32 event, void* eventParam)
{
    uint16 i;
    switch(event)
    {
        case CYBLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
            printf("EVT_ADVERTISING \r\n");
            break;

        case CYBLE_EVT_GATTS_WRITE_REQ:
            printf("EVT_GATT_WRITE_REQ: %x = ",((CYBLE_GATTS_WRITE_REQ_PARAM_T *)eventParam)->handleValPair.attrHandle);
            for(i = 0; i < ((CYBLE_GATTS_WRITE_REQ_PARAM_T *)eventParam)->handleValPair.value.len; i++)
            {
                printf("%2.2x ", ((CYBLE_GATTS_WRITE_REQ_PARAM_T *)eventParam)->handleValPair.value.val[i]);
            }
            printf("\r\n");
            break;

        case CYBLE_EVT_GATTS_XCNHG_MTU_REQ:
            printf("EVT_GATTS_XCNHG_MTU_REQ \r\n");
            break;

        default:
        #if (0)
            printf("unknown event: %lx \r\n", event);
        #endif
            break;
    }
}



/* [] END OF FILE */
