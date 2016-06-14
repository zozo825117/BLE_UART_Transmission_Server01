/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
//****************************************************************************
// header file
//****************************************************************************/
#include "common.h"
#include "comm.h"

//#define test
//****************************************************************************
// Variable
//****************************************************************************/
u_comm_state uCommState;
uint8 Buffer[300],UartLen=0;
uint8 *UartPtr;
//****************************************************************************
// Function
//****************************************************************************/
void CommUartRxReady(void);

/*******************************************************************************
* Function Name: Transport_RX_ISR
********************************************************************************
*
* Summary:
*  Handles the Interrupt Service Routine for the UART RX.
*  Contains cmdRxState m/c to check for valid command reciept and triggers flag
*
* Parameters:
*  NONE
*
* Return:
*  NONE
*
* Theory:
*  NONE
*
* Side Effects:
*  Command bufffers and newCmdRxDoneFlag will be modified
*
* Note:
*
*******************************************************************************/
CY_ISR(Transport_RX_ISR)
{
    //volatile static uint32 expectedPktLength;
    //uint8 *currentPktBuf = primaryCmdRxBuf;
        
    if(UART_DEB_CHECK_INTR_RX_MASKED(UART_DEB_INTR_RX_BREAK_DETECT))
    {    
        /*Break can occur on start of each command, reset cmdRxState m/c & RX buffers*/
        //cmdRxState = STATE_HEADER;
        CommUartRxReady();
        UART_DEB_ClearRxInterruptSource(UART_DEB_INTR_RX_BREAK_DETECT);
    }
    else if(UART_DEB_CHECK_INTR_RX_MASKED(UART_DEB_INTR_RX_NOT_EMPTY))
    {    

        if(0 != UART_DEB_SpiUartGetRxBufferSize())
        {
            /*Get each input byte into buffer, before checking for cmdRxState transition*/
      		  *UartPtr++ = (uint8)UART_DEB_SpiUartReadRxData(); 		// store received data to Buf[]
      		  UartLen++;
        		 if(UartLen>=200)
        		 {
        				 UartLen = 200;
        				 UartPtr = &Buffer[201];
        		 }
            /*restart the timer on each recieved byte*/
            Timer_10ms_Stop();
            Timer_10ms_Start();


        }
        UART_DEB_ClearRxInterruptSource(UART_DEB_INTR_RX_NOT_EMPTY);
    }
    else
    {
        /*Other UART inturrepts are not handled*/
    }
}
/*******************************************************************************
* Function Name: Transport_Timer_ISR
********************************************************************************
*
* Summary:
*  Handles the Interrupt Service Routine for the UART Timer.
*  Resets RX command state m/c for every 10ms delay on RX byte
*  If RX packet has started, there should not be more than 10ms delay between each RX byte with-in the packet
*
* Parameters:
*  NONE
*
* Return:
*  NONE
*
* Theory:
*  NONE
*
* Side Effects:
*  State m/c and other parameters will be reset
*
* Note:
*
*******************************************************************************/
CY_ISR(Transport_Timer_ISR)
{
    uint32 intrSrc = Timer_10ms_GetInterruptSource();

    if(Timer_10ms_INTR_MASK_TC == intrSrc)
    {
        /*Reset RX state m/c*/
        uCommState.Bit.UartRxFinished = ENABLED;
        Buffer[0] = UartLen;
        Timer_10ms_ClearInterrupt(intrSrc);
    }
    else
    {
        /*Ignore other interrupt sources*/
    }

    isr_10ms_ClearPending();
}

///////////////////////////////////////////////////////////////////////////////
/// \brief			UartRxReady
/// \param			none
/// \return 		none
/// \deprecated 
///////////////////////////////////////////////////////////////////////////////
void CommUartRxReady(void)
{
    UartPtr = &Buffer[1];          // Initial Ptr
    UartLen = 0;                   // return received bytes 
    uCommState.Bit.UartRxRDY = ENABLED;
}
///////////////////////////////////////////////////////////////////////////////
/// \brief			UartRxReady
/// \param			none
/// \return 		none
/// \deprecated 
///////////////////////////////////////////////////////////////////////////////
void CommUartTxByte (void) 
{
	UartLen = Buffer[0];
	UartPtr = &Buffer[1];
  for(;UartLen>0;UartLen--)
    	{
        UART_DEB_UartPutChar(*UartPtr++);
    	}
}
///////////////////////////////////////////////////////////////////////////////
/// \brief			UartRxReady
/// \param			none
/// \return 		none
/// \deprecated 
///////////////////////////////////////////////////////////////////////////////
void CommBLERxReady(void)
{
    uCommState.Bit.BLERxRDY = ENABLED;
}
///////////////////////////////////////////////////////////////////////////////
/// \brief			MonitorBLE
/// \param			none
/// \return 			none
/// \deprecated monitor ble send usd uart tx
///////////////////////////////////////////////////////////////////////////////
void CommMonitorBLE(void)
{
    if(uCommState.Bit.BLERxFinshed == ENABLED)
			{	    
				uCommState.Bit.BLERxFinshed = DISABLED;

				CommUartTxByte();
				// Then switch to Uart RX state
				CommUartRxReady();            
				BLE_LED_OFF;
	
	      // Then switch to RF RX state
	      CommBLERxReady();
		}
}
///////////////////////////////////////////////////////////////////////////////
/// \brief			CommMonitorUart
/// \param			none
/// \return 			none
/// \deprecated monitor uart  send use ble
///////////////////////////////////////////////////////////////////////////////
void CommMonitorUart(void)
{
    CYBLE_API_RESULT_T apiResult;
    CYBLE_GATT_HANDLE_VALUE_PAIR_T CustomNotificationhandle;
  
    if(uCommState.Bit.UartRxFinished == ENABLED)
    {
        uCommState.Bit.UartRxFinished = DISABLED;
             
				uCommState.Bit.BLERxRDY = DISABLED;
      
      if(Buffer[1] == BUTTON_UPDATA_HANDLE)
      {
				CustomNotificationhandle.attrHandle = BUTTON_UPDATA_HANDLE;
				CustomNotificationhandle.value.val = &Buffer[2];
				CustomNotificationhandle.value.len = Buffer[0]-1;
				CustomNotificationhandle.value.actualLen = Buffer[0];
        printf("ble BUTTON_UPDATA_HANDLE \r\n");
      }
      else if(Buffer[1] == TMP_ML_HANDLE)
      {
				CustomNotificationhandle.attrHandle = TMP_ML_HANDLE;
				CustomNotificationhandle.value.val = &Buffer[2];
				CustomNotificationhandle.value.len = Buffer[0]-1;
				CustomNotificationhandle.value.actualLen = Buffer[0];
        printf("ble TMP_ML_HANDLE \r\n");
      }
      else if(Buffer[1] == NOW_ML_HANDLE)
      {
				CustomNotificationhandle.attrHandle = NOW_ML_HANDLE;
				CustomNotificationhandle.value.val = &Buffer[2];
				CustomNotificationhandle.value.len = Buffer[0]-1;
				CustomNotificationhandle.value.actualLen = Buffer[0];
        printf("ble TMP_ML_HANDLE \r\n");
      }
      else
      { 
				CustomNotificationhandle.attrHandle = UART_TX_HANDLE;
				CustomNotificationhandle.value.val = &Buffer[1];
				CustomNotificationhandle.value.len = Buffer[0];
				CustomNotificationhandle.value.actualLen = Buffer[0];
      }
				
#if 1

				/* Send notification to client using previously filled structure */
				apiResult = CyBle_GattsNotification(cyBle_connHandle, &CustomNotificationhandle);
#else

				apiResult = CyBle_GattcWriteWithoutResponse(cyBle_connHandle,&CustomNotificationhandle);
#endif
				/* Send uart receive handle as attribute for read by central device */
				CyBle_GattsWriteAttributeValue(&CustomNotificationhandle,FALSE,&cyBle_connHandle,FALSE);
#ifdef test
				if(apiResult != CYBLE_ERROR_OK)
        {
            printf("ble send Error: %x \r\n", apiResult);
        }
				else
					{

						printf("ble send ok len: %x \r\n", CustomNotificationhandle.value.len);
					}
#endif

        CommUartRxReady();
        Uart_LED_OFF;
        // Then switch to RF RX state
        CommBLERxReady();
    }

}
///////////////////////////////////////////////////////////////////////////////
/// \brief			CommInit
/// \param			none
/// \return 		none
/// \deprecated 
///////////////////////////////////////////////////////////////////////////////
void CommInit(void)
{
  UART_DEB_Start();               /* Start communication component */
  //UART_DEB_SetCustomInterruptHandler(Transport_RX_ISR);
    
  //isr_10ms_StartEx(Transport_Timer_ISR);
  
  CommUartRxReady();
  CommBLERxReady();
}


/* [] END OF FILE */