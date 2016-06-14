/*******************************************************************************
* File Name: main.c
*
* Version: 1.0
*
* Description:
*  This project demonstrates the operation of the Heart Rate Profile
*  in Server (Peripheral) role.
*
*******************************************************************************
* Copyright 2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "project.h"
#include "common.h"
#include "comm.h"

#define BLE_DISCONNECTED_STATE			0x00
#define BLE_ADV_STATE					0x01
#define BLE_CONNECTED_STATE				0x02
#define ADV_LED_BLINK_PERIOD			20000
#define CONN_LED_PERIOD					90000


volatile uint32 mainTimer = 0;
uint32 SysTickTimer = 0;
uint8 InterruptHpn;

CYBLE_API_RESULT_T apiResult;

uint8 ble_state;


uint8 test_buf[]={
	0,1,2,3,4,5,6,7,8,9,
	10,11,12,13,14,15,16,17,18,19,
	20,21,22,23,24,25,26,27,28,29,
	30,31,32,33,34,35,36,37,38,39,
	40,41,42,43,44,45,46,47,48,49,
	50,51,52,53,54,55,56,57,58,59,
	60,61,62,63,64,65,66,67,68,69,
	70,71,72,73,74,75,76,77,78,79,
	80,81,82,83,84,85,86,87,88,89,
	90,91,92,93,94,95,96,97,98,99,

};

uint8 test_ptr=0,send_flag=0;

uint16 connIntv;


/* General application callback */
void AppCallBack(uint32 event, void* eventParam)
{
	CYBLE_GATTS_WRITE_REQ_PARAM_T *wrReqParam;
	//CYBLE_GATT_HANDLE_VALUE_PAIR_T zozoTThandle;
	uint8 len;

#ifdef DEBUG_OUT   
		//printf("GAC event %lu \r\n",event);  
    DebugOut(event, eventParam);
#endif

    switch(event)
    {
        case CYBLE_EVT_STACK_ON:
						printf("EVT_STACK_ON \r\n");
					goto start_advert;
        case CYBLE_EVT_GAP_DEVICE_DISCONNECTED:
						//ble_state = BLE_DISCONNECTED_STATE;
        case CYBLE_EVT_TIMEOUT:
start_advert:
            /* Put the device into discoverable mode so that remote can search it. */
            StartAdvertisement();
            //Advertising_LED_Write(LED_ON);
						ble_state = BLE_ADV_STATE;
            test_ptr=0;
            break;
						

				case CYBLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
						printf("EVT_GAPP_ADVERTISEMENT_START_STOP \r\n");
					break;
					
				case CYBLE_EVT_GATT_CONNECT_IND:
						printf("EVT_GATT_CONNECT_IND \r\n");
					break;
					
				case CYBLE_EVT_GATTS_XCNHG_MTU_REQ:
						printf("EVT_GATTS_XCNHG_MTU_REQ \r\n");
					break;

        case CYBLE_EVT_GAP_DEVICE_CONNECTED:
            //Advertising_LED_Write(LED_OFF);
            ble_state = BLE_CONNECTED_STATE;
						connIntv = ((CYBLE_GAP_CONN_PARAM_UPDATED_IN_CONTROLLER_T *)eventParam)->connIntv * 1000* 5u /4u;
            printf("EVT_GAP_DEVICE_CONNECTED: connIntv = %d us \r\n", connIntv);            
					break;

				case CYBLE_EVT_GATTS_WRITE_CMD_REQ:
        case CYBLE_EVT_GATTS_WRITE_REQ:
					  //printf("evt gatts write req \r\n");
						wrReqParam = (CYBLE_GATTS_WRITE_REQ_PARAM_T *) eventParam;
						if(wrReqParam->handleValPair.attrHandle == UART_TX_HANDLE)
							{
								len = wrReqParam->handleValPair.value.len;
								Buffer[0] = len;
								memcpy(&Buffer[1],wrReqParam->handleValPair.value.val,len);
								uCommState.Bit.BLERxFinshed = ENABLED;
								//printf("len %d buf[0] %d  \r\n", len,Buffer[0]);
							}

					break;

        default:
            break;
    }
}


/*******************************************************************************
* Function Name: Timer_Interrupt
********************************************************************************
*
* Summary:
*  Handles the Interrupt Service Routine for the WDT timer.
*  It is called from common WDT ISR located in BLE component. 
*
*******************************************************************************/
void Timer_Interrupt(void)
{
    if(CySysWdtGetInterruptSource() & WDT_INTERRUPT_SOURCE)
    {
        static uint8 led = LED_OFF;
        
        /* Blink LED to indicate that device advertises */
        if(CYBLE_STATE_ADVERTISING == CyBle_GetState())
        {
            led ^= LED_OFF;
            Advertising_LED_Write(led);
        }
        
        /* Indicate that timer is raised to the main loop */
        mainTimer++;
        
        /* Clears interrupt request  */
        CySysWdtClearInterrupt(WDT_INTERRUPT_SOURCE);
    }
}
/*******************************************************************************
* Define Interrupt service routine and allocate an vector to the Interrupt
********************************************************************************/
CY_ISR(InterruptHandler)
{
    /* Check interrupt source and clear Inerrupt */
    InterruptHpn = Timer_GetInterruptSourceMasked();
   	if (InterruptHpn == Timer_INTR_MASK_CC_MATCH)
    {
        Timer_ClearInterrupt(Timer_INTR_MASK_CC_MATCH);      
        //test_1_Write(~ test_1_Read());
    }
    else
    {
        Timer_ClearInterrupt(Timer_INTR_MASK_TC);
        test_2_Write(~ test_2_Read());
				SysTickTimer++;
				
    }
}


/*******************************************************************************
* Function Name: WDT_Start
********************************************************************************
*
* Summary:
*  Configures WDT(counter 2) to trigger an interrupt every second.
*
*******************************************************************************/
void WDT_Start(void)
{
   WdtIsr_StartEx(&Timer_Interrupt);
    /* Unlock the WDT registers for modification */
    CySysWdtUnlock(); 
    /* Setup ISR callback */
    //CyBle_WdtRegisterIsrCallback(&Timer_Interrupt);
    /* Write the mode to generate interrupt on match */
    CySysWdtWriteMode(WDT_COUNTER, CY_SYS_WDT_MODE_INT);
    /* Configure the WDT counter clear on a match setting */
    CySysWdtWriteClearOnMatch(WDT_COUNTER, WDT_COUNTER_ENABLE);
    /* Configure the WDT counter match comparison value */
    CySysWdtWriteMatch(WDT_COUNTER, WDT_1SEC);
    /* Reset WDT counter */
    CySysWdtResetCounters(WDT_COUNTER);
    /* Enable the specified WDT counter */
    CySysWdtEnable(WDT_COUNTER_MASK);
    /* Lock out configuration changes to the Watchdog timer registers */
    CySysWdtLock();    
}
/*******************************************************************************
* Function Name: HandleLEDs
********************************************************************************
* Summary:
*        Update LED states as per BLE Status
*
* Parameters:
*  state: present BLE state.
*
* Return:
*  void
*
*******************************************************************************/
void HandleLEDs(uint8 state)
{

		static uint16 led_counter = TRUE;
		static uint32 led_conn_counter = TRUE;
		static uint8 on_off_status;
		static uint8 conn_status_update = FALSE;

		switch(state)
		{
			case BLE_CONNECTED_STATE:
				if(conn_status_update)
				{
					conn_status_update = FALSE;
					led_conn_counter = CONN_LED_PERIOD;
				}
				else
				{
					if(led_conn_counter != FALSE)
					{
						if(CONN_LED_PERIOD == led_conn_counter)
						{
							Advertising_LED_Write(LED_ON);
						}
						else if(led_conn_counter == 1)
						{
							Advertising_LED_Write(LED_OFF);

						}
						
						led_conn_counter--;
					}
				}
			break;
			
			case BLE_ADV_STATE:
				if((--led_counter) == FALSE)
				{
					led_counter = ADV_LED_BLINK_PERIOD;
					if(FALSE == on_off_status)
					{
						on_off_status = TRUE;
						Advertising_LED_Write(LED_ON);
		
					}
					else
					{
						on_off_status = FALSE;
						Advertising_LED_Write(LED_OFF);
					}
				}
				
				conn_status_update = TRUE;
			break;
			
			case BLE_DISCONNECTED_STATE:
				Advertising_LED_Write(LED_OFF);
				led_counter = FALSE;
				
				conn_status_update = TRUE;
			break;
			
			default:
			
			break;
		}

}


int main()
{
		//uint8 i;
    CYBLE_LP_MODE_T lpMode;

    CyGlobalIntEnable;

    CommInit();
    printf("BLE Uart Transmission Server Example Project \r\n");
    
    Advertising_LED_Write(LED_OFF);
    Uart_LED_OFF;
    BLE_LED_OFF;

    /* Start CYBLE component and register generic event handler */
    apiResult = CyBle_Start(AppCallBack);
    if(apiResult != CYBLE_ERROR_OK)
    {
        printf("CyBle_Start API Error: %x \r\n", apiResult);
    }

    WDT_Start();
		
		/* Enable the Interrupt component connected to interrupt */
		TC_CC_ISR_StartEx(InterruptHandler);
		
		/* Start the components */
		Timer_Start();
    
    /***************************************************************************
    * Main polling loop
    ***************************************************************************/
    while(1)
    {
        if(CyBle_GetState() != CYBLE_STATE_INITIALIZING)
        {
            /* Enter DeepSleep mode between connection intervals */
            lpMode = CyBle_EnterLPM(CYBLE_BLESS_DEEPSLEEP);
            if(lpMode == CYBLE_BLESS_DEEPSLEEP) 
            {
                CySysPmSleep();
                /* Handle advertising led blinking */
                HandleLEDs(ble_state);
            }
        }
        
        /***********************************************************************
        * Wait for connection established with Central device
        ***********************************************************************/
        if(CyBle_GetState() == CYBLE_STATE_CONNECTED)
        {
            /*******************************************************************
            *  Periodically measure a battery level and temperature and send 
            *  results to the Client
            *******************************************************************/    
            CommMonitorUart();
            CommMonitorBLE();
            
            #if 0
            if(mainTimer != 0u)
            {
                mainTimer = 0u;

                if(storeBondingData == ENABLED)
                {
                    cystatus retValue;
                    retValue = CyBle_StoreBondingData(0u);
                    printf("Store bonding data, status: %lx \r\n", retValue);
                    storeBondingData = DISABLED;
                }
    
            }
            #endif
            
            
        }
        
        /*******************************************************************
        *  Processes all pending BLE events in the stack
        *******************************************************************/        
        CyBle_ProcessEvents();

    }
}


/* [] END OF FILE */
