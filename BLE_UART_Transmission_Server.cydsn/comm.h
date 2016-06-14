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
#include "project.h"

//****************************************************************************
// marco
//****************************************************************************/
#define Uart_LED_OFF            Uart_LED_Write(1)
#define Uart_LED_ON             Uart_LED_Write(0)

#define BLE_LED_OFF             BLE_LED_Write(1)
#define BLE_LED_ON              BLE_LED_Write(0)

//ble
//#define EN_TT_CCC_HANDLE				        0x002Fu
#define UART_TX_HANDLE								 0x00E0u
//#define CCC_DATA_LEN										21u
#define BUTTON_UPDATA_HANDLE					 0x0019u
#define TMP_ML_HANDLE					 				 0x0012u
#define NOW_ML_HANDLE					 				 0x000Eu
//****************************************************************************
// struct 
//****************************************************************************/
struct t_comm_state
{
	uint16 BLERxFinshed         : 1 ;  //Bits 0
	uint16 BLERxRDY             : 1 ;  //Bits 1   
	uint16 UartRxFinished        : 1 ;  //Bits 2
	uint16 UartRxRDY            : 1 ;  //Bits 3   
	uint16 Reserved             : 8 ;  //Bits 8-15    Reserved
};    

typedef union {
 uint16  All;
 struct t_comm_state  Bit;
}u_comm_state;


//****************************************************************************
// Var 
//****************************************************************************/
extern uint8 Buffer[300];
extern u_comm_state uCommState;

//****************************************************************************
// function 
//****************************************************************************/
void CommMonitorUart(void);
void CommMonitorBLE(void);
void CommInit(void);

/* [] END OF FILE */