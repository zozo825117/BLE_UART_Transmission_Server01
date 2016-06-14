/*******************************************************************************
* File Name: Uart_LED.c  
* Version 2.10
*
* Description:
*  This file contains API to enable firmware control of a Pins component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "Uart_LED.h"

#define SetP4PinDriveMode(shift, mode)  \
    do { \
        Uart_LED_PC =   (Uart_LED_PC & \
                                (uint32)(~(uint32)(Uart_LED_DRIVE_MODE_IND_MASK << (Uart_LED_DRIVE_MODE_BITS * (shift))))) | \
                                (uint32)((uint32)(mode) << (Uart_LED_DRIVE_MODE_BITS * (shift))); \
    } while (0)


/*******************************************************************************
* Function Name: Uart_LED_Write
********************************************************************************
*
* Summary:
*  Assign a new value to the digital port's data output register.  
*
* Parameters:  
*  prtValue:  The value to be assigned to the Digital Port. 
*
* Return: 
*  None 
*  
*******************************************************************************/
void Uart_LED_Write(uint8 value) 
{
    uint8 drVal = (uint8)(Uart_LED_DR & (uint8)(~Uart_LED_MASK));
    drVal = (drVal | ((uint8)(value << Uart_LED_SHIFT) & Uart_LED_MASK));
    Uart_LED_DR = (uint32)drVal;
}


/*******************************************************************************
* Function Name: Uart_LED_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  Uart_LED_DM_STRONG     Strong Drive 
*  Uart_LED_DM_OD_HI      Open Drain, Drives High 
*  Uart_LED_DM_OD_LO      Open Drain, Drives Low 
*  Uart_LED_DM_RES_UP     Resistive Pull Up 
*  Uart_LED_DM_RES_DWN    Resistive Pull Down 
*  Uart_LED_DM_RES_UPDWN  Resistive Pull Up/Down 
*  Uart_LED_DM_DIG_HIZ    High Impedance Digital 
*  Uart_LED_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void Uart_LED_SetDriveMode(uint8 mode) 
{
	SetP4PinDriveMode(Uart_LED__0__SHIFT, mode);
}


/*******************************************************************************
* Function Name: Uart_LED_Read
********************************************************************************
*
* Summary:
*  Read the current value on the pins of the Digital Port in right justified 
*  form.
*
* Parameters:  
*  None 
*
* Return: 
*  Returns the current value of the Digital Port as a right justified number
*  
* Note:
*  Macro Uart_LED_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 Uart_LED_Read(void) 
{
    return (uint8)((Uart_LED_PS & Uart_LED_MASK) >> Uart_LED_SHIFT);
}


/*******************************************************************************
* Function Name: Uart_LED_ReadDataReg
********************************************************************************
*
* Summary:
*  Read the current value assigned to a Digital Port's data output register
*
* Parameters:  
*  None 
*
* Return: 
*  Returns the current value assigned to the Digital Port's data output register
*  
*******************************************************************************/
uint8 Uart_LED_ReadDataReg(void) 
{
    return (uint8)((Uart_LED_DR & Uart_LED_MASK) >> Uart_LED_SHIFT);
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(Uart_LED_INTSTAT) 

    /*******************************************************************************
    * Function Name: Uart_LED_ClearInterrupt
    ********************************************************************************
    *
    * Summary:
    *  Clears any active interrupts attached to port and returns the value of the 
    *  interrupt status register.
    *
    * Parameters:  
    *  None 
    *
    * Return: 
    *  Returns the value of the interrupt status register
    *  
    *******************************************************************************/
    uint8 Uart_LED_ClearInterrupt(void) 
    {
		uint8 maskedStatus = (uint8)(Uart_LED_INTSTAT & Uart_LED_MASK);
		Uart_LED_INTSTAT = maskedStatus;
        return maskedStatus >> Uart_LED_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */
