/*******************************************************************************
* File Name: test_2.h  
* Version 2.10
*
* Description:
*  This file containts Control Register function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_test_2_H) /* Pins test_2_H */
#define CY_PINS_test_2_H

#include "cytypes.h"
#include "cyfitter.h"
#include "test_2_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    test_2_Write(uint8 value) ;
void    test_2_SetDriveMode(uint8 mode) ;
uint8   test_2_ReadDataReg(void) ;
uint8   test_2_Read(void) ;
uint8   test_2_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define test_2_DRIVE_MODE_BITS        (3)
#define test_2_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - test_2_DRIVE_MODE_BITS))

#define test_2_DM_ALG_HIZ         (0x00u)
#define test_2_DM_DIG_HIZ         (0x01u)
#define test_2_DM_RES_UP          (0x02u)
#define test_2_DM_RES_DWN         (0x03u)
#define test_2_DM_OD_LO           (0x04u)
#define test_2_DM_OD_HI           (0x05u)
#define test_2_DM_STRONG          (0x06u)
#define test_2_DM_RES_UPDWN       (0x07u)

/* Digital Port Constants */
#define test_2_MASK               test_2__MASK
#define test_2_SHIFT              test_2__SHIFT
#define test_2_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define test_2_PS                     (* (reg32 *) test_2__PS)
/* Port Configuration */
#define test_2_PC                     (* (reg32 *) test_2__PC)
/* Data Register */
#define test_2_DR                     (* (reg32 *) test_2__DR)
/* Input Buffer Disable Override */
#define test_2_INP_DIS                (* (reg32 *) test_2__PC2)


#if defined(test_2__INTSTAT)  /* Interrupt Registers */

    #define test_2_INTSTAT                (* (reg32 *) test_2__INTSTAT)

#endif /* Interrupt Registers */


/***************************************
* The following code is DEPRECATED and 
* must not be used.
***************************************/

#define test_2_DRIVE_MODE_SHIFT       (0x00u)
#define test_2_DRIVE_MODE_MASK        (0x07u << test_2_DRIVE_MODE_SHIFT)


#endif /* End Pins test_2_H */


/* [] END OF FILE */
