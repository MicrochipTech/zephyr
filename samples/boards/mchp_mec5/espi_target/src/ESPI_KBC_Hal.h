/****************************************************************************
* 2014 Microchip Technology Inc. and its subsidiaries.
* You may use this software and any derivatives exclusively with
* Microchip products.
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".
* NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
* INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
* AND FITNESS FOR A PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.
* TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
* CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF
* FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
* MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE
* OF THESE TERMS.
*/

/******************************************************************************/
/** @defgroup ESPI_PC ESPI_PC
 *  @{
 */

/** @file ESPI_PC.h
* \brief ESPI_PC header file
* \author KBCEC Team
* 
******************************************************************************/
#ifndef ESPI_KBC_Hal_H
#define ESPI_KBC_Hal_H

/* function declarations */
/* init */
typedef unsigned char           BYTE;
typedef unsigned short          WORD;
typedef unsigned long           DWORD;

#define trace0(nbr,cat,b,str) 
#define trace1(nbr,cat,b,str,p1) 
#define trace2(nbr,cat,b,str,p1,p2) 

/* EC cmds */
void ESPI_KBC_Cmd(unsigned char command_num);
void ESPI_Srvc_Kbcdat1(unsigned char value);

#endif /* ESPI_PC_H */

/**   @}
 */
