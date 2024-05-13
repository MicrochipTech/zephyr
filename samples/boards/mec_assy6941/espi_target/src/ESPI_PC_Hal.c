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

/*
 **********************************************************************************
 *  ESPI_PC_Hal.C
 *      This is the ESPI service file: Peripheral channel process
 **********************************************************************************
 */

/******************************************************************************/
/** @defgroup ESPI_PC ESPI_PC
 *  @{
 */

/** @file ESPI_PC.c
* \brief ESPI_PC source file
* \author KBCEC Team
*
******************************************************************************/

/* for global */
#include <zephyr/kernel.h>
#include <soc.h>
//#include <soc_espi.h>
#include <errno.h>
#include <zephyr/arch/common/ffs.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/espi/mchp-mec5-espi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>
/* TODO #include "espi_utils.h" */
//#include "espi_mchp_mec5_private.h"

/* MEC5 HAL */
#include <device_mec5.h>
#include <mec_retval.h>
#include <mec_pcr_api.h>
#include <mec_espi_api.h>
#include <mec_acpi_ec_api.h>

//LOG_MODULE_REGISTER(espi_acpi_ec, CONFIG_ESPI_LOG_LEVEL);

/* get ACPI EC0 register struct from device tree */
#define ACPI_EC0_NODE DT_NODELABEL(acpi_ec0)
struct acpi_ec_regs *const acpi_ec0_regs = (struct acpi_ec_regs *)DT_REG_ADDR(ACPI_EC0_NODE);

/* get ESPI device - only one instance */
static const struct device *const espi_dev = DEVICE_DT_GET(DT_NODELABEL(espi0));

/* for local */
#include "ESPI_PC_Hal.h"

#define ON  1
#define OFF 0
/* ON: support KabyLake platform; OFF: Skylake platform */
#define KBL_SUPPORTED	        ON
#define ESPI_OOB_FC_TEST		OFF

/* global variables */
unsigned char Cmd_Byte2 = 0;
unsigned char Tmp_Load2 = 0;    /* Increment value byte counter. */
unsigned char EC_Addr;         	/* Store address byte. */
unsigned char Ext_Cb3_ACPI_ENABLED = 0;
unsigned char espi_acpi_ec_space[256] = {
    // 0     1     2     3     4     5     6     7     8     9     A     B     C     D     E     F
    0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 00h-0Fh
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 10h-1Fh
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 20h-2Fh
    0x99, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 30h-3Fh
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 40h-4Fh
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x62, 0x11, 0x9F, 0x0E, 0x96, 0x11, 0x8B, 0x1F, 0x00, // 50h-5Fh
    0x00, 0xA3, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 60h-6Fh
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 70h-7Fh
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 80h-8Fh
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 90h-9Fh
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // A0h-AFh
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // B0h-BFh
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // C0h-CFh
    0x00, 0x00, 0x00, 0xB0, 0x1D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // D0h-DFh
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // E0h-EFh
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x07, 0x00, 0x00, 0x00, 0x00  // F0h-FFh
};


/* Generate SCI to host*/
void ACPI_Gen_Int(void)
{
	/* check if system is in acpi mode */
	if(Ext_Cb3_ACPI_ENABLED == 1)
	{
		espi_send_vwire(espi_dev, ESPI_VWIRE_SIGNAL_SCI, 0);
		k_busy_wait(100);
		espi_send_vwire(espi_dev, ESPI_VWIRE_SIGNAL_SCI, 1);
	}
}

/* tx data to EC0 */
void Data_To_Host2(unsigned char value)
{
	//mec_acpi_ec_host_to_ec_data_wr8(acpi_ec0_regs, 0x00, value);
	mec_acpi_ec_e2h_data_wr8(acpi_ec0_regs, 0x00, value);
}

unsigned char Output_Buffer2_Full(void)
{
	unsigned char ret;
	ret = mec_acpi_ec_status_obf(acpi_ec0_regs);
	return ret;
}

/******************************************************************************/
/** ESPI_ACPI_Cmd
 * Process ACPI commands...
 *
 * @param None
 *
 * @return None
 *
 ******************************************************************************/
void ESPI_ACPI_Cmd(BYTE command_num)
{
	BYTE value_tmp;
    unsigned short value2;

	trace1(0, ESPI_PC, 0, "ESPI_ACPI_Cmd: command_num  = %02Xh", command_num);
	switch(command_num)
	{
		case 0x04:     
			break;

		case 0x05:     
			break;

		case 0x06:                
	        while(Output_Buffer2_Full())
			{
			}
			/* Query system status */
			/* bit5=0, undocked; bit4=1, AC powered; bit[3:0]=0, CPU thermal state */
			value_tmp = 0x10;
            Data_To_Host2(value_tmp);
			//ACPI_Gen_Int();
			break;

		case 0x09:                
	        while(Output_Buffer2_Full())
			{
			}
			/* 1st byte: "K" */
            Data_To_Host2(0x4b);
			//ACPI_Gen_Int();
	        trace1(0, ESPI_PC, 0, "ESPI_ACPI_Cmd: TGL 0x09 return K = %02Xh", 0x4B);

	        while(Output_Buffer2_Full())
			{
			}
			/* 2nd byte: "S" */
            Data_To_Host2(0x53);
			//ACPI_Gen_Int();
	        trace1(0, ESPI_PC, 0, "ESPI_ACPI_Cmd: TGL 0x09 return S = %02Xh", 0x53);

	        while(Output_Buffer2_Full())
			{
			}
			/* 3rd byte: "C" */
            Data_To_Host2(0x43);
			//ACPI_Gen_Int();
	        trace1(0, ESPI_PC, 0, "ESPI_ACPI_Cmd: TGL 0x09 return C = %02Xh", 0x43);

	        while(Output_Buffer2_Full())
			{
			}
#if 1
/* TGL porting */

			/* 4th byte: 0 */
            Data_To_Host2(0x94);
#else
            Data_To_Host2(0x00);
#endif
			//ACPI_Gen_Int();
			/* case: 0x09: */
	        trace1(0, ESPI_PC, 0, "ESPI_ACPI_Cmd: TGL 0x09 return 4th Byte = %02Xh", 0x94);
			break;

		case 0x0A:                
	        while(Output_Buffer2_Full())
			{
			}
			/* Get switch status */
			/* bit5=0, undocked; bit4=1, AC powered; bit2=0, NMI jumpr is ON; bit1=0, virtual batt status; bit0=0, lid switch is open */
			value_tmp = 0x10;
            Data_To_Host2(value_tmp);
			//ACPI_Gen_Int();
			break;

		case 0x0D:                
	        while(Output_Buffer2_Full())
			{
			}
#if 1
/* TGL porting */

            Data_To_Host2(0xF8);
            //Data_To_Host2(0x01);
	        trace1(0, ESPI_PC, 0, "ESPI_ACPI_Cmd: TGL 0x0D return FAB ID = %02Xh", 0xF8);
#else
#if (KBL_SUPPORTED == ON)
/* ON: support KabyLake platform */

            Data_To_Host2(0xC0);
#else
/* Skylake platform */

            Data_To_Host2(0x02);
#endif
#endif
			//ACPI_Gen_Int();

	        while(Output_Buffer2_Full())
			{
			}
#if 1
/* TGL porting */

            Data_To_Host2(0x01);
	        trace1(0, ESPI_PC, 0, "ESPI_ACPI_Cmd: TGL 0x0D return BRD ID = %02Xh", 0x01);
#else
            Data_To_Host2(0x04);
#endif
			//ACPI_Gen_Int();
			break;

		case 0x0E:     
			/* have one parameter */
			Cmd_Byte2 = command_num;  /* Signal to wait for value from Host. */
			break;

#if 1
/* TGL porting */

		case 0x13:     
			/* have one parameter */
			Cmd_Byte2 = command_num;  /* Signal to wait for value from Host. */
			break;
#endif

#if (KBL_SUPPORTED == ON)
/* ON: support KabyLake platform */

		case 0x1A:      
            /* PWMUpdate: PWM value is in EC space */
	        /* need to do here to process PWM update */
	        trace1(0, ESPI_PC, 0, "ESPI_ACPI_Cmd: KBL 0x1A PWMUpdate = %02Xh", command_num);
			break;

		case 0x1B:      
            /* get PMIC vendor ID */
	        while(Output_Buffer2_Full())
			{
			}
            Data_To_Host2(0x1F);
	        trace1(0, ESPI_PC, 0, "ESPI_ACPI_Cmd: KBL 0x1B return PMIC Vendor ID to host = %02Xh", 0x1F);
			break;

		case 0x1C:     
			/* have one parameter */
			Cmd_Byte2 = command_num;  /* Signal to wait for value from Host. */
	        trace1(0, ESPI_PC, 0, "ESPI_ACPI_Cmd: KBL 0x1C, a parameter from host, Cmd_Byte2 = %02Xh", Cmd_Byte2);
			break;

		case 0xB0:               
            /* BIOS guard cmd */
	        while(Output_Buffer2_Full())
			{
			}
            Data_To_Host2(0x00);
	        trace1(0, ESPI_PC, 0, "ESPI_ACPI_Cmd: KBL 0xB0 return (can be disabled from BIOS menu) = %02Xh", 0x00);
			break;
#endif

		case 0x23:     
			break;

		case 0x27:     
			/* have one parameter */
			Cmd_Byte2 = command_num;  /* Signal to wait for value from Host. */
			break;

		case 0x29:     
			/* have one parameter */
			Cmd_Byte2 = command_num;  /* Signal to wait for value from Host. */
			break;

		case 0x2D:     
#if 0
/* TGL porting */

	        while(Output_Buffer2_Full())
			{
			}
			/* 1st byte: "K" */
            Data_To_Host2(0x4b);
			//ACPI_Gen_Int();
	        trace1(0, ESPI_PC, 0, "ESPI_ACPI_Cmd: TGL 0x2D return K = %02Xh", 0x4B);

	        while(Output_Buffer2_Full())
			{
			}
			/* 2nd byte: "S" */
            Data_To_Host2(0x53);
			//ACPI_Gen_Int();
	        trace1(0, ESPI_PC, 0, "ESPI_ACPI_Cmd: TGL 0x2D return S = %02Xh", 0x53);

	        while(Output_Buffer2_Full())
			{
			}
			/* 3rd byte: "C" */
            Data_To_Host2(0x43);
			//ACPI_Gen_Int();
	        trace1(0, ESPI_PC, 0, "ESPI_ACPI_Cmd: TGL 0x2D return C = %02Xh", 0x43);

	        while(Output_Buffer2_Full())
			{
			}
			/* 4th byte: 0 */
            Data_To_Host2(0x94);
			//ACPI_Gen_Int();
	        trace1(0, ESPI_PC, 0, "ESPI_ACPI_Cmd: TGL 0x2D return 4th Byte = %02Xh", 0x94);
			/* case: 0x2D: */
#endif
			break;

		case 0x2E:     
			break;

		case 0x2F:     
			break;

#if 1
/* TGL porting */

		case 0x39:     
			/* have one parameter */
			Cmd_Byte2 = command_num;  /* Signal to wait for value from Host. */
			break;
#endif

		case 0x46:     
			break;

		case 0x4B:     
			break;

		case 0x4D:     
			/* have one parameter */
			Cmd_Byte2 = command_num;  /* Signal to wait for value from Host. */
			break;

		case 0x58:     
			/* have one parameter */
			Cmd_Byte2 = command_num;  /* Signal to wait for value from Host. */
			break;

		case 0x64:     
			break;

		case 0x70:     
			/* SMI Query */
//			value2 = Pc_Cmd8(0x92);
			value2 = 0;
			trace1(0, ESPI_PC, 0, "ESPI_ACPI_Cmd: SMI cause code  = %04Xh", value2);
			if(value2 != (WORD) ~0)
			{
				while(Output_Buffer2_Full())
				{
				}
				/* send SMI cause code to host */
				Data_To_Host2((BYTE)value2);
			}
			break;

#if 1
/* processed in Insyde ACPI cmd/data */
		case 0x80:     
			/* Generate interrupt to signal the Host that the
            command has been taken and more value can be sent. */
			ACPI_Gen_Int();

			/* have one parameter */
			Cmd_Byte2 = command_num;  /* Signal to wait for value from Host. */
			break;

		case 0x81:     
			/* Generate interrupt to signal the Host that the
            command has been taken and more value can be sent. */
			ACPI_Gen_Int();

			/* have 2 parameter2 */
			Cmd_Byte2 = command_num;  /* Signal to wait for value from Host. */
			break;

		case 0x82:     
			/* Set Burst bit in secondary Host interface status register. */
//			Write_Host_Status_Reg2(Read_Host_Status_Reg2() |
//								   maskSTATUS_PORT2_BURST);
			while(Output_Buffer2_Full())
			{
			}
			/* enable burst mode */
			value_tmp = 0x90;
            Data_To_Host2(value_tmp);
			ACPI_Gen_Int();
			break;

		case 0x83:     
			/* Clear Burst bit in secondary Host interface status register. */
//			Write_Host_Status_Reg2(Read_Host_Status_Reg2() &
//                                   ~maskSTATUS_PORT2_BURST);
			/* disable burst mode */
			ACPI_Gen_Int();
			break;
#endif

		case 0x8A:     
	        while(Output_Buffer2_Full())
			{
			}
			/* Get switch status - dock status, =1, docked; =0, undocked */
			value_tmp = 0x00;
            Data_To_Host2(value_tmp);
	        trace1(0, ESPI_PC, 0, "ESPI_ACPI_Cmd: TGL 0x8A return SW sts - undocked = %02Xh", value_tmp);
			//ACPI_Gen_Int();
			break;
			
		case 0x90:                
	        while(Output_Buffer2_Full())
			{
			}
#if 1
/* TGL porting */

			/* 1st byte: FW revision - major */
            Data_To_Host2(0x01);
#else
            Data_To_Host2(BUILDNUMBER >> 8);
#endif
	        trace1(0, ESPI_PC, 0, "ESPI_ACPI_Cmd: TGL 0x90 return FW Rev major = %02Xh", 0x01);
			//ACPI_Gen_Int();

	        while(Output_Buffer2_Full())
			{
			}
			/* 2nd byte: FW revision - minor */
#if 1
/* TGL porting */

            Data_To_Host2(0x10);
#else
            Data_To_Host2(BUILDNUMBER & 0x00FF);
#endif
	        trace1(0, ESPI_PC, 0, "ESPI_ACPI_Cmd: TGL 0x90 return FW Rev minor = %02Xh", 0x10);
			//ACPI_Gen_Int();
			/* case: 0x90: */
			break;

		case 0xAA:     
			/* enable ACPI mode */
			Ext_Cb3_ACPI_ENABLED = 1;
			break;

		case 0xAB:     
			/* disable ACPI mode, back to APM mode */
			Ext_Cb3_ACPI_ENABLED = 0;
			break;

		case 0xBC:     
			/* disable SMIs */
			break;
			
 		case 0xBD:     
			/* enable SMIs */
			break;

#if 1
/* TGL porting */

		case 0xDF:     
			/* have one parameter */
			Cmd_Byte2 = command_num;  /* Signal to wait for value from Host. */
			break;

		case 0xE1:     
			break;

		case 0xF3:     
			/* have one parameter */
			Cmd_Byte2 = command_num;  /* Signal to wait for value from Host. */
			break;
#endif

#if (ESPI_OOB_FC_TEST == ON)
/* Support MEC14xx ESPI OOB & FC TX & RX */

		case 0xF1:     
			trace0(0, ESPI_PC, 0, "ESPI_ACPI_Cmd: rx MCHP PE command 0xF1.");
			/* write memory: have 9 parameters */
			Cmd_Byte2 = command_num;  /* Signal to wait for value from Host. */
			break;

		case 0xF2:     
			trace0(0, ESPI_PC, 0, "ESPI_ACPI_Cmd: rx MCHP PE command 0xF2.");
			/* read memory: have 5 parameters */
			Cmd_Byte2 = command_num;  /* Signal to wait for value from Host. */
			break;
#endif

       default:
			trace1(0, ESPI_PC, 0, "ESPI_ACPI_Cmd: oop??? unsupported cmd, command_num  = %02Xh", command_num);
			//while(1);
            break;
	} /* switch(command_num) */
}

/******************************************************************************/
/** ESPI_Srvc_Pcdat2
 * Process ACPI commands with parameters
 *
 * @param None
 *
 * @return None
 *
 ******************************************************************************/
void ESPI_Srvc_Pcdat2(BYTE value)
{
	BYTE value_tmp;

	trace1(0, ESPI_PC, 0, "ESPI_Srvc_Pcdat2: command: Cmd_Byte2 = %02Xh", Cmd_Byte2);
	trace1(0, ESPI_PC, 0, "ESPI_Srvc_Pcdat2: data: value = %02Xh", value);
	switch(Cmd_Byte2)
	{
		case 0x0E:     
			/* value is to select AON or ALS */
			/* need to add app code here... */
			value_tmp = value;
			Cmd_Byte2 = 0;
			break;

#if 1
/* TGL porting */

		case 0x13:     
			/* value is the set deep sleep mode */
			/* need to add app code here... */
			value_tmp = value;
	        trace1(0, ESPI_PC, 0, "ESPI_Srvc_Pcdat2: TGL CMD 0x13 data: value_tmp = %02Xh", value_tmp);
			Cmd_Byte2 = 0;
			break;
#endif

#if (KBL_SUPPORTED == ON)
/* ON: support KabyLake platform */

		case 0x1C:     
			/* value is ??? not in spec. */
			/* need to add app code here... */
			value_tmp = value;
	        trace1(0, ESPI_PC, 0, "ESPI_ACPI_Cmd: KBL 0x1C's parameter from host = %02Xh", value_tmp);
			Cmd_Byte2 = 0;
			break;
#endif

		case 0x27:     
			/* value is to enable low power mode: =1, enable; =0, disable */
			/* need to add app code here... */
			value_tmp = value;
			Cmd_Byte2 = 0;
			break;

		case 0x29:     
			/* value is the set deep sleep mode */
			/* need to add app code here... */
			value_tmp = value;
			Cmd_Byte2 = 0;
			break;

#if 1
/* TGL porting */

		case 0x39:     
			/* value is the set deep sleep mode */
			/* need to add app code here... */
			value_tmp = value;
	        trace1(0, ESPI_PC, 0, "ESPI_Srvc_Pcdat2: TGL CMD 0x39 data: value_tmp = %02Xh", value_tmp);
			Cmd_Byte2 = 0;
			break;
#endif

		case 0x58:     
			/* value is the vritical temperature threshold */
			/* need to add app code here... */
			value_tmp = value;
			Cmd_Byte2 = 0;
			break;

		case 0x4D:     
			/* value is the set PECI injected temperature */
			/* need to add app code here... */
			value_tmp = value;
			Cmd_Byte2 = 0;
			break;

#if 1
/* processed in Insyde ACPI cmd/data */
		case 0x80:     // read
			/* Generate interrupt to signal the Host that the
            data has been taken and more value can be sent. */
			ACPI_Gen_Int();
			/* value is the ACPI space index */
			value_tmp = espi_acpi_ec_space[value];
	        while(Output_Buffer2_Full())
			{
			}
            Data_To_Host2(value_tmp);
			ACPI_Gen_Int();
			Cmd_Byte2 = 0;
			break;

		case 0x81:     // wr
			/* Generate interrupt to signal the Host that the
            data has been taken and more value can be sent. */
			ACPI_Gen_Int();
			/* process 2 parameters */
			if(Tmp_Load2 == 0) 
			{
				/* Just received 1st address byte (index) from Host. */
				Tmp_Load2++;             /* Increment value byte counter. */
				EC_Addr = value;         /* Store address byte. */
			}
			else if(Tmp_Load2 == 1) 
			{
				/* Just received 2nd data byte from Host. */
				/* need to add app code here... */
				espi_acpi_ec_space[EC_Addr] = value;     
				Cmd_Byte2 = 0;
				Tmp_Load2 = 0;
			}
			break;
#endif

#if (ESPI_OOB_FC_TEST == ON)
/* Support MEC14xx ESPI OOB & FC TX & RX */

		case 0xF1:     
			trace0(0, ESPI_PC, 0, "ESPI_Srvc_Pcdat2: rx MCHP PE command 0xF1's data.");
			/* write memory: have 9 parameters */
			ESPI_write_mem_0xF1(value);
			break;

		case 0xF2:     
			trace0(0, ESPI_PC, 0, "ESPI_Srvc_Pcdat2: rx MCHP PE command 0xF2's data.");
			/* read memory: have 5 parameters */
			ESPI_read_mem_0xF2(value);
			break;
#endif

#if 1
/* TGL porting */

		case 0xDF:     
			/* function is ... */
			/* need to add app code here... */
			value_tmp = value;
	        trace1(0, ESPI_PC, 0, "ESPI_Srvc_Pcdat2: TGL CMD 0xDF data: value_tmp = %02Xh", value_tmp);
			Cmd_Byte2 = 0;
			break;

		case 0xF3:     
			/* function is ... */
			/* need to add app code here... */
			value_tmp = value;
	        trace1(0, ESPI_PC, 0, "ESPI_Srvc_Pcdat2: TGL CMD 0xF3 data: value_tmp = %02Xh", value_tmp);
			Cmd_Byte2 = 0;
			break;
#endif

        default:
			trace1(0, ESPI_PC, 0, "ESPI_Srvc_Pcdat2: oop??? unsupported cmd with parameter, Cmd_Byte2  = %02Xh", Cmd_Byte2);
			//LOG_INF("Dat2, unsupported Cmd_Byte2=0x%02x", Cmd_Byte2);
			//while(1);
            break;
	} /* switch(Cmd_Byte2) */
}

/**   @}
 */
