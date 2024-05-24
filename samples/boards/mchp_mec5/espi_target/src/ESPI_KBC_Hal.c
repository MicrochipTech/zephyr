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
#include <mec_kbc_api.h>

//LOG_MODULE_REGISTER(espi_acpi_ec, CONFIG_ESPI_LOG_LEVEL);

/* get KBC0 / 8042 register struct from device tree */
#define KBC0_8042_NODE DT_NODELABEL(kbc0)
//struct kbc_regs *const kbc0_8042_regs = (struct kbc_regs *)DT_REG_ADDR(KBC0_8042_NODE);
struct mec_kbc_regs *const kbc0_8042_regs = (struct mec_kbc_regs *)DT_REG_ADDR(KBC0_8042_NODE);

/* for local */
#include "ESPI_KBC_Hal.h"

/* global variables */
unsigned char Cmd_Byte = 0;
unsigned char Cmd_D4h_AUX_Cmd = 0;
unsigned char AUX_F3h_dat = 0;
unsigned char AUX_E8h_dat = 0;

unsigned char Cmd_KBD = 0;
unsigned char Ccb42_byte = 0;
unsigned char KBD_EDh_dat = 0;

/* support kbc / 8042 cmd & data process */
/* tx KBC/KBD data to KBC */
void Data_To_Host(unsigned char value)
{
	unsigned char sts_byte;
	/* set status properly */
//	sts_byte = mec_kbc_status(kbc0_8042_regs);
	sts_byte = mec_hal_kbc_status(kbc0_8042_regs);
	sts_byte &=  ~(MEC_KBC_STS_UD2_B1  | /* Parity error bits. */
                   MEC_KBC_STS_UD2_B0  | /* General time out error bits. */
                   MEC_KBC_STS_AUXOBF  | /* Send time out error bits for AT. */
                   MEC_KBC_STS_AUXOBF);  /* Aux device bits for PS/2. */
//	mec_kbc_status_wr(kbc0_8042_regs, sts_byte, 0x00);
	mec_hal_kbc_status_wr(kbc0_8042_regs, sts_byte, 0x00);

//	mec_kbc_wr_data(kbc0_8042_regs, value, MEC_KBC_DATA_KB);
	mec_hal_kbc_wr_data(kbc0_8042_regs, value, MEC_KBC_DATA_KB);
}

/* tx AUX data to KBC */
void Data_To_Host_AUX(unsigned char value)
{
	unsigned char sts_byte;
	/* set status properly */
//	sts_byte = mec_kbc_status(kbc0_8042_regs);
	sts_byte = mec_hal_kbc_status(kbc0_8042_regs);
	sts_byte = (sts_byte & ~(MEC_KBC_STS_UD2_B1  | /* Parity error bits. */
                   			MEC_KBC_STS_UD2_B0   | /* General time out error bits. */
                   			MEC_KBC_STS_AUXOBF))   /* Send time out error bits for AT. */
                   			| MEC_KBC_STS_AUXOBF;  /* Aux device bits for PS/2. */
//	mec_kbc_status_wr(kbc0_8042_regs, sts_byte, 0x00);
	mec_hal_kbc_status_wr(kbc0_8042_regs, sts_byte, 0x00);

//	mec_kbc_wr_data(kbc0_8042_regs, value, MEC_KBC_DATA_AUX);
	mec_hal_kbc_wr_data(kbc0_8042_regs, value, MEC_KBC_DATA_AUX);
}

unsigned char Output_Buffer_Full(void)
{
	unsigned char ret;
//	ret = mec_kbc_status(kbc0_8042_regs) & MEC_KBC_STS_OBF;
	ret = mec_hal_kbc_status(kbc0_8042_regs) & MEC_KBC_STS_OBF;
	return ret;
}

/******************************************************************************/
/** ESPI_Srvc_Kbcdat1
 * Process KBC commands...
 *
 * @param None
 *
 * @return None
 *
 ******************************************************************************/
void ESPI_Srvc_Kbcdat1(unsigned char value)
{
	unsigned char value_tmp;

	if(Cmd_Byte == 0)
	{	/* this is KBD cmd, host writes 60h / data register directly */

		if(Cmd_KBD == 0)
		{	/* first kbd cmd */
			switch(value)
			{
				case 0xED:
					/* AUX cmd - ack FAh to host */
	        		while(Output_Buffer_Full())
					{
					}
					/* ACK */
					value_tmp = 0xFA;
            		Data_To_Host(value_tmp);
					Cmd_KBD = 0xED;
					break;


      			default:
			    	break;
			} /* switch(Cmd_Byte) */
		}
		else
		{
			switch(Cmd_KBD)
			{
				case 0xED:
					KBD_EDh_dat = value;
					/* AUX cmd - ack FAh to host */
	        		while(Output_Buffer_Full())
					{
					}
					/* ACK */
					value_tmp = 0xFA;
            		Data_To_Host(value_tmp);
					Cmd_KBD = 0;
					break;

      			default:
			    	break;
			} /* switch(Cmd_Byte) */
		}

	}
	else
	{
		switch(Cmd_Byte)
		{
			case 0xD4:
				/* 0xD4 - following cmd or data to AUX device */
				if(Cmd_D4h_AUX_Cmd == 0)
				{
				switch(value)
				{
					case 0xE8:
						/* AUX cmd - ack FAh to host */
	        			while(Output_Buffer_Full())
						{
						}
						/* ACK */
            			Data_To_Host_AUX(0xFA);
						Cmd_Byte = 0;
						Cmd_D4h_AUX_Cmd = 0xE8;
						break;

					case 0xF3:
						/* AUX cmd - ack FAh to host */
	        			while(Output_Buffer_Full())
						{
						}
						/* ACK */
            			Data_To_Host_AUX(0xFA);
						Cmd_Byte = 0;
						Cmd_D4h_AUX_Cmd = 0xF3;
						break;

					case 0xE6:
						/* AUX cmd - ack FAh to host */
	        			while(Output_Buffer_Full())
						{
						}
						/* ACK */
						//value_tmp = 0xFA;
            			Data_To_Host_AUX(0xFA);
						Cmd_Byte = 0;
						break;

					case 0xF4:
						/* AUX cmd - ack FAh to host */
	        			while(Output_Buffer_Full())
						{
						}
						/* ACK */
						value_tmp = 0xFA;
            			Data_To_Host_AUX(value_tmp);
						Cmd_Byte = 0;
						break;

					case 0xFF:
						/* AUX cmd - reset AUX */
	        			while(Output_Buffer_Full())
						{
						}
						/* ACK */
						//value_tmp = 0xFA;
            			Data_To_Host_AUX(0xFA);

	        			while(Output_Buffer_Full())
						{
						}
						/* ACK */
            			Data_To_Host_AUX(0xAA);

	        			while(Output_Buffer_Full())
						{
						}
						/* ACK */
            			Data_To_Host_AUX(0x00);

						Cmd_Byte = 0;
						break;

					default:
						break;
				} /* switch(value) */
				}
				else
				{	/* has previous AUX cmd */
					switch(Cmd_D4h_AUX_Cmd)
					{
						case 0xE8:
							/* AUX cmd - ack FAh to host */
	        				while(Output_Buffer_Full())
							{
							}
							/* ACK */
							//value_tmp = 0xFA;
            				Data_To_Host_AUX(0xFA);

							/* grab data of E8h */
							AUX_E8h_dat = value;
							Cmd_Byte = 0;
							Cmd_D4h_AUX_Cmd = 0;
							break;

						case 0xF3:
							/* AUX cmd - ack FAh to host */
	        				while(Output_Buffer_Full())
							{
							}
							/* ACK */
							value_tmp = 0xFA;
            				Data_To_Host_AUX(value_tmp);

							/* grab data of F3h */
							AUX_F3h_dat = value;
							Cmd_Byte = 0;
							Cmd_D4h_AUX_Cmd = 0;
							break;

						default:
							break;
					}
				}
				break;

			case 0x60:
				/* 0x60 - following one data */
				Ccb42_byte = value;
				Cmd_Byte = 0;
				break;


      		default:
			    break;
		} /* switch(Cmd_Byte) */
	}
}


/******************************************************************************/
/** ESPI_KBC_Cmd
 * Process KBC commands...
 *
 * @param None
 *
 * @return None
 *
 ******************************************************************************/
void ESPI_KBC_Cmd(unsigned char command_num)
{
	unsigned char value_tmp;

	switch(command_num)
	{
		case 0xAA:
	        while(Output_Buffer_Full())
			{
			}
			/* self test */
			value_tmp = 0x55;
            Data_To_Host(value_tmp);
			break;

		case 0xD4:
			/* 0xD4 - following cmd or data to AUX device */
			/* will rx one data byte */
			Cmd_Byte = command_num;   /* Save command number and wait for value byte. */
			break;

		case 0x60:
			/* 0x60 - following one data */
			Cmd_Byte = command_num;   /* Save command number and wait for value byte. */
			break;

		case 0x20:
	        while(Output_Buffer_Full())
			{
			}
			/* read */
            Data_To_Host(Ccb42_byte);
			break;

		case 0xAD:     
			break;

		case 0xA7:     
			break;

		case 0xA8:     
			break;

      	default:
			trace1(0, ESPI_PC, 0, "ESPI_KBC_Cmd: oop??? unsupported cmd, command_num  = %02Xh", command_num);
            break;
	} /* switch(command_num) */
}

/**   @}
 */
