/*
 * Copyright (c) 2021 Microchip Technology Inc. All Rights Reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _MICROCHIP_XEC_SOC_DT_H_
#define _MICROCHIP_XEC_SOC_DT_H_

#define MCHP_XEC_NO_PULL	0x0
#define MCHP_XEC_PULL_UP	0x1
#define MCHP_XEC_PULL_DOWN	0x2
#define MCHP_XEC_REPEATER	0x3
#define MCHP_XEC_PUSH_PULL	0x0
#define MCHP_XEC_OPEN_DRAIN	0x1
#define MCHP_XEC_NO_OVAL	0x0
#define MCHP_XEC_OVAL_LOW	0x1
#define MCHP_XEC_OVAL_HIGH	0x2
#define MCHP_XEC_DRVSTR_NONE	0x0
#define MCHP_XEC_DRVSTR_2MA	0x1
#define MCHP_XEC_DRVSTR_4MA	0x2
#define MCHP_XEC_DRVSTR_8MA	0x3
#define MCHP_XEC_DRVSTR_12MA	0x4

/* Helper macros for use with Microchip XEC DMAC controller
 * return 0xff as default value if there is no 'dmas' property
 */
#define MCHP_XEC_DT_INST_DMA_CELL(n, name, cell)			\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, dmas),			\
		    (DT_INST_DMAS_CELL_BY_NAME(n, name, cell)), (0xff))

#define MCHP_XEC_DT_INST_DMA_TRIGSRC(n, name)				\
	MCHP_XEC_DT_INST_DMA_CELL(n, name, trigsrc)

#define MCHP_XEC_DT_INST_DMA_CHANNEL(n, name)				\
	MCHP_XEC_DT_INST_DMA_CELL(n, name, channel)

#define MCHP_XEC_DT_INST_DMA_CTLR(n, name)				\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, dmas),			\
		    (DT_INST_DMAS_CTLR_BY_NAME(n, name)),		\
		    (DT_INVALID_NODE))

#endif /* _MICROCHIP_XEC_SOC_DT_H_ */
