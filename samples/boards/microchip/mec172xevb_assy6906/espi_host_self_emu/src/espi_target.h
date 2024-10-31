#ifndef __SAMPLES_BOARDS_MICROCHIP_MEC172XEVB_ASSY6906_ESPI_HOST_SELF_EMU_ESPI_TARGET_H_
#define __SAMPLES_BOARDS_MICROCHIP_MEC172XEVB_ASSY6906_ESPI_HOST_SELF_EMU_ESPI_TARGET_H_

#include <stdint.h>
#include <stddef.h>

int espi_target_init(uint32_t flags);
void espi_target_thread_entry(void *p1, void *p2, void *p3);

#endif /* __SAMPLES_BOARDS_MICROCHIP_MEC172XEVB_ASSY6906_ESPI_HOST_SELF_EMU_ESPI_TARGET_H_ */
