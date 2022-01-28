/**
 * \file crc8.h
 * Functions and types for CRC checks.
 *
 * Generated on Mon Apr  2 17:26:19 2012,
 * by pycrc v0.7.10, http://www.tty1.net/pycrc/
 * using the configuration:
 *    Width        = 8
 *    Poly         = 0x07
 *    XorIn        = 0x00
 *    ReflectIn    = False
 *    XorOut       = 0x55
 *    ReflectOut   = False
 *    Algorithm    = table-driven
 *****************************************************************************/
#ifndef __CRC8_H__
#define __CRC8_H__

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * The definition of the used algorithm.
 *****************************************************************************/
#define CRC_ALGO_TABLE_DRIVEN 1


/**
 * The type of the CRC values.
 *
 * This type must be big enough to contain at least 8 bits.
 *****************************************************************************/
typedef uint8_t crc8_t;


/**
 * Calculate the initial crc value.
 *
 * \return     The initial crc value.
 *****************************************************************************/
static inline crc8_t crc8_init(void)
{
	return 0x00;
}


/**
 * Update the crc value with new data.
 *
 * \param crc      The current crc value.
 * \param data     Pointer to a buffer of \a data_len bytes.
 * \param data_len Number of bytes in the \a data buffer.
 * \return         The updated crc value.
 *****************************************************************************/
crc8_t crc8_update(crc8_t crc, const unsigned char *data, size_t data_len);


/**
 * Calculate the final crc value.
 *
 * \param crc  The current crc value.
 * \return     The final crc value.
 *****************************************************************************/
static inline crc8_t crc8_finalize(crc8_t crc)
{
	return crc ^ 0x55;
}


#ifdef __cplusplus
}
#endif

#endif      /* __CRC8_H__ */
/* end crc8.h */
