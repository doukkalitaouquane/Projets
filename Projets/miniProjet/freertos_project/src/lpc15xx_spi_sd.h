/**************************************************************************//**
 * @file     lpc15xx_spi_sd.h
 * @brief    Header file for lpc15xx_spi_sd.c.
 * @version  1.0
 * @date     28. Nov. 2015
 *
 *
 ******************************************************************************/

#ifndef __LPC17xx_SPI_H
#define __LPC17xx_SPI_H

#include "board.h"                 


/*In SPI mode, max clock speed is 20MHz for MMC and 25MHz for SD */
#define SPI_CLOCKRATE_LOW   (uint32_t) (400000)   /* 400kHz */
#define SPI_CLOCKRATE_HIGH  (uint32_t) (25000000)     /*  25MHz */

/* Public functions */
void    SPI_Init (void);
void    SPI_ConfigClockRate (uint32_t SPI_CLOCKRATE);
void    SPI_CS_Low (void);
void    SPI_CS_High (void);
uint8_t SPI_SendByte (uint8_t data);
uint8_t SPI_RecvByte (void);

#endif  // __LPC17xx_SPI_H

/* --------------------------------- End Of File ------------------------------ */

