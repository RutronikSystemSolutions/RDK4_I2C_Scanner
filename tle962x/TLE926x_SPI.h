/*********************************************************************************************************************
 * Copyright (c) 2021, Infineon Technologies AG
 *
 *
 * Distributed under the Boost Software License, Version 1.0.
 *
 *
 * Boost Software License - Version 1.0 - August 17th, 2003
 *
 * Permission is hereby granted, free of charge, to any person or organization
 * obtaining a copy of the software and accompanying documentation covered by
 * this license (the "Software") to use, reproduce, display, distribute,
 * execute, and transmit the Software, and to prepare derivative works of the
 * Software, and to permit third-parties to whom the Software is furnished to
 * do so, all subject to the following:
 *
 * The copyright notices in the Software and this entire statement, including
 * the above license grant, this restriction and the following disclaimer,
 * must be included in all copies of the Software, in whole or in part, and
 * all derivative works of the Software, unless such copies or derivative
 * works are solely in the form of machine-executable object code generated by
 * a source language processor.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
 * FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 *********************************************************************************************************************/

/********************************************************************************************************
 * @file        TLE926x_SPI.h
 *
 * @brief       Declaration file for TLE926x SBC family device SPI functions
 *
 * @version     V1.0.0
 * @date
 * @author      Fedy Farhat
 * @author      Michael Schaffarczyk
 ********************************************************************************************************/

#ifndef TLE926x_SPI_H
#define TLE926x_SPI_H

/* ================================================================================ */
/* ============================   HEADER FILES     ================================ */
/* ================================================================================ */

#include "cy_pdl.h"
#include "cybsp.h"

/*RDK4 Header Files*/
#include "cycfg_pins.h"
#include "cyhal.h"

/* ================================================================================ */
/* =============================   SPI Functions     ============================== */
/* ================================================================================ */
/*RDK4 Adaptation*/
#define RDK4_SPI_CS			SPI_CS
#define RDK4_SPI_MOSI		ARD_SPI_MOSI
#define RDK4_SPI_MISO		ARD_SPI_MISO
#define RDK4_SPI_CLK		ARD_SPI_SCK
#define RDK4_SPI_HANDLE		rdk4_spi_obj
#define RDK4_SPI_FREQ		4000000UL /*SBC Max Frequency 4MHz*/

/**
 * @brief   Initialize the SPI Interface with PDL Driver
 *
 *
 *
 *
 * @retval  Method has to return 0 if initialization was successful.
 */
uint8_t sbc_spi_init(void);

/**
 * @brief   IMPORTANT! THIS METHOD HAS TO BE DEFINED BY THE USER
 *
 *          The function will be called by the library everytime when a SPI communication is needed.
 *          The function proceeds a bidirectional 16-bit transfer to/from the SBC  .
 *          As some UCs only supports 8-Bit transfers, the input arguments are split in two 8-bit arguments.
 *          For further implementation details have a look at datasheet chapter 13.1 or at the Arduino-examples.
 *
 * @param   Upper   The first 8 bit to transmit to the SBC.
 * @param   Lower   The second 8 bit to transmit to the SBC.
 * @retval  The function will return all 16 bits received from the SBC.
 *          Bit[15:8] are the first 8 bits received (Status-Information-Field).
 *          Bit[7:0] is the data-field transmitted of the SBC.
 */
uint16_t SBC_SPI_TRANSFER16(uint8_t Upper, uint8_t Lower);

#endif /* TLE926x_SPI_H */
