/*
 * iZotope Gamut UI driver
 *
 * Copyright (C) 2016-2017, iZotope inc.
 *
 * Authors: Matthew Campbell <mcampbell@izotope.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __IZSPI_H
#define __IZSPI_H

#include <linux/module.h>
#include <linux/mutex.h>
#include "izcbuf.h"

/* defines for SPI comms */
#define FRM_SZ (128) /* max size of a phy frame */
#define BUF_SZ (2048) /* size of the circular buffer for queeuing */
#if FRM_SZ > BUF_SZ
#error FRM_SZ must be < BUF_SZ
#endif

/* Header byte bit fields */
#define HEADER_SIZE_MASK (0x7F)
#define HEADER_CAN_RX_BIT (0x80)

/* Indexes of bytes in packets */
#define NDX_ATMEL_HEADER (2)
#define NDX_IMX_HEADER (0)
#define NDX_ATMEL_PAYLOAD (3)
#define NDX_IMX_PAYLOAD (1)
#define NDX_CRC_HI (123)
#define NDX_CRC_LO (124)
#define NDX_ACK (127)
#define ACK_ACK (0x80)
#define ACK_NAK (0x40)
#define ACK_MASK (ACK_ACK | ACK_NAK)
#define ACK_CRC_MASK (0x3F)
#define IMX_PAYLOAD_SZ (NDX_CRC_HI - NDX_IMX_PAYLOAD)
#define ATMEL_PAYLOAD_SZ (NDX_CRC_HI - NDX_ATMEL_PAYLOAD)

/* Return codes for izspi_xfer_frame */
#define IZSPI_DATA_TXED (1 << 0)
#define IZSPI_DATA_RXED (1 << 1)
#define IZSPI_SPI_ERROR (1 << 2)
#define IZSPI_ATMEL_RX_BUF_FULL (1 << 3)
#define IZSPI_ATMEL_RX_ERROR (1 << 4)

/*
 * Packets in the ring buffer follow this format:
 *      byte 0 : size of payload
 * 1 - payload : payload
 * rinse and repeat
 */
struct izspi {
	struct spi_device *spi;
	struct device *dev;
	/* buffers for immediate transmission */
	u8 tx[FRM_SZ];
	u8 rx[FRM_SZ];

	struct cbuf txbuf;
	struct cbuf rxbuf;
	/* debugging info */
	uint32_t txpkt_cnt;
	uint32_t rxpkt_cnt;
};

int izspi_xfer_frame(struct izspi *izspi);
int izspi_can_tx_frame(struct izspi *izspi);
int izspi_can_rx_frame(const struct izspi *izspi);
int izspi_write_packet(struct izspi *izspi, char *buf, int count);
int izspi_read_packet(struct izspi *izspi,  char *buf);
int izspi_can_read_packet(const struct izspi *izspi);
int izspi_can_write_packet(const struct izspi *izspi);

#endif /* __IZSPI_H */
