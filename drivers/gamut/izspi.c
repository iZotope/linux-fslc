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

#include <linux/module.h>
#include <linux/crc-itu-t.h>
#include "izspi.h"
#include "gamut-ui-main.h"

// would be used to report a key press to input event system
// unused (currently) input system code
//	input_report_key(prv->input, KEY_F1 + i, !!(btn & (1<<i)));
//	input_sync(prv->input);

/**
 * build_tx_frame - assemble a tx frame from packets in a circular buffer
 * @tx: pointer to buffer to build frame in
 * @txbuf: pointer to circular buffer to pull packets from (note, doesn't claim)
 * @can_rx: set if you want the can_rx bit set in the Header
 * @pkts_in_frame: outparam, number of packets put into tx frame, set to Null
 		   if not needed.
 *
 * Return:
 *	number of bytes read from txbuf on success
 *	-1 = error in size payload reporting of txbuf
 */
static int build_tx_frame(u8 *tx, struct cbuf *txbuf, int can_rx, int* pkts_in_frame)
{
	u8 *payload_ptr;
	int pkt_sz;
	int frm_sz;
	u16 crc;
	int err;

	memset(tx, 0, FRM_SZ);

	frm_sz = 0;
	payload_ptr = tx + NDX_IMX_PAYLOAD;
	while (1) {
		pkt_sz = cbuf_peek(txbuf, frm_sz);
		if (pkt_sz < 0) {
			break;
		}

		if (frm_sz + pkt_sz + 1 > IMX_PAYLOAD_SZ) {
			break;
		}

		err = cbuf_copy_out(txbuf, payload_ptr, frm_sz, pkt_sz + 1);
		if (err) {
			// TODO: we lost sync if we are here, wipe the TX buffer?
			return -1;
		}
		//cbuf_consume(txbuf, pkt_sz + 1);
		payload_ptr += pkt_sz + 1;
		if(pkts_in_frame)
			(*pkts_in_frame)++;
		frm_sz += pkt_sz + 1;
	}

	tx[NDX_IMX_HEADER] = frm_sz;
	if (can_rx) {
		tx[0] |= HEADER_CAN_RX_BIT;
	}
	crc = crc_itu_t(0, tx + NDX_IMX_HEADER,
				frm_sz + (NDX_IMX_PAYLOAD - NDX_IMX_HEADER));
	tx[NDX_CRC_HI] = crc >> 8;
	tx[NDX_CRC_LO] = crc & 0xFF;

	return frm_sz;
}


static const char * const imx_ack_err_str[] = {
	/*  0 */ "no error",
	/* -1 */ "sent payload size out of bounds",
	/* -2 */ "CRC mismatch",
	/* -3 */ "traversing packet size doesn't match frame size",
	/* -4 */ "IMX set 'cantrx' but Atmel sent data anyway"
};
/**
 * build_tx_ack - build the appriate tx ack based on the rx buffers
 * @tx: pointer to tx buffer to put ack in (must be FRM_SZ or more)
 * @rx: pointer to rx buffer to base ack on (must be FRM_SZ or more)
 * @pkts_in_frame: outparam, number of packets present in rx frame, set to Null
 		   if not needed.
 * Note that any negative return value indicates a NAK to the Atmel.
 *
 * Return:
 *	0 = rx frame ACK'ED due to being good
 *	-1 = rx frame NAK'ED due to payload size reported out of bounds
 *	-2 = rx frame's CRC didn't match the locally calculated
 *	-3 = traversing the DLL packets didn't add up to the frame size
 *	-4 = IMX was unabled to RX, but atmel sent data anyway
 */
static int build_tx_ack(u8 *tx, const u8 *rx, int* pkts_in_frame)
{
	int frm_sz;
	int pkt_sz;
	u16 crc_rcvd;
	u16 crc_calc;
	int i;
	int rcvd_pkt_cnt = 0;

	if (pkts_in_frame)
		*pkts_in_frame = 0;

	tx[NDX_ACK] = rx[NDX_CRC_HI] & ACK_CRC_MASK;
	tx[NDX_ACK] ^= ACK_CRC_MASK;
	// default to NAK
	tx[NDX_ACK] |= ACK_NAK;

	/* self reported frame payload size must be within limits */
	frm_sz = rx[NDX_ATMEL_HEADER] & HEADER_SIZE_MASK;
	if (frm_sz > ATMEL_PAYLOAD_SZ) {
		return -1;
	}

	/* CRCs must match */
	crc_calc = crc_itu_t(0, rx + NDX_ATMEL_HEADER, frm_sz +
				(NDX_ATMEL_PAYLOAD - NDX_ATMEL_HEADER));

	crc_rcvd = rx[NDX_CRC_HI];
	crc_rcvd <<= 8;
	crc_rcvd |= rx[NDX_CRC_LO];
	if (crc_calc != crc_rcvd) {
		return -2;
	}

	/* traversing the DLL packets must match the reported frame payload */
	i = NDX_ATMEL_PAYLOAD;
	while (i < frm_sz + NDX_ATMEL_PAYLOAD) {
		pkt_sz = rx[i++];
		i += pkt_sz;
		if (i > frm_sz + NDX_ATMEL_PAYLOAD) {
			return -3;
		}
		rcvd_pkt_cnt++;
	}
	if (i - NDX_ATMEL_PAYLOAD !=
			(rx[NDX_ATMEL_HEADER] & HEADER_SIZE_MASK)) {
		return -3;
	}

	if (!(tx[NDX_IMX_HEADER] & HEADER_CAN_RX_BIT) &&
		(rx[NDX_ATMEL_HEADER] & HEADER_SIZE_MASK)) {
		return -4;
	}

	if (pkts_in_frame) {
		*pkts_in_frame = rcvd_pkt_cnt;
	}

	// clear the NAK and put an ACK
	tx[NDX_ACK] &= ~ACK_MASK;
	tx[NDX_ACK] |= ACK_ACK;
	 return 0;
}


static const char * const atmel_ack_err_str[] = {
	/*  0 */ "no error",
	/* -1 */ "Checksum mismatch from ATMEL",
	/* -2 */ "Atmel set NAK bit",
};

 /**
  * check_ack - check the ack of a rx frame
  * @rx: pointer to full rx frame
  * @tx: pointer to full tx frame associated with the above rx
  *
  * Return:
  *	0 = ack okay
  *	-1 = inverted checksum mismatch from Atmel
  *	-2 = NAK'd by Atmel
  */
static int check_ack(const u8 *rx, const u8* tx)
{
	u8 crc = tx[NDX_CRC_HI];
	crc ^= 0xFF;
	crc &= ACK_CRC_MASK;


	if (crc != (rx[NDX_ACK] & ACK_CRC_MASK)) {
		return -1;
	}

	if ( !((rx[NDX_ACK] & ACK_ACK) && !(rx[NDX_ACK] & ACK_NAK)) ) {
		return -2;
	}

	return 0;
}

/**
 * xfer_frame - xfers a single frame to/from the atmel
 *
 * Return:
 *	bit field of events, some good, some bad
 *	IZSPI_DATA_TXED = data was transmitted
 *	IZSPI_DATA_RXED = data was recieved
 *	IZSPI_SPI_ERROR = there was an SPI error
 *	IZSPI_ATMEL_RX_BUF_FULL = The atmel refused reception of data due to
 				  full buffer
 *	IZSPI_ATMEL_RX_ERROR = other unspcified error from atmel relating to
 			       reception
 */
static int xfer_frame(struct izspi *izspi)
{
	/* TODO: move any printfs to the thend of the function
	having error reporting inbetween the data phase and ACK pahse
	causes a timing gap on the wire.
	*/
	struct cbuf *rxbuf = &izspi->rxbuf;
	int frm_sz;
	int tx_payload_sz;
	int err;
	int keep_rx_data = 1;
	int ret = 0;
	int pkts_sent = 0;
	int pkts_rcvd = 0;

	keep_rx_data = cbuf_space(&izspi->rxbuf) >= IMX_PAYLOAD_SZ;
	tx_payload_sz = build_tx_frame(izspi->tx, &izspi->txbuf, keep_rx_data, 
					&pkts_sent);
	if (tx_payload_sz < 0) {
		dev_err(izspi->dev, "Lost sync in the TX cbuf\n");
		// TODO: I think we should clear the TX queue, build an
		// empty tx packet and carry on, the Atmel might still want
		// to say somthing
		return -1;
	}

	/* transmit upto the CRC */
	/* TODO: what to do about the SS line if the first SPI transaction
	errors out?
	*/
	err = gamut_ui_spi_xfer(izspi, izspi->tx, izspi->rx, NDX_CRC_LO + 1, 1);
	if (err) {
		dev_err(izspi->dev, "Error with SPI transaction: %d\n", err);
		ret |= IZSPI_SPI_ERROR;
	}

	err = build_tx_ack(izspi->tx, izspi->rx, &pkts_rcvd);
	if (err) {
		keep_rx_data = 0;
		if (err != -4) {
			dev_err(izspi->dev, "IMX NAK'd the Atmel packet. Error: %d, '%s'\n",
						err, imx_ack_err_str[abs(err)]);
		}
	} else if (err != -2) { /* if rx'd packet has a good checksum */
		/* see if atmel's rx buffer is full */
		if (!(izspi->rx[NDX_ATMEL_HEADER] & HEADER_CAN_RX_BIT)) {
			ret |= IZSPI_ATMEL_RX_BUF_FULL;
		}
	}

	/* transmit the ACK that we built */
	err = gamut_ui_spi_xfer(izspi, izspi->tx + NDX_CRC_LO + 1,
				       izspi->rx + NDX_CRC_LO + 1,
				       NDX_ACK - NDX_CRC_LO, 0);
	if (err) {
		dev_err(izspi->dev, "Error with SPI transaction: %d\n", err);
		ret |= IZSPI_SPI_ERROR;
	}

	err = check_ack(izspi->rx, izspi->tx);
	if (err >= 0) {
		izspi->txpkt_cnt += pkts_sent;
		cbuf_consume(&izspi->txbuf, tx_payload_sz);
		if (tx_payload_sz) {
			ret |= IZSPI_DATA_TXED;
		}
	} else {
		dev_err(izspi->dev, "Atmel ACK problem, Error: %d, '%s'\n",
			err, atmel_ack_err_str[abs(err)]);
		ret |= IZSPI_ATMEL_RX_ERROR;
	}

	if (keep_rx_data) {
		frm_sz =  izspi->rx[NDX_ATMEL_HEADER] & HEADER_SIZE_MASK;
		if (frm_sz > cbuf_space(rxbuf)) {
			// block until we can copy in the RX bufferr
			//retun for now with warning
			//shouldn't get here?
			return -1;
		}
		cbuf_copy_in(rxbuf, &izspi->rx[NDX_ATMEL_PAYLOAD], 0, frm_sz);
		izspi->rxpkt_cnt += pkts_rcvd;
		cbuf_produce(rxbuf, frm_sz);
		if (frm_sz) {
			ret |= IZSPI_DATA_RXED;
		}
	}

	return ret;
}

/**
 * izspi_can_tx_frame - check if an izspi can transmit a frame
 * @izspi: pointer to izspi to check
 *
 * Return:
 *	1: able to tx a frame if izspi_xfer_frame was Called
 *	0: not ready to xfer frame.
 */
int izspi_can_tx_frame(struct izspi *izspi)
{
	if (cbuf_cnt(&izspi->txbuf)) {
		return 1;
	}
	return 0;
}

/**
 * izspi_can_rx_frame - checks if we can rx a frame based on the buffer fullness
 *
 *
 * Return:
 *	0 = can't rx a packet
 *	1 = can rx a packet
 */
int izspi_can_rx_frame(const struct izspi *izspi)
{
	if (cbuf_space(&izspi->rxbuf) >= ATMEL_PAYLOAD_SZ) {
		return 1;
	}
	return 0;
}


/**
 * izspi_xfer_frame - trancieve a frame with the Atmel
 * @izspi: pointer to izspi instance
 *
 * This will trancieve one frame with the Atmel
 *
 * Return:
 *	postive value deontes success with a bitfiled for actions perfromed
 *	IZSPI_DATA_TXED = data was transmitted
 *	IZSPI_DATA_RXED = data was recieved
 *	-EIO = error with SPI transaction
 */
int izspi_xfer_frame(struct izspi *izspi)
{
	int ret = 0;

	/* We hold the tx consumer mutex through the whole transaction to
	   proctect the izspi->tx and izspi->rx buffers as well. No need to take
	   the rx producer mutex as this is the only producer and is covered by
	   the consumer mutex. If we were paranoid we could take both */
	mutex_lock(&izspi->txbuf.consumer_mutex);
	ret = xfer_frame(izspi);
	mutex_unlock(&izspi->txbuf.consumer_mutex);

	return ret;
}

/**
 * izspi_write_packet - write a packet into the izspi cbuf
 * @izspi: pointer to izspi to put packet into
 * @buf: pointer to buffer to copy packet from.
 * @count: number of bytes in the packet
 *
 * Return:
 *	on success the number of bytes written in, can be 0
 *	-ENOSPC = no space left in buffer, nothing written into buffer
 *	-EFBIG = tried to write a packet that is > max IMX packet size
 *	TODO: any other error cases?
 */
int izspi_write_packet(struct izspi *izspi, char *buf, int count)
{
	struct cbuf *txbuf = &izspi->txbuf;
	int err = 0;
	char tmp = count;

	if (count > IMX_PAYLOAD_SZ + 1) {
		return -EFBIG;
	}

	mutex_lock(&txbuf->producer_mutex);
	if (cbuf_space(txbuf) < count + 1) {
		mutex_unlock(&txbuf->producer_mutex);
		return -ENOSPC;
	}
	tmp = count;
	// TODO: check error codes
	err = cbuf_copy_in(txbuf, &tmp, 0, 1);
	err = cbuf_copy_in(txbuf, buf, 1, tmp);
	cbuf_produce(txbuf, tmp + 1);

	mutex_unlock(&txbuf->producer_mutex);

	return count;
}

/**
 * izspi_read_packet - read a packet from the izspi cbuf into a bufferr
 * @izspi: pointer to izspi to get frame from
 * @buf: pointer to buffer to copy frame into. Must be atleast PK_SZ in size
 *
 * Return:
 *	number of bytes copied into buf on success, 0 indicates no data avalibe
 *	TODO: error on circ buff was out of sync
 */
int izspi_read_packet(struct izspi *izspi, char *buf)
{
	struct cbuf *rxbuf = &izspi->rxbuf;
	int pkt_sz;
	int err;

	mutex_lock(&rxbuf->consumer_mutex);
	pkt_sz = cbuf_peek(rxbuf, 0);
	if (pkt_sz < 0) {
		mutex_unlock(&rxbuf->consumer_mutex);
		return 0;
	}

	err = cbuf_copy_out(rxbuf, buf, 1, pkt_sz);
	if(err < 0) {
		// TODO: return an error and clear the RX circ buffer
		mutex_unlock(&rxbuf->consumer_mutex);
		return 0;
	}
	cbuf_consume(rxbuf, pkt_sz + 1);
	mutex_unlock(&rxbuf->consumer_mutex);
	return pkt_sz;
}


/**
 * izspi_can_read_packet - returns if the rx queue has a packet to read
 *
 * Return:
 *	0 = not ready for packet to be read
 *	1 = ready for packet to be read
 */
int izspi_can_read_packet(const struct izspi *izspi)
{
	return cbuf_cnt(&izspi->rxbuf);
}

/**
 * izspi_can_write_packet - returns if the tx queue is ready for a packet
 *
 * Return:
 *	0 = not ready for packet to be written
 *	1 = ready for packet to be written
 */
int izspi_can_write_packet(const struct izspi *izspi)
{
	return cbuf_space(&izspi->txbuf) > IMX_PAYLOAD_SZ;
}
