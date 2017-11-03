#include <string.h>
#include <stdio.h>
#include "../izspi.h"
#include "../gamut-ui-main.h"
#include "unity.h"
#include "crc16.h"

/* globals for use in mocking and tests */
#define GLOBAL_BUF_SZ (BUF_SZ * 2)
static unsigned char g_rx[GLOBAL_BUF_SZ], g_tx[GLOBAL_BUF_SZ];
static unsigned char g_tmpbuf[GLOBAL_BUF_SZ];
static unsigned char *g_prx, *g_ptx;
static struct izspi izspi;

/* internal helper functions */
static void print_buffer(uint8_t *buf, int size)
{
	int i,j;

	const int stride = 16;
	printf("     |");
	for(i=0; i<stride; i++) {
		printf(" 0x%02X",i);
	}
	printf("\n-----+");
	for(i=0; i<stride; i++) {
		printf("-----");
	}
	printf("\n");
	for(i=0; i<size; i+=stride) {
		printf("0x%02X |",i);
		for(j=i;j<i+stride;j++) {
			if(j == size)
				break;
			printf(" 0x%02X",buf[j]);
		}
		printf("\n");
	}
	printf("\n");
}

enum pkt_type {
	PKT_IMX,
	PKT_ATMEL,
};

/* outbuf must be 128 or larger! */
// init param means weather to start from scartach or not
#define FRM_INIT	(1 << 0) // init the frame, otherwise just append packets
#define FRM_NO_RX	(1 << 1) // clear the 'can RX' bit
static void build_frame(enum pkt_type type, char * out_buf, const char* payload, int size, unsigned int opts)
{
	int ndx_payload, ndx_header, payload_sz, cur_size;
	unsigned char rx_was_set;

	if(type==PKT_IMX) {
		ndx_payload = NDX_IMX_PAYLOAD;
		ndx_header = NDX_IMX_HEADER;
		payload_sz = IMX_PAYLOAD_SZ;
	} else if(type==PKT_ATMEL) {
		ndx_payload = NDX_ATMEL_PAYLOAD;
		ndx_header = NDX_ATMEL_HEADER;
		payload_sz = ATMEL_PAYLOAD_SZ;
	} else {
		TEST_FAIL_MESSAGE("Unit test fail: called build packet with invalid type");
		return;
	}

	if (opts & FRM_INIT) {
		memset(out_buf, 0, FRM_SZ);
		out_buf[ndx_header] = HEADER_CAN_RX_BIT;
	}

	cur_size = out_buf[ndx_header] & HEADER_SIZE_MASK;
	if(size + cur_size > payload_sz) {
		TEST_FAIL_MESSAGE("Unit test fail: Called build_atmel_packet with paylod too big!");
	}

	if (payload) {
		// setup the Header
		rx_was_set = out_buf[ndx_header] & HEADER_CAN_RX_BIT;
		out_buf[ndx_header] = (size + cur_size + 1) & HEADER_SIZE_MASK;
		if(rx_was_set)
			out_buf[ndx_header] |= HEADER_CAN_RX_BIT;

		out_buf[ndx_payload + cur_size] = size;
		memcpy(out_buf + ndx_payload + cur_size + 1, payload, size);

		cur_size += size + 1;

	} else {
		// force size to zero if the payload buffer was null
		size = 0;
	}

	if(opts & FRM_NO_RX)
		out_buf[ndx_header] &= HEADER_SIZE_MASK;

	// add 2 on end for header byte of frame and packet
	uint16_t crc = crc16_block(0, out_buf + ndx_header, cur_size + 1);
	out_buf[NDX_CRC_HI] = crc >> 8;
	out_buf[NDX_CRC_LO] = crc & 0xFF;

}

static void build_ack(u8 *buf1, u8 *buf2)
{
	buf1[NDX_ACK] = buf2[NDX_CRC_HI] & ACK_CRC_MASK;
	buf1[NDX_ACK] ^= ACK_CRC_MASK;
	buf1[NDX_ACK] |= ACK_ACK;

	buf2[NDX_ACK] = buf1[NDX_CRC_HI] & ACK_CRC_MASK;
	buf2[NDX_ACK] ^= ACK_CRC_MASK;
	buf2[NDX_ACK] |= ACK_ACK;
}

static void reset_spi_simulator()
{
	memset(g_rx, 0, sizeof(g_rx));
	g_prx = g_rx;
	memset(g_tx, 0, sizeof(g_tx));
	g_ptx = g_tx;
}

/* Mocked out/stubbed functions */
int gamut_ui_spi_xfer(struct izspi *izspi, char *tx, char *rx, int size, int hold_ss)
{
	// do the RX
	if (g_prx + size > g_rx + sizeof(g_rx)) {
		TEST_FAIL_MESSAGE("Unit test error, underran the fake SPI rx buffer when reading");
	}

	memcpy(rx, g_prx, size);
	g_prx += size;

	// do the TX
	if (g_ptx + size > g_tx + sizeof(g_tx)) {
		TEST_FAIL_MESSAGE("Unit test error, overran the tx buffer when writing");
	}

	memcpy(g_ptx, tx, size);
	g_ptx += size;

	return 0;
}

/* setup and teardown */
void setUp(void)
{
	memset(&izspi, 0, sizeof(izspi));
	reset_spi_simulator();
	memset(g_tmpbuf, 0, sizeof(g_tmpbuf));
	cbuf_devm_init(&izspi.txbuf, NULL, BUF_SZ);
	cbuf_devm_init(&izspi.rxbuf, NULL, BUF_SZ);
}

void tearDown(void)
{
	devm_freeall();
}

/* the actual tests */

/********************
 * Test the test code used in this file (meta-tests)
 *******************/
 /* test the test bench */
 void test_packet_builder(void)
 {
	char out_buf1[] = {0xDE, 0xAD, 0xBE, 0xEF};
	char out_buf2[] = {0x12, 0x34, 0x56};
	char re[FRM_SZ];
	int ndx;

	// test the atmel
	build_frame(PKT_ATMEL, g_tmpbuf, out_buf1, sizeof(out_buf1), FRM_INIT);
	build_frame(PKT_ATMEL, g_tmpbuf, out_buf2, sizeof(out_buf2), 0);

	memset(re, 0, sizeof(re));
	re[NDX_ATMEL_HEADER] = 0x89; // can rx with payload of 9
	ndx = NDX_ATMEL_PAYLOAD;
	re[ndx++] = 0x04; //size of out_buf1;
	re[ndx++] = 0xde; //out_buf1 payload
	re[ndx++] = 0xad;
	re[ndx++] = 0xbe;
	re[ndx++] = 0xef;
	re[ndx++] = 0x03; //size of out_buf2
	re[ndx++] = 0x12; //out_buf2 payload
	re[ndx++] = 0x34;
	re[ndx++] = 0x56;
	re[NDX_CRC_HI] = 0xba; // manually calculated
	re[NDX_CRC_LO] = 0x63;
	TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(re, g_tmpbuf, sizeof(re),"Built ATMEL packet didn't match truth packet");

	// test imx packet
	build_frame(PKT_IMX, g_tmpbuf, out_buf1, sizeof(out_buf1), FRM_INIT);
	build_frame(PKT_IMX, g_tmpbuf, out_buf2, sizeof(out_buf2), 0);

	memset(re, 0, sizeof(re));
	re[NDX_IMX_HEADER] = 0x89; // can rx with payload of 9
	ndx = NDX_IMX_PAYLOAD;
	re[ndx++] = 0x04; //size of out_buf1;
	re[ndx++] = 0xde;
	re[ndx++] = 0xad;
	re[ndx++] = 0xbe;
	re[ndx++] = 0xef;
	re[ndx++] = 0x03;
	re[ndx++] = 0x12;
	re[ndx++] = 0x34;
	re[ndx++] = 0x56;
	re[NDX_CRC_HI] = 0xba; // manually calculated
	re[NDX_CRC_LO] = 0x63;
	TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(re, g_tmpbuf, sizeof(re), "Built IMX  packet didn't match truth packet");
 }

void test_packet_builder_empty_packet(void)
{
	char re[FRM_SZ];

	build_frame(PKT_ATMEL, g_tmpbuf, NULL, 0, FRM_INIT);

	memset(re, 0, sizeof(re));

	re[NDX_ATMEL_HEADER] = 0x80;
	re[NDX_CRC_HI] = 0x91;
	re[NDX_CRC_LO] = 0x88;

	TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(re, g_tmpbuf, sizeof(re),"Built ATMEL packet didn't match truth packet");
}

 /********************
  * Normal (success) cases
  *******************/
/* make sure a write puts a packet in the cbuf */
void test_write_packet_to_cbuf(void)
{
	char out_buf[] = {0x00, 0xDE, 0xAD, 0xBE, 0xEF};
	out_buf[0] = sizeof(out_buf) - 1;
	izspi_write_packet(&izspi, out_buf + 1, sizeof(out_buf) - 1);

	TEST_ASSERT_EQUAL_INT_MESSAGE(sizeof(out_buf),cbuf_cnt(&izspi.txbuf),"Cbuf didn't report corret number of bytes");
	TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(out_buf,izspi.txbuf.buf,sizeof(out_buf),"packet didn't end up in circular buffer");
}

/* test every permintation of tail, index, count */
void test_copy_out_permintations(void)
{
	const int cpyout_sz = 50;
	char cpyout[50];
	struct cbuf b;
	const int size = 128;
	cbuf_devm_init(&b, NULL, size);

	memset(b.buf,5,b.size);

	for(b.tail=0; b.tail < b.size; b.tail++) {
		// set the buffer to full
		b.head = (b.tail - 1) & (b.size - 1);
		for (int ndx=0; ndx<b.size-1; ndx++) {
			for (int count=0; count+ndx<cpyout_sz; count++) {
				memset(cpyout,12,sizeof(cpyout));
				cbuf_copy_out(&b, cpyout, ndx, count);
				int i;
				for(i=0; i<sizeof(cpyout); i++) {
					if(cpyout[i] != 5)
						break;
				}
				if (count != i) {
					printf("error\n");
				}
				TEST_ASSERT_EQUAL_INT_MESSAGE(count,i,"copied wrong number of bytes in");
			}
		}
	}
}

/* test recieving a frame with a single packet from the atmel */
void test_recv_frame_from_atmel(void)
{
	char out_buf[] = {0xDE, 0xAD, 0xBE, 0xEF};

	build_frame(PKT_ATMEL, g_rx, out_buf, sizeof(out_buf), FRM_INIT);
	// build the responce packet
	build_frame(PKT_IMX, g_tmpbuf, NULL, 0, FRM_INIT);
	build_ack(g_rx, g_tmpbuf);

	izspi_xfer_frames(&izspi);
	int ret = izspi_read_packet(&izspi, g_tmpbuf);
	TEST_ASSERT_EQUAL_INT_MESSAGE(sizeof(out_buf),ret,"Read back wrong number of bytes in packet.");

	TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(out_buf, g_tmpbuf, sizeof(out_buf),"Read back packet was wrong");
}

void test_send_frame_to_atmel(void)
{
	char out_buf[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE};
	int err;
	// packt to compare against
	build_frame(PKT_IMX, g_tmpbuf, out_buf, sizeof(out_buf), FRM_INIT);
	// responce from the atmel
	build_frame(PKT_ATMEL, g_rx, NULL, 0, FRM_INIT);
	// build the acks for both
	build_ack(g_rx, g_tmpbuf);

	err = izspi_write_packet(&izspi, out_buf, sizeof(out_buf));
	TEST_ASSERT_EQUAL_INT_MESSAGE(sizeof(out_buf), err,"Packet write was unsuccessful");


	izspi_xfer_frames(&izspi);

	TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(g_tmpbuf, g_tx, FRM_SZ,"Packet sent from IXM to ATMEL not correct.");
}

/* Note that this is a pretty contrived case, but we ran into this in testing.
It's where cbuf_peak was not wrapping around the cbuf. When it peaks to see The
size of the next packet and if it will fit, it's getting the high byte of the
head member of the cbuf struct. If this size would appear to fit it charges on
ahead and copies the too big packet into the frame (with the proper packet size
as cbuf_copy_out wraps properly). At this point your DLL packets don't add up
to what you thought would fit in a frame and you have a malformed frame.
*/
void test_write_packet_over_cbuf_boundry(void)
{
	// force cbuf to hit a boundry
	izspi.txbuf.head = 257; // make the high byte of head 1
	izspi.txbuf.tail = BUF_SZ - 121;
	memset(izspi.txbuf.buf,120,izspi.txbuf.size);
	memset(izspi.txbuf.buf,50,51);

	char out_buf[IMX_PAYLOAD_SZ - 2];
	memset(out_buf, 0x44, sizeof(out_buf));
	char dummy_buf[64];
	memset(out_buf, 0x55, sizeof(dummy_buf));

	// packt to compare against
	build_frame(PKT_IMX, g_tmpbuf, izspi.txbuf.buf + izspi.txbuf.tail, 120, FRM_INIT);
	// responce from the atmel
	build_frame(PKT_ATMEL, g_rx, NULL, 0, FRM_INIT);
	// build the acks for both
	build_ack(g_rx, g_tmpbuf);

	int res = izspi_write_packet(&izspi, out_buf, sizeof(out_buf));
	TEST_ASSERT_EQUAL_INT_MESSAGE(sizeof(out_buf),res,"Didn't write packet into buffer");

	izspi_xfer_frames(&izspi);

	TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(g_tmpbuf, g_tx, FRM_SZ,"Packet sent from IXM to ATMEL not correct.");
}

// make a frame with multiple collesed packets and read them all
void test_recv_collesed_frame_from_atmel(void)
{
	char out_buf1[] = {0x01, 0x02, 0x03,};
	char out_buf2[] = {0x01, 0xDE,};
	char out_buf3[] = {0x08, 0xff, 0xfe, 0xcc, 0x12, 0x33, 0xbc, 0x33, 0xbc,};
	build_frame(PKT_ATMEL, g_rx, out_buf1, sizeof(out_buf1), FRM_INIT);
	build_frame(PKT_ATMEL, g_rx, out_buf2, sizeof(out_buf2), 0);
	build_frame(PKT_ATMEL, g_rx, out_buf3, sizeof(out_buf3), 0);
	// build the IMX frame for the acks
	build_frame(PKT_IMX, g_tmpbuf, NULL, 0, FRM_INIT);
	// build the acks
	build_ack(g_rx, g_tmpbuf);

	izspi_xfer_frames(&izspi);

	TEST_ASSERT_EQUAL_INT_MESSAGE(sizeof(out_buf1),izspi_read_packet(&izspi, g_tmpbuf),"packet 1 read wasn't right size");
	TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(out_buf1, g_tmpbuf, sizeof(out_buf1),"Packet 1 didn't match what was sent from Atmel");
	TEST_ASSERT_EQUAL_INT_MESSAGE(sizeof(out_buf2),izspi_read_packet(&izspi, g_tmpbuf),"packet 2 read wasn't right size");
	TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(out_buf2, g_tmpbuf, sizeof(out_buf2),"Packet 2 didn't match what was sent from Atmel");
	TEST_ASSERT_EQUAL_INT_MESSAGE(sizeof(out_buf3),izspi_read_packet(&izspi, g_tmpbuf),"packet 4 read wasn't right size");
	TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(out_buf3, g_tmpbuf, sizeof(out_buf3),"Packet 3 didn't match what was sent from Atmel");
}

void test_send_collesed_to_atmel(void)
{
	int err;
	char outbuf1[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE};
	char outbuf2[] = {0x80, 0x80, 0x0b, 0xad, 0xfe};
	build_frame(PKT_IMX, g_tmpbuf, outbuf1, sizeof(outbuf1), FRM_INIT);
	build_frame(PKT_IMX, g_tmpbuf, outbuf2, sizeof(outbuf2), 0);
	// build an empy ack frame from atmel
	build_frame(PKT_ATMEL, g_rx, NULL, 0, FRM_INIT);
	build_ack(g_rx, g_tmpbuf);

	err = izspi_write_packet(&izspi, outbuf1, sizeof(outbuf1));
	TEST_ASSERT_EQUAL_INT_MESSAGE(sizeof(outbuf1), err,"Packet 1 write was unsuccessful");
	err = izspi_write_packet(&izspi, outbuf2, sizeof(outbuf2));
	TEST_ASSERT_EQUAL_INT_MESSAGE(sizeof(outbuf2), err,"Packet 2 write was unsuccessful");

	izspi_xfer_frames(&izspi);
	TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(g_tmpbuf, g_tx, FRM_SZ,"Collesed frame that was sent wasn't correct");
}

/* test that queueing up packets to the atmel that add up to more than
 * IMX_PAYLOAD_SZ results in two frames sent, also that packets are split
 * between frames without breaking up */
void test_send_collesed_to_atmel_larger_than_frame(void)
{
	int err;
	char outbuf1[IMX_PAYLOAD_SZ / 2]; // first two fill the first frame
	char outbuf2[IMX_PAYLOAD_SZ / 3];
	char outbuf3[IMX_PAYLOAD_SZ / 2]; // this should go in second frame
	// init the buffers to different values
	for (int i=0; i<IMX_PAYLOAD_SZ/2; i++) {
		if (i < sizeof(outbuf1))
			outbuf1[i] = 0x10 | (i & 0xF);
		if (i < sizeof(outbuf2))
			outbuf2[i] = 0x20 | (i & 0xF);
		if (i < sizeof(outbuf3))
			outbuf3[i] = 0x30 | (i & 0xF);
	}

	// queue up more than IMX_PAYLOAD_SZ in packets
	err = izspi_write_packet(&izspi, outbuf1, sizeof(outbuf1));
	TEST_ASSERT_EQUAL_INT_MESSAGE(sizeof(outbuf1), err,"Packet 1 write was unsuccessful");
	err = izspi_write_packet(&izspi, outbuf2, sizeof(outbuf2));
	TEST_ASSERT_EQUAL_INT_MESSAGE(sizeof(outbuf2), err,"Packet 2 write was unsuccessful");
	err = izspi_write_packet(&izspi, outbuf3, sizeof(outbuf3));
	TEST_ASSERT_EQUAL_INT_MESSAGE(sizeof(outbuf3), err,"Packet 3 write was unsuccessful");

	// build all packets in advance
	// build the first gold frame to compare against
	build_frame(PKT_IMX, g_tmpbuf, outbuf1, sizeof(outbuf1), FRM_INIT);
	build_frame(PKT_IMX, g_tmpbuf, outbuf2, sizeof(outbuf2), 0);
	//build the second gold frame
	build_frame(PKT_IMX, g_tmpbuf + FRM_SZ, outbuf3, sizeof(outbuf3), FRM_INIT);
	// build first responce frame
	build_frame(PKT_ATMEL, g_rx, NULL, 0, FRM_INIT);
	// build second responce frame
	build_frame(PKT_ATMEL, g_rx + FRM_SZ, NULL, 0, FRM_INIT);


	// setup acks for both
	build_ack(g_rx, g_tmpbuf);
	build_ack(g_rx + FRM_SZ, g_tmpbuf + FRM_SZ);

	// Do the transfer
	izspi_xfer_frames(&izspi);

	TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(g_tmpbuf, g_tx, FRM_SZ * 2,"First collesed frame wasn't correct");
}

/********************
 * Not really fail cases, but kinda contrived
 ********************/
void test_read_packet_returns_0_on_empty_queue(void)
{
	TEST_ASSERT_EQUAL_INT_MESSAGE(0,izspi_read_packet(&izspi, g_tmpbuf),"Read packet didn't return 0 when nothing in it");
}

void test_rxokay_bit_not_set_when_rxqueue_full(void)
{
	int err;
	char out_buf[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE};
	// fill the RX buffer completely
	struct cbuf *rxbuf = &izspi.rxbuf;
	while(cbuf_space(rxbuf)) {
		cbuf_copy_in(rxbuf, g_tmpbuf, 0, 1);
		cbuf_produce(rxbuf, 1);
	}

	// build the IMX frame
	build_frame(PKT_IMX, g_tmpbuf, out_buf, sizeof(out_buf), FRM_INIT|FRM_NO_RX);
	// build an empy ack frame from atmel
	build_frame(PKT_ATMEL, g_rx, NULL, 0, FRM_INIT);
	build_ack(g_rx, g_tmpbuf);

	err = izspi_write_packet(&izspi, out_buf, sizeof(out_buf));
	TEST_ASSERT_EQUAL_INT_MESSAGE(sizeof(out_buf), err,"Packet write was unsuccessful");

	izspi_xfer_frames(&izspi);

	// make sure the packet went out from paranoia
	TEST_ASSERT_EQUAL_INT_MESSAGE(sizeof(out_buf) + 1, g_tx[0] & HEADER_SIZE_MASK, "Desn't look like the packet was sent properly from IMX");
	TEST_ASSERT_BITS_LOW_MESSAGE(HEADER_CAN_RX_BIT, g_tx[0],"Can_rx bit was set from IMX when rxbuffer was full");
}

void test_nak_to_atmel_when_imx_said_cant_rx_and_atmel_sends_data(void)
{
	char out_buf[] = {0xDE};

	struct cbuf *rxbuf = &izspi.rxbuf;
	while(cbuf_space(rxbuf)) {
		cbuf_copy_in(rxbuf, g_tmpbuf, 0, 1);
		cbuf_produce(rxbuf, 1);
	}

	build_frame(PKT_IMX, g_tmpbuf, NULL, 0, FRM_INIT|FRM_NO_RX);
	build_frame(PKT_ATMEL, g_rx, out_buf, sizeof(out_buf), FRM_INIT);
	build_ack(g_rx, g_tmpbuf);

	izspi_xfer_frames(&izspi);

	TEST_ASSERT_BITS_MESSAGE(ACK_MASK, ACK_NAK, g_tx[NDX_ACK],"IMX didn't send a NAK");
}


/********************
 * Failure cases
 *******************/
void test_send_too_large_packet_fails(void)
{
	char buf[2 * FRM_SZ];
	int err;

	err = izspi_write_packet(&izspi, buf, sizeof(buf));
	TEST_ASSERT_EQUAL_INT_MESSAGE(-EFBIG, err,"Didn't return -EBIGF when write too large a packet");
}

void test_nospc_returned_on_tx_buf_full(void)
{
	int err;
	// fill the RX buffer completely
	struct cbuf *txbuf = &izspi.txbuf;
	while(cbuf_space(txbuf)) {
		cbuf_copy_in(txbuf, g_tmpbuf, 0, 1);
		cbuf_produce(txbuf, 1);
	}

	err = izspi_write_packet(&izspi, g_tmpbuf, 10);
	TEST_ASSERT_EQUAL_INT_MESSAGE(-ENOSPC, err,"Didn't report -ENOSPC on write packet when txbuf was full");
}

void test_nak_bad_crc(void)
{
	char outbuf[] = {0x01, 0x02, 0x03};
	build_frame(PKT_ATMEL, g_rx, outbuf, sizeof(outbuf), FRM_INIT);
	// corrupt a bit, CRC is now invalid
	g_rx[NDX_ATMEL_PAYLOAD+1] ^= 0x40;

	izspi_xfer_frames(&izspi);

	TEST_ASSERT_BITS_MESSAGE(ACK_ACK | ACK_NAK, ACK_NAK, g_tx[NDX_ACK],"Should NAK bad packet");

	int err = izspi_read_packet(&izspi, g_tmpbuf);
	TEST_ASSERT_EQUAL_INT_MESSAGE(0, err, "Shouldn't be anything in the RX queue");
}

// if we recieve a packet with a frame size that doesn't match up when
// treversing collesed (or a single) praket we need to nak and drop.
void test_nak_dll_size_not_equal_frame_size(void)
{
	char out_buf[] = {0xDE, 0xAD, 0xBE, 0xEF};

	build_frame(PKT_ATMEL, g_rx, out_buf, sizeof(out_buf), FRM_INIT);
	// tweak the packet and update the crc
	g_rx[NDX_ATMEL_PAYLOAD] = sizeof(out_buf) + 5;
	uint16_t crc = crc16_block(0, g_rx + NDX_ATMEL_HEADER, sizeof(out_buf) + 2);
	g_rx[NDX_CRC_HI] = crc >> 8;
	g_rx[NDX_CRC_LO] = crc & 0xFF;
	// build the responce packet
	build_frame(PKT_IMX, g_tmpbuf, NULL, 0, FRM_INIT);
	build_ack(g_rx, g_tmpbuf);

	izspi_xfer_frames(&izspi);

	TEST_ASSERT_BITS_MESSAGE(ACK_MASK, ACK_NAK, g_tx[NDX_ACK],"Should NAK bad packet.");

	int err = izspi_read_packet(&izspi, g_tmpbuf);
	TEST_ASSERT_EQUAL_INT_MESSAGE(0, err, "Shouldn't be anything in the RX queue");
}

// make sure we retransmit a packet after it being marked as no can rx once
// also make sure we kept the data that the atmel sent with the no can rx frame
void test_imx_retransmit_after_no_canrx(void)
{
	int err;
	char out_buf[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE};
	char atmel_buf[] = {0xff, 0xee, 0xbb};
	char inbuf[FRM_SZ];
	memset(inbuf,0,sizeof(inbuf));
	// packt to compare against
	build_frame(PKT_IMX, g_tmpbuf, out_buf, sizeof(out_buf), FRM_INIT);
	// responce from the atmel
	build_frame(PKT_ATMEL, g_rx, atmel_buf, sizeof(atmel_buf), FRM_INIT|FRM_NO_RX);
	// build the acks for both
	build_ack(g_rx, g_tmpbuf);

	err = izspi_write_packet(&izspi, out_buf, sizeof(out_buf));
	TEST_ASSERT_EQUAL_INT_MESSAGE(sizeof(out_buf), err,"Packet write was unsuccessful");

	izspi_xfer_frames(&izspi);
	err = izspi_read_packet(&izspi, inbuf);
	TEST_ASSERT_EQUAL_INT_MESSAGE(sizeof(atmel_buf),err,"Didn't read packet from atmel when no can rx set");
	TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(atmel_buf,inbuf,sizeof(atmel_buf),"Read in buffer from atmel didn't match when can rx wasn't set");

	reset_spi_simulator();

	// rebuild the ack packet with ack set
	build_frame(PKT_ATMEL, g_rx, NULL, 0, FRM_INIT);
	// build the acks for both
	build_ack(g_rx, g_tmpbuf);

	izspi_xfer_frames(&izspi);

	TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(g_tmpbuf,g_tx,FRM_SZ,"Didn't retransmit the packet again after the NAK from Atmel.");
}

void test_imx_retransmit_after_nak(void)
{
	int err;
	char out_buf[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE};
	// packt to compare against
	build_frame(PKT_IMX, g_tmpbuf, out_buf, sizeof(out_buf), FRM_INIT);
	// responce from the atmel
	build_frame(PKT_ATMEL, g_rx, NULL, 0, FRM_INIT);
	// build the acks for both
	build_ack(g_rx, g_tmpbuf);
	// net the ack to be a NAK
	g_rx[NDX_ACK] &= ~ACK_MASK;
	g_rx[NDX_ACK] |= ACK_NAK;

	err = izspi_write_packet(&izspi, out_buf, sizeof(out_buf));
	TEST_ASSERT_EQUAL_INT_MESSAGE(sizeof(out_buf), err,"Packet write was unsuccessful");

	izspi_xfer_frames(&izspi);
	reset_spi_simulator();

	// rebuild the ack packet with ack set
	build_frame(PKT_ATMEL, g_rx, NULL, 0, FRM_INIT);
	// build the acks for both
	build_ack(g_rx, g_tmpbuf);

	izspi_xfer_frames(&izspi);

	TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(g_tmpbuf,g_tx,FRM_SZ,"Didn't retransmit the packet again after the NAK from Atmel.");
}

void test_canrx_bit_not_set_when_rxbuf_full(void)
{
	char buf[] = {0x01, 0x02, 0x03};
	// completely fill the RXbuf
	struct cbuf *rxbuf = &izspi.rxbuf;
	while(cbuf_space(rxbuf)) {
		cbuf_copy_in(rxbuf, g_tmpbuf, 0, 1);
		cbuf_produce(rxbuf, 1);
	}

	// create a packet to rx
	build_frame(PKT_ATMEL, g_rx, buf, sizeof(buf), FRM_INIT);
	// create the model tx packet
	build_frame(PKT_IMX, g_tx, NULL, 0, FRM_INIT|FRM_NO_RX);
	build_ack(g_rx, g_tx);

	izspi_xfer_frames(&izspi);

	TEST_ASSERT_BITS_LOW_MESSAGE(HEADER_CAN_RX_BIT,g_tx[NDX_IMX_HEADER],"IMX didn't clear the can RX bit");
	/* also check that we NAKed the packet */
	TEST_ASSERT_BITS_MESSAGE(ACK_MASK, ACK_NAK,g_tx[NDX_ACK],"IMX didn't nak the Atmel data");
}
