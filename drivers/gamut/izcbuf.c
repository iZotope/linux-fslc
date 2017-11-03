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

#include "izcbuf.h"

/**
 * cbuf_devm_init - allocate a new circular buffer using device managed alloc
 * @cbuf: pointer to cbuf to initlize
 * @dev: pointer to device to attached managed memeory allocation to
 * @size: circular buffer size to allocate (must be power of 2 size)
 *
 * Return:
 *	0 = success
 *	-ENOMEM = no memory to allocate buffer
 */
int cbuf_devm_init(struct cbuf *cbuf, struct device *dev, int size)
{
	// TODO: would be good to enfore size is power of 2
	cbuf->size = size;
	cbuf->buf = devm_kzalloc(dev, cbuf->size, GFP_KERNEL);
	if (!cbuf->buf) {
		return -ENOMEM;
	}

	mutex_init(&cbuf->producer_mutex);
	mutex_init(&cbuf->consumer_mutex);

	return 0;
}

/**
 * cbuf_copy_in - copy into cbuf from a buffer
 * @cbuf: pointer to cbuf to copy into
 * @buf: pointer to buffer to copy from
 * @index: index to copy to
 * @count: number of bytes to copy
 *
 * Note that this should be mutex protected by the caller
 * This will not update the cbuf pointer, you must call cbuf_produce when
 * you are done copying in what you want.
 *
 * Return:
 *	0 = success
 *	-ENOSPC = free space in cbuf < count. No partial copy performed
 */
int cbuf_copy_in(struct cbuf *cbuf, const char *buf, int index, int count)
{
	// TODO: use the mutex!
	int cpysz;
	int offset_head;

	if (count <= 0) {
		return 0;
	}

	offset_head = (cbuf->head + index) & (cbuf->size - 1);

	if (count > CIRC_SPACE(offset_head, cbuf->tail, cbuf->size)) {
		return -ENOSPC;
	}

	if (count <= CIRC_SPACE_TO_END(offset_head, cbuf->tail, cbuf->size)) {
		cpysz = count;
	} else {
		cpysz = CIRC_SPACE_TO_END(offset_head, cbuf->tail, cbuf->size);
	}

	memcpy(&cbuf->buf[offset_head], buf, cpysz);
	if (cpysz < count) {
		memcpy(cbuf->buf, buf+cpysz, count-cpysz);
	}

	return 0;
}

/**
 * cbuf_produce - produces bytes from a circular buffer (updates the head)
 * @cbuf: pointer to circular buffer to consume from
 * @count: number of bytes to produce
 *
 * Return:
 *	0 = count bytes consumed
 *	-ENOSPC = free space in cbuf < count. cbuf pointres not updated
 */
int cbuf_produce(struct cbuf *cbuf, int count)
{
	if ( count <= 0) {
		return 0;
	}

	if (count > CIRC_SPACE(cbuf->head, cbuf->tail, cbuf->size)) {
		return -ENOSPC;
	}

	// TODO: understand why this fuction was used
	smp_store_release(&cbuf->head, (cbuf->head + count) & (cbuf->size - 1));
	return 0;
}

/**
 * cbuf_copy_out - copy from a cbuf into a userpsace buffer
 * @cbuf: pointer to cbuf to copy from
 * @buf: pointer to buffer to copy into, if NULL no copy performed
 * @index: index to copy from
 * @count: number of bytes to copy
 *
 * This should be mutex protected by the caller
 * This will not update the cbuf pointer, you must call cbuf_consume when
 * you are done copying what you want.
 *
 * Return:
 *	0 = success
 *	-EAGAIN = not enough bytes in cbuf to copy out, bo bytes are copied out.
 */
int cbuf_copy_out(struct cbuf *cbuf, char *buf, int index, int count)
{
	int cpysz;
	int offset_tail;

	if (count <= 0) {
		return 0;
	}

	if (count + index > CIRC_CNT(cbuf->head, cbuf->tail, cbuf->size)) {
		return -EAGAIN;
	}

	offset_tail = (cbuf->tail + index) & (cbuf->size - 1);

	if (count <= CIRC_CNT_TO_END(cbuf->head, offset_tail, cbuf->size)) {
		cpysz = count;
	} else {
		cpysz = CIRC_CNT_TO_END(cbuf->head, offset_tail, cbuf->size);
	}

	if (buf) {
		memcpy(buf, &cbuf->buf[offset_tail], cpysz);
		if (cpysz < count) {
			memcpy(buf+cpysz, cbuf->buf, count-cpysz);
		}
	}

	return 0;
}

/**
 * cbuf_consume - consume bytes from a circular buffer
 * @cbuf: pointer to circular buffer to consume from
 * @count: number of bytes to consume
 *
 * Return:
 *	0 = count bytes consumed
 *	-EAGAIN = bytes in cbuf < count. cbuf pointres not updated
 */
int cbuf_consume(struct cbuf *cbuf, int count)
{
	if ( count <= 0) {
		return 0;
	}

	if (count > CIRC_CNT(cbuf->head, cbuf->tail, cbuf->size)) {
		return -EAGAIN;
	}

	// TODO: understand why this fuction was used
	smp_store_release(&cbuf->tail, (cbuf->tail + count) & (cbuf->size - 1));
	return 0;
}
