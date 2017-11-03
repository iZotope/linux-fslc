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

#ifndef __IZCBUF_H
#define __IZCBUF_H

#include <linux/module.h>
#include <linux/circ_buf.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>

struct cbuf {
	char *buf;
	int head;
	int tail;
	int size;
	struct mutex producer_mutex;
	struct mutex consumer_mutex;
};

#define CBUF_CLAIM	(1)
#define CBUF_NO_CLAIM	(0)

/**
 * cbuf_cnt - get the count of bytes in a cbuf
 * @cbuf: pointer to cbuf to check count in
 *
 * Return: number of bytes in the cbuf
 */
static inline int cbuf_cnt(const struct cbuf *cbuf)
{
	return CIRC_CNT(cbuf->head, cbuf->tail, cbuf->size);
}

/**
 * cbuf_space - get the number of bytes free in a cbuf
 * @cbuf: pointer to cbuf to check space
 *
 * Return: number of bytes free in cbuf
 */
static inline int cbuf_space(const struct cbuf *cbuf)
{
	return CIRC_SPACE(cbuf->head, cbuf->tail, cbuf->size);
}

/**
 * cbuf_peek - peek a single byte within a cbuf
 * @cbuf: pointer to cbuf to peek into
 * @ndx: byte index relative to the tail to peek
 *
 * Return:
 *      the byte value at index 'ndx'
 *      -1 - if ndx > bytes in cbuf
 */
static inline int cbuf_peek(const struct cbuf *cbuf, int ndx)
{
	if (cbuf_cnt(cbuf) < ndx + 1) {
		return -1;
	}
	return cbuf->buf[(cbuf->tail + ndx) & (cbuf->size - 1)];
}

int cbuf_devm_init(struct cbuf *cbuf, struct device *dev, int size);
int cbuf_peek(const struct cbuf *cbuf, int ndx);
int cbuf_copy_in(struct cbuf *cbuf, const char *buf, int index, int count);
int cbuf_produce(struct cbuf *cbuf, int count);
int cbuf_copy_out(struct cbuf *cbuf, char *buf, int index, int count);
int cbuf_consume(struct cbuf *cbuf, int count);

#endif /* __IZCBUF_H */
