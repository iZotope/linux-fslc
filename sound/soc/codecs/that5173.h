/*
 * That Corp that5173 Alsa SoC driver
 *
 * Copyright (C) 2016-2017, iZotope inc.
 *
 * Authors: Matthew Campbell <mcampbell@izotope.com>
 *          Jonah Petri <jpetri@izotope.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define THAT5173_CLEAR_GPO(n) (1<<(2*(n)))
#define THAT5173_SET_GPO(n)   (3<<(2*(n)))
#define THAT5173_GPO_MASK(n)  (3<<(2*(n)))

int that5173_set_gpo_values(struct snd_soc_codec *codec, u8* gpo_buf, unsigned gpo_buf_len);
