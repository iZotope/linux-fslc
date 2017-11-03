#ifndef __GAMUT_UI_MAIN_H
#define __GAMUT_UI_MAIN_H

int gamut_ui_spi_xfer(struct izspi *izspi, char *tx, char *rx, int size, int hold_ss);

#endif /* __GAMUT_UI_MAIN_H */
