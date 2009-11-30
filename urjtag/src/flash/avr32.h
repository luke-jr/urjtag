/* ************ AVR32UC Flash support by brag ************* */
/* */

#ifndef URJ_SRC_AVR32_H
#define URJ_SRC_AVR32_H

#include <urjtag/types.h>
#include <urjtag/flash.h>

int urj_flash_avr32_autodetect(urj_bus_t *bus);
int urj_flash_avr32_flashmem(urj_bus_t *bus,FILE *f,uint32_t addr,int noverify);

#endif /* URJ_SRC_AVR32_H */
