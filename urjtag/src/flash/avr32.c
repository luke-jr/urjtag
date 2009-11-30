/* ************ AVR32UC Flash support by brag ************* */
/* */

#include <sysdep.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>     /* usleep */

#include <urjtag/log.h>
#include <urjtag/error.h>
#include <urjtag/flash.h>
#include <urjtag/bus.h>
#include <urjtag/part.h>
#include <urjtag/tap_register.h>

#include "flash.h"
#include "cfi.h"
#include "avr32.h"

typedef unsigned long DD;
typedef unsigned short DW;
typedef unsigned char DB;

#define FLASH_PBB 0x80000000
#define FLASHC 0xFFFE1400
#define FCR (FLASHC+0x0)
#define FCMD (FLASHC+0x4)
#define FSR (FLASHC+0x8)
#define FGPFRHI (FLASHC+0xc)
#define FGPFRLO (FLASHC+0x10)

#define FCMD_KEY 0xA5000000
#define FCMD_NOP 0
#define FCMD_WP 1
#define FCMD_EP 2
#define FCMD_CPB 3
#define FCMD_LP 4
#define FCMD_UP 5
#define FCMD_EA 6
#define FCMD_WGPB 7
#define FCMD_EGPB 8
#define FCMD_SSB 9
#define FCMD_PGPFB 10
#define FCMD_EAGPF 11
#define FCMD_QPR 12
#define FCMD_WUP 13
#define FCMD_EUP 14
#define FCMD_QPRUP 15

struct avr32parts{
	DD id;
	char pn[32];
	DD pages;
	DD bytes;
};

static const struct avr32parts g_avr32parts[]={
	{0x01EDC03F,{"AT32UC3A0512"},1024,512*1024},
	{0x01EDD03F,{"AT32UC3A1512"},1024,512*1024},
	{0x01EDF03F,{"AT32UC3A0256"},512,256*1024},
	{0x01EE003F,{"AT32UC3A1256"},512,256*1024},
	{0x01EE403F,{"AT32UC3B0256"},512,256*1024},
	{0x01EE503F,{"AT32UC3B1256"},512,256*1024},
	{0x01EE203F,{"AT32UC3A0128"},256,128*1024},
	{0x01EE303F,{"AT32UC3A1128"},256,128*1024},
	{0x01EE603F,{"AT32UC3B0128"},256,128*1024},
	{0x01EE903F,{"AT32UC3B1128"},256,128*1024},
	{0x01EEA03F,{"AT32UC3B064"},128,64*1024},
	{0x01EEB03F,{"AT32UC3B164"},128,64*1024},
	{0,{""}}
};
const DD g_flashsz[]={32*1024,64*1024,128*1024,256*1024,384*1024,512*1024,768*1024,1024*1024};

struct avr32finfo{
	const struct avr32parts *part;
	DD fsize;
};
static struct avr32finfo g_avr32finfo;

int urj_flash_avr32_autodetect(urj_bus_t *bus){ //1 - detected, 0 - otherwise
	DD id;
	int i;
	id=urj_tap_register_get_value(bus->part->id);
	id&=0x0FFFFFFF;
	i=0;
	while(g_avr32parts[i].pn[0]){
		if(g_avr32parts[i].id==id){
			g_avr32finfo.part=&g_avr32parts[i];
			id=URJ_BUS_READ(bus,FSR)>>13;
			g_avr32finfo.fsize=g_flashsz[id];
			return 1;
		}
		i++;
	}
	return 0;
}

static void avr32f_print_info(urj_bus_t *bus){
	int i;
	i=URJ_BUS_READ(bus,FSR)>>13;
	g_avr32finfo.fsize=g_flashsz[i];
	urj_log(URJ_LOG_LEVEL_NORMAL,"Flash on %s\nSize(detected): %lu\nPages: %lu\nPage size: %u words\n",
		g_avr32finfo.part->pn,g_avr32finfo.fsize,g_avr32finfo.part->pages,128);
}

#define FLASHCMD_TO 100
#define FLASHCMD_TRIES 1000
static int avr32f_EP(urj_bus_t *bus,DD page){
	DD reg;
	int ec;
	ec=FLASHCMD_TRIES;
	while(1){
		reg=URJ_BUS_READ(bus,FSR);
		if(reg&1)break;
		usleep(FLASHCMD_TO);
		if(ec)ec--;else return URJ_STATUS_FAIL;
	}
	reg=FCMD_KEY|(page<<8)|FCMD_EP;
	URJ_BUS_WRITE(bus,FCMD,reg);
	reg=URJ_BUS_READ(bus,FSR);
	if(reg&0x08)return URJ_STATUS_FAIL;
	return URJ_STATUS_OK;
}

static int avr32f_CPB(urj_bus_t *bus){
	DD reg;
	int ec;
	ec=FLASHCMD_TRIES;
	while(1){
		reg=URJ_BUS_READ(bus,FSR);
		if(reg&1)break;
		usleep(FLASHCMD_TO);
		if(ec)ec--;else return URJ_STATUS_FAIL;
	}
	reg=FCMD_KEY|FCMD_CPB;
	URJ_BUS_WRITE(bus,FCMD,reg);
	reg=URJ_BUS_READ(bus,FSR);
	if(reg&0x08)return URJ_STATUS_FAIL;
	return URJ_STATUS_OK;
}

static int avr32f_WP(urj_bus_t *bus,DD page){
	DD reg;
	int ec;
	ec=FLASHCMD_TRIES;
	while(1){
		reg=URJ_BUS_READ(bus,FSR);
		if(reg&1)break;
		usleep(FLASHCMD_TO);
		if(ec)ec--;else return URJ_STATUS_FAIL;
	}
	reg=FCMD_KEY|(page<<8)|FCMD_WP;
	URJ_BUS_WRITE(bus,FCMD,reg);
	reg=URJ_BUS_READ(bus,FSR);
	if(reg&0x08)return URJ_STATUS_FAIL;
	return URJ_STATUS_OK;
}

static int avr32f_writepb(urj_bus_t *bus,uint32_t *buff){
	DD addr;
	int i;
	addr=FLASH_PBB;
	for(i=0;i<128;i++){
		URJ_BUS_WRITE(bus,addr,buff[i]);
		addr+=4;
	}
	return URJ_STATUS_OK;
}

static int avr32_read(urj_bus_t *bus,DD addr,uint32_t *buff,int words){
	addr&=~(4-1);
	while(words){
		*buff=URJ_BUS_READ(bus,addr);
		words--;
		addr+=4;
		buff++;
	}
	return URJ_STATUS_OK;
}

int urj_flash_avr32_flashmem(urj_bus_t *bus,FILE *f,uint32_t addr,int noverify){
	uint32_t buff[128],buff1[128];
	DD n,pn,bc,a;
	
	avr32f_print_info(bus);
	if(addr&0x1FF){
		urj_log(URJ_LOG_LEVEL_NORMAL,_("Warning: page offset ignored\n"));
	}
	urj_log(URJ_LOG_LEVEL_NORMAL,_("burning...\n"));
	a=addr&0x0FFFF200;
	pn=a>>9;bc=0;
	if(feof(f))return URJ_STATUS_FAIL;
	while(1){
		n=fread(buff,1,sizeof(buff),f);
		if(!n)break;
		if(n<sizeof(buff))memset(((DB*)buff)+n,0xFF,sizeof(buff)-n);
		a=(pn<<9)|FLASH_PBB;
		urj_log(URJ_LOG_LEVEL_NORMAL,_("addr: 0x%08lX page: %lu\r"),a,pn);
		if(avr32f_EP(bus,pn)!=URJ_STATUS_OK){
			urj_error_set(URJ_ERROR_FLASH_PROGRAM,_("\n erase error\n"));
			return URJ_STATUS_FAIL;
		}
		if(avr32f_CPB(bus)!=URJ_STATUS_OK){
			urj_error_set(URJ_ERROR_FLASH_PROGRAM,_("\n CPB error\n"));
			return URJ_STATUS_FAIL;
		}
		avr32f_writepb(bus,buff);
		if(avr32f_WP(bus,pn)!=URJ_STATUS_OK){
			urj_error_set(URJ_ERROR_FLASH_PROGRAM,_("\nwrite error\n"));
			return URJ_STATUS_FAIL;
		}
		bc+=n;
		if(n<sizeof(buff)||feof(f)||pn>=g_avr32finfo.part->pages-1)break;
		pn++;
	}
	
	if(noverify){
		urj_log (URJ_LOG_LEVEL_NORMAL, _("\nverify skipped\n"));
		return URJ_STATUS_OK;
	}
	
	fseek(f,0,SEEK_SET);
	urj_log(URJ_LOG_LEVEL_NORMAL,_("\nverifying...\n"));
	pn=0;bc=0;
	while(1){
		n=fread(buff,1,sizeof(buff),f);
		if(!n)break;
		a=(pn<<9)|FLASH_PBB;
		urj_log(URJ_LOG_LEVEL_NORMAL,_("addr: 0x%08lX page: %lu\r"),a,pn);
		avr32_read(bus,a,buff1,128);
		if(memcmp(buff,buff1,n)){
			urj_error_set(URJ_ERROR_FLASH_PROGRAM,_("\verification failed\n"));
			return URJ_STATUS_FAIL;
		}
		bc+=n;
		if(n<sizeof(buff)||feof(f)||pn>=g_avr32finfo.part->pages-1)break;
		pn++;
	}
	
	urj_log(URJ_LOG_LEVEL_NORMAL,_("\n%lu bytes total\nDone.\n"),bc);
	return URJ_STATUS_OK;
}
