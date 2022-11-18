/* -------------------------------------------------------------------------
   scd_mars.c  scd driver for Realtek Neptune/Mars
   -------------------------------------------------------------------------
    Copyright (C) 2008 Kevin Wang <kevin_wang@realtek.com.tw>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
----------------------------------------------------------------------------
Update List :
----------------------------------------------------------------------------
    1.0     |   20090312    |   Kevin Wang  | 1) create phase
----------------------------------------------------------------------------*/
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <asm/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <soc/realtek/rtk_chip.h>
#include "scd_mars_priv.h"
#include "scd_mars_reg.h"

MODULE_LICENSE("GPL");

#define MIN(a,b)  (((a)<(b))?(a):(b))

extern MARS_DTS_INFO_T dts_info;

#define SA_SHIRQ IRQF_SHARED

/* Basic operation */

#if 1

#define rtd_inb(offset)             (*(volatile unsigned char  *)(offset))
#define rtd_inw(offset)             (*(volatile unsigned short *)(offset))
#define rtd_inl(offset)             readl(offset)
#define rtd_outb(offset,val)        (*(volatile unsigned char  *)(offset) = val)
#define rtd_outw(offset,val)        (*(volatile unsigned short *)(offset) = val)
#define rtd_outl(offset,val)        writel(val, offset)

#else

#define rtd_inb(offset)             (*(volatile unsigned char  *)(offset))
#define rtd_inw(offset)             (*(volatile unsigned short *)(offset))
#define rtd_inl(offset)             (*(volatile unsigned long  *)(offset))
#define rtd_outb(offset,val)        do { (*(volatile unsigned char  *)(offset) = val); printk("  @write reg=%08x, val=%08x\n", offset, (unsigned long) val); }while(0)
#define rtd_outw(offset,val)        do { (*(volatile unsigned short *)(offset) = val); printk("  @write reg=%08x, val=%08x\n", offset, (unsigned long) val); }while(0)
#define rtd_outl(offset,val)        do { (*(volatile unsigned long  *)(offset) = val); printk("  @write reg=%08x, val=%08x\n", offset, (unsigned long) val); }while(0)

#endif

#define DBG_CHAR(x)                 rtd_outl(0xb8007800, x)


static const unsigned long          SC_BASE[]={SC_BASE0};
static const unsigned long          MIS_SC_ISR[]={MIS_SC0_INT};

#define SET_MIS_ISR(misc, val)      rtd_outl(misc, val)
#define SET_SCFP(base, val)         rtd_outl(base + SCFP,    val)
#define SET_SCCR(base, val)         rtd_outl(base + SCCR,    val)
#define SET_SCPCR(base, val)        rtd_outl(base + SCPCR,   val)
#define SET_SC_TXFIFO(base, val)    rtd_outl(base + SCTXFIFO,val)
#define SET_SCFCR(base, val)        rtd_outl(base + SCFCR,   val)
#define SET_SCIRSR(base, val)       rtd_outl(base + SCIRSR,  val)
#define SET_SCIRER(base, val)       rtd_outl(base + SCIRER,  val)


#define GET_MIS_ISR(misc)           rtd_inl(misc)
#define GET_SCFP(base)              rtd_inl(base + SCFP)
#define GET_SCCR(base)              rtd_inl(base + SCCR)
#define GET_SCPCR(base)             rtd_inl(base + SCPCR)
#define GET_SC_TXFIFO(base)         rtd_inl(base + SCTXFIFO)  // for bit8:TX_FIFO_FULL
#define GET_SC_TXLENR(base)         rtd_inl(base + SCTXLENR)
#define GET_SC_RXFIFO(base)         rtd_inl(base + SCRXFIFO)
#define GET_SC_RXLENR(base)         rtd_inl(base + SCRXLENR)
#define GET_SCFCR(base)             rtd_inl(base + SCFCR)
#define GET_SCIRSR(base)            rtd_inl(base + SCIRSR)
#define GET_SCIRER(base)            rtd_inl(base + SCIRER)

#define MAX_SC_CLK                   8000000
#define MIN_SC_CLK                   1000000

#define TX_RX_DEPTH		40

#ifdef ISR_POLLING
#define ISR_POLLING_INTERVAL        (HZ)
static void mars_scd_timer(unsigned long arg);
#endif

extern char *parse_token(const char *parsed_string, const char *token);

/// clk/etu to lookup table
#define ENABLE_CLK_ETU_LOOKUP_TABLE

#if defined(ENABLE_CLK_ETU_LOOKUP_TABLE)
typedef enum {
	SC_CLK_9000000 = 0,
	SC_CLK_4500000,
	SC_CLK_3000000,
	SC_CLK_6750000,
	SC_CLK_13500000,
	// add new supported clk above.
	SC_CLK_LIST_MAX
} SC_CLK_LIST_e;

//based on 7816-3 TA1, F/D
// It dosn't means driver support etu setting if enable these etu eum....
typedef enum {
	//SC_ETU_8,
	//SC_ETU_12,
	SC_ETU_16,
	SC_ETU_20,
	SC_ETU_23_25,
	//SC_ETU_24,
	SC_ETU_31,
	SC_ETU_32,
	SC_ETU_46_5,
	//SC_ETU_48,
	//SC_ETU_62,
	//SC_ETU_64,
	SC_ETU_93,
	//SC_ETU_96,
	//SC_ETU_128,
	SC_ETU_186,
	//SC_ETU_192,
	//SC_ETU_256,
	//SC_ETU_279,
	SC_ETU_372,
	//SC_ETU_384,
	//SC_ETU_465,
	//SC_ETU_512,
	//SC_ETU_558,
	//SC_ETU_744,
	//SC_ETU_768,
	//SC_ETU_930,
	//SC_ETU_1024,
	//SC_ETU_1116,
	//SC_ETU_1488,
	//SC_ETU_1536,
	//SC_ETU_1860,
	//SC_ETU_2048,
	// add new supported etu above.
	SC_ETU_LIST_MAX,
} SC_ETU_LIST_e;

#define ETU_16		16
#define ETU_20		20
#define ETU_23_25	23 /* 23.25 */
#define ETU_31		31
#define ETU_32		32
#define ETU_46_5	46 /* 46.5 */
#define ETU_93		93
#define ETU_186		186
#define ETU_372		372

typedef enum
{
	MAR_SCD_BAUDDIV2_UNSUPPORTED = 0,
	MAR_SCD_BAUDDIV2_31 = 31,  // HW IP dependent
	MAR_SCD_BAUDDIV2_32 = 32,
	MAR_SCD_BAUDDIV2_39 = 39,
} MAR_SCD_BAUDDIV2_LIST_e;

typedef struct
{
	unsigned int		src_clk;
	unsigned char           pre_clock_div;      // pre_clock divider
	unsigned char           clock_div;          // clock divider
	unsigned char           baud_div1;
	MAR_SCD_BAUDDIV2_LIST_e e_baud_div2;
} MAR_SCD_ETU_t;

typedef struct
{
	MAR_SCD_ETU_t           etu_list[SC_ETU_LIST_MAX];
} MAR_SCD_CLK_LIST_t;

#define GET_BAUDDIV2_HW_VALUE(x, e)  \
	do { \
		switch((e)) \
		{ \
			case MAR_SCD_BAUDDIV2_31: (x) = 0x0; break; \
			case MAR_SCD_BAUDDIV2_32: (x) = 0x1; break; \
			case MAR_SCD_BAUDDIV2_39: (x) = 0x10; break; \
			default: SC_WARNING("unsupported BD2 (%d)\n", (e) ); \
		} \
	} while(0)

static MAR_SCD_CLK_LIST_t clk_info[] =
{
	[SC_CLK_13500000] = {
		.etu_list = {
			[SC_ETU_16]   = {.src_clk= SYSTEM_CLK_216M, .pre_clock_div= 2, .clock_div= 8, .baud_div1= 4, .e_baud_div2= MAR_SCD_BAUDDIV2_32},
			[SC_ETU_20]   = {.src_clk= SYSTEM_CLK_216M, .pre_clock_div= 2, .clock_div= 8, .baud_div1= 5, .e_baud_div2= MAR_SCD_BAUDDIV2_32},
			[SC_ETU_23_25]= {.src_clk= SYSTEM_CLK_216M, .pre_clock_div= 4, .clock_div= 4, .baud_div1= 3, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
			[SC_ETU_31]   = {.src_clk= SYSTEM_CLK_216M, .pre_clock_div=16, .clock_div= 1, .baud_div1= 1, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
			[SC_ETU_32]   = {.src_clk= SYSTEM_CLK_216M, .pre_clock_div=16, .clock_div= 1, .baud_div1= 1, .e_baud_div2= MAR_SCD_BAUDDIV2_32},
			[SC_ETU_46_5] = {.src_clk= SYSTEM_CLK_216M, .pre_clock_div= 2, .clock_div= 8, .baud_div1=12, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
			[SC_ETU_93]   = {.src_clk= SYSTEM_CLK_216M, .pre_clock_div= 2, .clock_div= 8, .baud_div1=24, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
			[SC_ETU_186]  = {.src_clk= SYSTEM_CLK_216M, .pre_clock_div=16, .clock_div= 1, .baud_div1= 6, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
			[SC_ETU_372]  = {.src_clk= SYSTEM_CLK_216M, .pre_clock_div=16, .clock_div= 1, .baud_div1=12, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
		},
	},
	[SC_CLK_9000000] = {
		.etu_list = {
			[SC_ETU_16]   = {.src_clk= SYSTEM_CLK_216M, .pre_clock_div=12, .clock_div= 2, .baud_div1= 1, .e_baud_div2= MAR_SCD_BAUDDIV2_32},
			[SC_ETU_20]   = {.src_clk= SYSTEM_CLK_216M, .pre_clock_div= 3, .clock_div= 8, .baud_div1= 5, .e_baud_div2= MAR_SCD_BAUDDIV2_32},
			[SC_ETU_23_25]= {.src_clk= SYSTEM_CLK_216M, .pre_clock_div= 2, .clock_div=12, .baud_div1= 9, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
			[SC_ETU_31]   = {.src_clk= SYSTEM_CLK_27M,  .pre_clock_div= 3, .clock_div= 1, .baud_div1= 1, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
			[SC_ETU_32]   = {.src_clk= SYSTEM_CLK_27M,  .pre_clock_div= 3, .clock_div= 1, .baud_div1= 1, .e_baud_div2= MAR_SCD_BAUDDIV2_32},
			[SC_ETU_46_5] = {.src_clk= SYSTEM_CLK_216M, .pre_clock_div= 4, .clock_div= 6, .baud_div1= 9, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
			[SC_ETU_93]   = {.src_clk= SYSTEM_CLK_27M,  .pre_clock_div= 3, .clock_div= 1, .baud_div1= 3, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
			[SC_ETU_186]  = {.src_clk= SYSTEM_CLK_27M,  .pre_clock_div= 3, .clock_div= 1, .baud_div1= 6, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
			[SC_ETU_372]  = {.src_clk= SYSTEM_CLK_27M,  .pre_clock_div= 3, .clock_div= 1, .baud_div1=12, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
		},
	},
	[SC_CLK_6750000] = {
		.etu_list = {
			[SC_ETU_16]   = {.src_clk= SYSTEM_CLK_216M, .pre_clock_div=16, .clock_div= 2, .baud_div1= 2, .e_baud_div2= MAR_SCD_BAUDDIV2_32},
			[SC_ETU_20]   = {.src_clk= SYSTEM_CLK_216M, .pre_clock_div= 4, .clock_div= 8, .baud_div1= 5, .e_baud_div2= MAR_SCD_BAUDDIV2_32},
			[SC_ETU_23_25]= {.src_clk= SYSTEM_CLK_216M, .pre_clock_div= 2, .clock_div=16, .baud_div1=12, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
			[SC_ETU_31]   = {.src_clk= SYSTEM_CLK_216M, .pre_clock_div=32, .clock_div= 1, .baud_div1= 1, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
			[SC_ETU_32]   = {.src_clk= SYSTEM_CLK_216M, .pre_clock_div=32, .clock_div= 1, .baud_div1= 1, .e_baud_div2= MAR_SCD_BAUDDIV2_32},
			[SC_ETU_46_5] = {.src_clk= SYSTEM_CLK_216M, .pre_clock_div= 4, .clock_div= 8, .baud_div1=12, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
			[SC_ETU_93]   = {.src_clk= SYSTEM_CLK_216M, .pre_clock_div= 4, .clock_div= 8, .baud_div1=24, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
			[SC_ETU_186]  = {.src_clk= SYSTEM_CLK_216M, .pre_clock_div=32, .clock_div= 1, .baud_div1= 6, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
			[SC_ETU_372]  = {.src_clk= SYSTEM_CLK_216M, .pre_clock_div=32, .clock_div= 1, .baud_div1=12, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
		},
	},
	[SC_CLK_4500000] = {
		.etu_list = {
			[SC_ETU_16]   = {.src_clk= SYSTEM_CLK_27M,  .pre_clock_div= 3, .clock_div= 2, .baud_div1= 1, .e_baud_div2= MAR_SCD_BAUDDIV2_32},
			[SC_ETU_20]   = {.src_clk= SYSTEM_CLK_216M, .pre_clock_div= 6, .clock_div= 8, .baud_div1= 5, .e_baud_div2= MAR_SCD_BAUDDIV2_32},
			[SC_ETU_23_25]= {.src_clk= SYSTEM_CLK_216M, .pre_clock_div= 2, .clock_div=24, .baud_div1=18, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
			[SC_ETU_31]   = {.src_clk= SYSTEM_CLK_27M,  .pre_clock_div= 6, .clock_div= 1, .baud_div1= 1, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
			[SC_ETU_32]   = {.src_clk= SYSTEM_CLK_27M,  .pre_clock_div= 6, .clock_div= 1, .baud_div1= 1, .e_baud_div2= MAR_SCD_BAUDDIV2_32},
			[SC_ETU_46_5] = {.src_clk= SYSTEM_CLK_27M,  .pre_clock_div= 3, .clock_div= 2, .baud_div1= 3, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
			[SC_ETU_93]   = {.src_clk= SYSTEM_CLK_27M,  .pre_clock_div= 6, .clock_div= 1, .baud_div1= 3, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
			[SC_ETU_186]  = {.src_clk= SYSTEM_CLK_27M,  .pre_clock_div= 6, .clock_div= 1, .baud_div1= 6, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
			[SC_ETU_372]  = {.src_clk= SYSTEM_CLK_27M,  .pre_clock_div= 6, .clock_div= 1, .baud_div1=12, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
		},
	},
	[SC_CLK_3000000] = {
		.etu_list = {
			[SC_ETU_16]   = {.src_clk= SYSTEM_CLK_216M, .pre_clock_div=36, .clock_div= 2, .baud_div1= 1, .e_baud_div2= MAR_SCD_BAUDDIV2_32},
			[SC_ETU_20]   = {.src_clk= SYSTEM_CLK_216M, .pre_clock_div= 9, .clock_div= 8, .baud_div1= 5, .e_baud_div2= MAR_SCD_BAUDDIV2_32},
			[SC_ETU_23_25]= {.src_clk= SYSTEM_CLK_216M, .pre_clock_div= 5, .clock_div=12, .baud_div1= 9, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
			[SC_ETU_31]   = {.src_clk= SYSTEM_CLK_27M,  .pre_clock_div= 9, .clock_div= 1, .baud_div1= 1, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
			[SC_ETU_32]   = {.src_clk= SYSTEM_CLK_27M,  .pre_clock_div= 9, .clock_div= 1, .baud_div1= 1, .e_baud_div2= MAR_SCD_BAUDDIV2_32},
			[SC_ETU_46_5] = {.src_clk= SYSTEM_CLK_216M, .pre_clock_div=36, .clock_div= 2, .baud_div1= 3, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
			[SC_ETU_93]   = {.src_clk= SYSTEM_CLK_216M, .pre_clock_div=36, .clock_div= 2, .baud_div1= 6, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
			[SC_ETU_186]  = {.src_clk= SYSTEM_CLK_216M, .pre_clock_div=36, .clock_div= 2, .baud_div1=12, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
			[SC_ETU_372]  = {.src_clk= SYSTEM_CLK_27M,  .pre_clock_div= 9, .clock_div= 1, .baud_div1=12, .e_baud_div2= MAR_SCD_BAUDDIV2_31},
		},
	},
};

// default CLK:3M, 372etu. should be synced in table
#define SCD_MARS_DEFAULT_CLK               (3000000)
#define SCD_MARS_DEFAULT_ETU               (372)
#define SCD_MARS_DEFAULT_PRE_CLKDIV        (9)
#define SCD_MARS_DEFAULT_CLKDIV            (1)
#define SCD_MARS_DEFAULT_BAUDDIV1          (12)
#define SCD_MARS_DEFAULT_BAUDDIV2_e        (MAR_SCD_BAUDDIV2_31)

#define SCD_MARS_DEFAULT_EXTRA_GI          (0)  // 7816-3, N=0
#define SCD_MARS_DEFAULT_CWI               (13) // 7816-3
#define SCD_MARS_DEFAULT_BWI               (4)  // 7816-3
#define SCD_MARS_DEFAULT_BGT               (22) // 7816-3
#define SCD_MARS_DEFAULT_PROTOCOL	   (0)  // T=0

static void mars_scd_set_system_clk(mars_scd* p_this)
{
	clk_set_rate(p_this->clk_sel, p_this->sys_clk);
}

static int mars_scd_set_clock_by_table(mars_scd* p_this, unsigned long clk)
{
	unsigned long val;
	unsigned long bd2 = 0;
	SC_ETU_LIST_e eETU;
	MAR_SCD_ETU_t *pCurETU = NULL;

	mars_scd_get_etu(p_this, &val);

	switch(val) {
	case 16:
		eETU = SC_ETU_16;
		break;
	case 20:
		eETU = SC_ETU_20;
		break;
	case 23:
		eETU = SC_ETU_23_25;
		break;
	case 31:
		eETU = SC_ETU_31;
		break;
	case 32:
		eETU = SC_ETU_32;
		break;
	case 46:
		eETU = SC_ETU_46_5;
		break;
	case 93:
		eETU = SC_ETU_93;
		break;
	case 186:
		eETU = SC_ETU_186;
		break;
	case 372:
	default:    // default to 372
		SC_WARNING("unsupport etu(%lu) in table, use default\n", val);
		eETU = SC_ETU_372;
		break;
	}

	switch(clk) {
	case 9000000:
		pCurETU = &(clk_info[SC_CLK_9000000].etu_list[eETU]);
		break;
	case 4500000:
		pCurETU = &(clk_info[SC_CLK_4500000].etu_list[eETU]);
		break;
	case 3000000:  // SCD_MARS_DEFAULT_CLK
		pCurETU = &(clk_info[SC_CLK_3000000].etu_list[eETU]);
		break;
	case 6750000:
		pCurETU = &(clk_info[SC_CLK_6750000].etu_list[eETU]);
		break;
	case 13500000:
		pCurETU = &(clk_info[SC_CLK_13500000].etu_list[eETU]);
		break;
	default:
		SC_WARNING("Unsupported sc_clk(%lu), do nothing\n", clk);
		return -1;
	}

	// make sure whether this etu is supported....
	if(MAR_SCD_BAUDDIV2_UNSUPPORTED == pCurETU->e_baud_div2) {
		SC_WARNING("Unsupported in (clk=%lu, etu=%lu), do nothing\n", clk, val);
		return -1;
	}

	p_this->pre_clock_div = pCurETU->pre_clock_div;
	p_this->clock_div     = pCurETU->clock_div;
	p_this->baud_div1     = pCurETU->baud_div1;
	p_this->baud_div2     = pCurETU->e_baud_div2;
	GET_BAUDDIV2_HW_VALUE(bd2, pCurETU->e_baud_div2);

	p_this->sys_clk = pCurETU->src_clk;
	mars_scd_set_system_clk(p_this);

	val = GET_SCFP(p_this->base) & ~SC_PRE_CLKDIV_MASK & ~SC_CLKDIV_MASK & ~SC_BAUDDIV_MASK;
	val |= ( SC_PRE_CLKDIV(p_this->pre_clock_div) |
		SC_CLKDIV(p_this->clock_div) |
		SC_BAUDDIV2(bd2) |
		SC_BAUDDIV1(p_this->baud_div1) );

	SC_INFO("clk(%lu), (0x%08x)->(0x%08lx), (%d, %d, %lu, %lu)\n",
		clk, GET_SCFP(p_this->base), val,
		p_this->pre_clock_div, p_this->clock_div, p_this->baud_div1, p_this->baud_div2);

	SET_SCFP(p_this->base, val);

	return 0;
}

static int mars_scd_set_etu_by_table(mars_scd* p_this, unsigned long etu)
{
	unsigned long clk;
	unsigned long val;
	unsigned long bd2 = 0;
	MAR_SCD_CLK_LIST_t *pCurClk = NULL;
	MAR_SCD_ETU_t      *pCurETU = NULL;

	mars_scd_get_clock(p_this, &clk);
	switch(clk) {
	case 9000000:
		pCurClk = &(clk_info[SC_CLK_9000000]);
		break;
	case 4500000:
		pCurClk = &(clk_info[SC_CLK_4500000]);
		break;
	case 3000000:  // SCD_MARS_DEFAULT_CLK
		pCurClk = &(clk_info[SC_CLK_3000000]);
		break;
	case 6750000:
		pCurClk = &(clk_info[SC_CLK_6750000]);
                break;
	case 13500000:
                pCurClk = &(clk_info[SC_CLK_13500000]);
                break;
	default:
		SC_WARNING("unsupported clk(%lu), do nothing!\n", clk);
		return -1;
	}

	switch(etu) {
	case ETU_16:
		pCurETU = &(pCurClk->etu_list[SC_ETU_16]);
		break;
	case ETU_20:
		pCurETU = &(pCurClk->etu_list[SC_ETU_20]);
		break;
	case ETU_23_25:
		pCurETU = &(pCurClk->etu_list[SC_ETU_23_25]);
		break;
	case ETU_31:
		pCurETU = &(pCurClk->etu_list[SC_ETU_31]);
		break;
	case ETU_32:
		pCurETU = &(pCurClk->etu_list[SC_ETU_32]);
		break;
	case ETU_46_5:
		pCurETU = &(pCurClk->etu_list[SC_ETU_46_5]);
		break;
	case ETU_93:
		pCurETU = &(pCurClk->etu_list[SC_ETU_93]);
		break;
	case ETU_186:
		pCurETU = &(pCurClk->etu_list[SC_ETU_186]);
		break;
	case ETU_372:
		pCurETU = &(pCurClk->etu_list[SC_ETU_372]);
		break;
	default:
		SC_WARNING("unsupported etu(%lu), do nothing!\n", etu);
		return -2;
	}

	// make sure whether this etu is supported....
	if(MAR_SCD_BAUDDIV2_UNSUPPORTED == pCurETU->e_baud_div2) {
		SC_WARNING("Unsupported in (clk=%lu, etu=%lu), do nothing\n", clk, etu);
		return -2;
	}

	p_this->pre_clock_div = pCurETU->pre_clock_div;
	p_this->clock_div     = pCurETU->clock_div;
	p_this->baud_div1     = pCurETU->baud_div1;
	p_this->baud_div2     = pCurETU->e_baud_div2;
	GET_BAUDDIV2_HW_VALUE(bd2, pCurETU->e_baud_div2);

	p_this->sys_clk = pCurETU->src_clk;
        mars_scd_set_system_clk(p_this);

	val = GET_SCFP(p_this->base) & ~SC_PRE_CLKDIV_MASK & ~SC_CLKDIV_MASK & ~SC_BAUDDIV_MASK;
	val |= ( SC_PRE_CLKDIV(p_this->pre_clock_div) |
		SC_CLKDIV(p_this->clock_div) |
		SC_BAUDDIV2(bd2) |
		SC_BAUDDIV1(p_this->baud_div1) );

	SC_INFO("clk(%lu), etu(%lu), (0x%08x)->(0x%08lx), (%d, %d, %lu, %lu)\n",
		p_this->sys_clk, etu, GET_SCFP(p_this->base), val,
		p_this->pre_clock_div, p_this->clock_div, p_this->baud_div1, p_this->baud_div2);

	SET_SCFP(p_this->base, val);

	return 0;
}
#endif //ENABLE_CLK_ETU_LOOKUP_TABLE

#define GET_SCPU_90K_TIMER(base) (rtd_inl(base+MIS_SCPU_CLK_90K_HI)<<32)|rtd_inl(base+MIS_SCPU_CLK_90K_LO)

#define WAIT_ATR_TIMEOUT                        (HZ)
#define MARS_SCD_POWER_ON_INTERVAL_us   (2720)    // MARS_SCD_POWER_ENABLE(1) until actual Vcc Hi, by oscilloscope
#define MARS_SCD_RESET_PULSE_WIDTH_us   (6505)    // Reset Pulse Width of TDA8035, from Vcc Hi to Rst Hi by cardsim
#define MARS_SCD_DEFAULT_ATR_ACTIVATION_TIMEOUT_clk (40000)   // Maximum wait Time between RST and TS (unit: Clock), spec defined.
#define MARS_SCD_DEFAULT_1BYTE_2_ATR_TO_CHECK_us    (88)  // overhead from ATR 1st byte to atr timeout check, by oscilloscope.
#define MARS_SCD_DEFAULT_ATR_CWT_etu        (9600)    // 9600 ETU
#define system_clock_to_us(x)   ((x) * 11)

static unsigned long inline mars_scd_get_system_clock_us(void)
{
	void __iomem *misc = ioremap(MIS_SCPU_CLK_90K_BASE, 0x100);

	//unsigned long long misc_90k = (rtd_inl(misc+MIS_SCPU_CLK_90K_HI)<<32) | rtd_inl(misc+MIS_SCPU_CLK_90K_LO);
	unsigned long long misc_90k = GET_SCPU_90K_TIMER(misc);
	iounmap(misc);

	return (unsigned long) system_clock_to_us(misc_90k); // 90 KHz -> us
}

/*------------------------------------------------------------------
 * Func : mars_scd_clk_to_us
 *
 * Desc : convert SC clock to us (not system clk)
 *
 * Parm : p_this : handle of mars smc
 *                 SC clk : given clk
 *
 * Retn : us
*------------------------------------------------------------------*/
static unsigned long inline mars_scd_clk_to_us(mars_scd* p_this, unsigned long clk)
{
	unsigned long clock;
	mars_scd_get_clock(p_this, &clock);
	return (clk * 1000) / (clock/1000);
}

/*------------------------------------------------------------------
 * Func : mars_scd_us_to_clock
 *
 * Desc : convert us to SC clk
 *
 * Parm : p_this : handle of mars smc
 *                 us : given usec
 *
 * Retn : number of SC clk
*------------------------------------------------------------------*/
static unsigned long inline mars_scd_us_to_clock(mars_scd* p_this, unsigned long us)
{
	unsigned long clock;
	mars_scd_get_clock(p_this, &clock);
	return (us * (clock/1000))/1000;
}

/*------------------------------------------------------------------
 * Func : mars_scd_etu_to_us
 *
 * Desc : convert etu to us
 *
 * Parm : p_this : handle of mars smc
 *                etu    : value of etu
 *
 * Retn : us
 *------------------------------------------------------------------*/
static unsigned long inline mars_scd_etu_to_us(mars_scd* p_this, unsigned long etu)
{
	unsigned long clock;
	unsigned long etu_per_clk;
	unsigned long ret;

	mars_scd_get_clock(p_this, &clock);
	mars_scd_get_etu(p_this, &etu_per_clk);
	ret = (etu * etu_per_clk * 1000) / (clock / 1000);
	SC_INFO("oo>> sc clk=%lu, e=%lu, %lu etu=%lu us\n", clock, etu_per_clk, etu, ret);
	//return (etu * etu_per_clk * 1000) / (clock / 1000);
	return ret;
}

/*------------------------------------------------------------------
 * Func : mars_scd_us_to_etu
 *
 * Desc : convert us to etu
 *
 * Parm : p_this : handle of mars smc
 *                us    : value of etu
 *
 * Retn : number of ETU
*------------------------------------------------------------------*/
static unsigned long inline mars_scd_us_to_etu(mars_scd* p_this, unsigned long us)
{
	unsigned long clock;
	unsigned long etu_per_clk;
	unsigned long ret;

	mars_scd_get_clock(p_this, &clock);
	mars_scd_get_etu(p_this, &etu_per_clk);
	ret = (us * (clock/1000))/(etu_per_clk*1000);
	SC_INFO("oo>> sc clk=%lu, e=%lu, %lu us = %lu etu\n", clock, etu_per_clk, us, ret);
	return ret;
}

#ifdef CONFIG_SMARTCARD_NX_ATR_TIMEOUT_CHECK
static void mars_scd_reset_time_counter(mars_scd *p_this)
{
	p_this->internal_timer_base = mars_scd_get_system_clock_us();
	SC_INFO("timer:%lu\n",p_this->internal_timer_base);
}

static unsigned long mars_scd_get_time_counter(mars_scd *p_this)
{
	unsigned long val = mars_scd_get_system_clock_us();

	if (val > p_this->internal_timer_base)
		return val - p_this->internal_timer_base;
	else
		return (0xFFFFFFFF - p_this->internal_timer_base) + val;  // overflow
}
#endif //CONFIG_SMARTCARD_NX_ATR_TIMEOUT_CHECK
/*
#define  GPIO_REQUEST(no)  \
    do {    \
        int ret = gpio_request( (no), "CARD_DETECT");    \
        if(ret < 0) {   \
            SC_WARNING("oo>> gpio_request(%d) failed(%d)\n", (no), ret); \
        }   \
        else {  \
            gpio_direction_output( (no), 0 ); \
            SC_WARNING("oo>> gpio_request(%d) ok(%d) and set to low\n", (no), ret);  \
        } \
    } while(0)

#define GPIO_PULSE(no, usdelay ) \
    do { \
        gpio_direction_output( (no), 1); \
        if( 0 != (usdelay) ) {    udelay( (usdelay) ); }\
        gpio_direction_output( (no), 0); \
    } while(0)

#define GPIO_OUT(no, val ) \
    do { gpio_direction_output( (no), (val)); } while(0)

#define CARD_DETECT_GPIO (10)
*/
/*------------------------------------------------------------------
 * Func : mars_scd_power_enable
 *
 * Desc : enable power for SMC
 *
 * Parm : p_this : handle of mars smartcard ifd
 *        on     : enable / disable power
 *
 * Retn : N/A
 *------------------------------------------------------------------*/
#define MARS_SCD_POWER_ENABLE(_p, _on) \
	do { \
		SC_INFO("power enable to (%d) @ %d\n", (_on), __LINE__); \
		mars_scd_power_enable( (_p), (_on) ); \
	} while(0)

void mars_scd_power_enable(mars_scd* p_this, unsigned char on)
{
	unsigned char val = (on) ? ((p_this->cmd_vcc_polarity) ? 1 :0) : ((p_this->cmd_vcc_polarity) ? 0 :1);

	p_this->pwr_on = on;

	if (p_this->cmd_vcc_en)
		gpio_direction_output(p_this->pin_cmd_vcc, val);
}

/*------------------------------------------------------------------
 * Func : mars_scd_power_select
 *
 * Desc : enable power select for SMC
 *
 * Parm : p_this : handle of mars smartcard ifd
 *        on     : enable / disable power select pin
 *
 * Retn : N/A
 *------------------------------------------------------------------*/
void mars_scd_power_select(mars_scd* p_this, unsigned long vcc)
{
	if (p_this->pwr_sel_en) {
		if (p_this->chip_id == CHIP_ID_RTD1319D) {
			if (vcc == SC_VCC_3V) { /* 3.3v */
				gpio_direction_output(p_this->pin_pwr_sel0, 1);
				gpio_direction_output(p_this->pin_pwr_sel1, 0);
			}
			else if (vcc == SC_VCC_5V) { /* 5v */
				gpio_direction_output(p_this->pin_pwr_sel0, 1);
				gpio_direction_output(p_this->pin_pwr_sel1, 1);
			}
			else { /* 1.8v */
				gpio_direction_output(p_this->pin_pwr_sel0, 0);
				gpio_direction_output(p_this->pin_pwr_sel1, 1);
			}
		}
		else {
			if (vcc == SC_VCC_3V)
				gpio_direction_output(p_this->pin_pwr_sel0, 0);
			else
				gpio_direction_output(p_this->pin_pwr_sel0, 1);
		}
	}
}

#define MARS_SCD_SET_STATE(_p, _state, _ret) \
	do { \
		SC_INFO("FSM=%d enter @ %d, %s\n", (_state), __LINE__, __func__); \
		(_ret) = mars_scd_set_state((_p), (_state)); \
		SC_INFO("FSM=%d done\n", (_state)); \
	} while(0)

/*------------------------------------------------------------------
 * Func : mars_scd_set_state
 *
 * Desc :
 *
 * Parm : p_this : handle of mars smartcard ifd
 *
 * Retn : N/A
 *------------------------------------------------------------------*/
int mars_scd_set_state(mars_scd *p_this, IFD_FSM fsm)
{
	void __iomem *base = p_this->base;
	unsigned char id = p_this->id;
	int ret = 0;
	//unsigned int bd2;

	if (p_this->fsm == fsm)
		return 0;

	switch(fsm) {
	case IFD_FSM_DISABLE:
		SC_INFO("SC%d - FSM = DISABLE\n", id);
		// oo >> just to disable clk ? should it recovery to default clk/etu ????
#if(1)
		SET_SCFP(base, GET_SCFP(p_this->base) & ~SC_CLK_EN_MASK ); // oo>> disable clk....
#else
		GET_BAUDDIV2_HW_VALUE(bd2, p_this->baud_div2);
		SET_SCFP(base, SC_CLK_EN(0)    |                 // oo>> disable clk....
				SC_CLKDIV(p_this->clock_div)|
				SC_BAUDDIV2(bd2)  |
				SC_BAUDDIV1(p_this->baud_div1)|
				SC_PRE_CLKDIV(p_this->pre_clock_div) );
#endif

		SET_SCPCR(base, SC_TXGRDT(SCD_MARS_DEFAULT_EXTRA_GI)    |
				SC_CWI(SCD_MARS_DEFAULT_CWI)    |
				SC_BWI(SCD_MARS_DEFAULT_BWI)    |
				SC_WWI(0)   |
				SC_BGT(SCD_MARS_DEFAULT_BGT)    |
				SC_EDC_EN(0)                    |
				SC_CRC(0)                       |
				SC_PROTOCOL_T(SCD_MARS_DEFAULT_PROTOCOL)|
				SC_T0RTY(0)                     |
				SC_T0RTY_CNT(0));

		SET_SCCR(base, SC_FIFO_RST(1) | SC_SCEN(0) | SC_AUTO_ATR(1) | SC_CONV(0) | SC_PS(p_this->parity));
		SET_SCIRER(base, 0);                  // Disable ISR
		SET_SCIRSR(base, 0xFFFFFFFF);
		MARS_SCD_POWER_ENABLE(p_this, 0);
		p_this->atr.length = -1;
		break;

	case IFD_FSM_DEACTIVATE:
		SC_INFO("SC%d - FSM = DEACTIVATE\n", id);
		// - begin - add to fix Nagxx ICC Test 11 - Paired 5040.2 deactivate with extra clk issue
		MARS_SCD_POWER_ENABLE(p_this, 0);
		udelay(100);
		// - end - add to fix Nagxx ICC Test 11 - Paired 5040.2 deactivate with extra clk issue
		mars_scd_set_etu(p_this, SCD_MARS_DEFAULT_ETU);
		mars_scd_set_parity(p_this, 1);
#if(1)
		// oo >> already config to 372 etu, just to clk en....
		// it maybe need to default clk ???
		SET_SCFP(base, (GET_SCFP(p_this->base) & ~SC_CLK_EN_MASK) | SC_CLK_EN(1) );
#else
		GET_BAUDDIV2_HW_VALUE(bd2, p_this->baud_div2);
		SET_SCFP(base, SC_CLK_EN(1) |
				SC_CLKDIV(p_this->clock_div)|
				SC_BAUDDIV2(bd2) |                    // fixed to 31
				SC_BAUDDIV1(p_this->baud_div1)|
				SC_PRE_CLKDIV(p_this->pre_clock_div) );
#endif

		SET_SCCR(base, SC_FIFO_RST(0) | SC_SCEN(0) | SC_AUTO_ATR(1) | SC_CONV(0) |SC_PS(p_this->parity));
		udelay(1000);
		SET_SCCR(base, SC_FIFO_RST(0) | SC_SCEN(1) | SC_AUTO_ATR(1) | SC_CONV(0) |SC_PS(p_this->parity) | (1<<19));
		SET_SCIRER(base, SC_CPRES_INT);
		SET_SCIRSR(base, 0xFFFFFFFF);
		//MARS_SCD_POWER_ENABLE(p_this, 0); oo>> TDA80xx is disable already.
		p_this->atr.length = -1;
		kfifo_reset(&p_this->rx_fifo);
		break;

	case IFD_FSM_RESET:
		p_this->atr.length = -1;

		if (!mars_scd_card_detect(p_this)) {
			SC_WARNING("SC%d - RESET mars scd failed, no ICC exist\n", id);
			MARS_SCD_SET_STATE(p_this, IFD_FSM_DEACTIVATE, ret);
			return SC_ERR_NO_ICC;
		}

#ifdef CONFIG_SMARTCARD_NX_ATR_TIMEOUT_CHECK
		// oo>> timeout is dependent on current Frequency.
		p_this->atr_char2char_timeout = mars_scd_etu_to_us(p_this, MARS_SCD_DEFAULT_ATR_CWT_etu);
		p_this->atr_activation_timeout =
				MARS_SCD_POWER_ON_INTERVAL_us +
				MARS_SCD_RESET_PULSE_WIDTH_us +
				mars_scd_clk_to_us(p_this, MARS_SCD_DEFAULT_ATR_ACTIVATION_TIMEOUT_clk) +
				MARS_SCD_DEFAULT_1BYTE_2_ATR_TO_CHECK_us;

		SC_INFO("activation to : %lu+%lu+%lu+%lu=%lu us, ch2ch to : %lu us\n",
			MARS_SCD_POWER_ON_INTERVAL_us,
			MARS_SCD_RESET_PULSE_WIDTH_us,
			mars_scd_clk_to_us(p_this, MARS_SCD_DEFAULT_ATR_ACTIVATION_TIMEOUT_clk),
			MARS_SCD_DEFAULT_1BYTE_2_ATR_TO_CHECK_us,
			p_this->atr_activation_timeout,
			p_this->atr_char2char_timeout);

		p_this->g_error_activation_timeout = 0;
		p_this->g_error_char2char_timeout = 0;

		p_this->atr_max_char2char = 0;

		// reset counter before power on to TDA8035
		// it consumes MARS_SCD_POWER_ON_INTERVAL_us for actual Vcc hi
		mars_scd_reset_time_counter(p_this);
#endif // CONFIG_SMARTCARD_NX_ATR_TIMEOUT_CHECK
		mars_scd_power_select(p_this, p_this->vcc);
		MARS_SCD_POWER_ENABLE(p_this, 1);   // Power On, TDA8035 consume MARS_SCD_RESET_PULSE_WIDTH_us from Vcc hi to Rst hi
//GPIO_PULSE(CARD_DETECT_GPIO, 20);
		udelay(100); // oo>> reduce to 100us, it does not inflence the actaul Vcc hi timing.
		mars_scd_set_etu(p_this, SCD_MARS_DEFAULT_ETU);
		mars_scd_set_parity(p_this, 1);
		kfifo_reset(&p_this->rx_fifo);

		SET_SCIRER(base, SC_CPRES_INT    |
				SC_ATRS_INT     |
				SC_RXP_INT      |
				SC_RCV_INT      |
				SC_RX_FOVER_INT);

		SET_SCCR(base, 0);

		SET_SCCR(base, SC_FIFO_RST(1)  |        // pull reset pin
				SC_RST(1)       |
				SC_SCEN(1)      |
				SC_AUTO_ATR(1)  |
				SC_CONV(0)      |
				SC_PS(p_this->parity));

#ifdef ENABLE_ATR_DYNAMIC_TIMEOUT
		if(ATR_PHASE_ap_init != p_this->e_atr_phase)
			SC_WARNING("it should @ init phase from AP (%d)???\n", p_this->e_atr_phase);

		p_this->e_atr_phase = ATR_PHASE_ap_reset;
		p_this->atr_timeout = jiffies + ATR_PHASE_TIMEOUT_JIFFIES_int_1;
#else
		p_this->atr_timeout = jiffies + HZ;
#endif  // ENABLE_ATR_DYNAMIC_TIMEOUT

// oo>> reset_time_counter before MARS_SCD_POWER_ENABLE(1) to reduce sw delay dependency.
//#ifdef CONFIG_SMARTCARD_NX_ATR_TIMEOUT_CHECK
//        mars_scd_reset_time_counter(p_this);
//#endif
		SC_INFO("SC%d - FSM = RESET & ATR\n", id);

		break;

	case IFD_FSM_ACTIVE:
		SC_INFO("SC%d - FSM = ACTIVATE\n", id);

		SET_SCIRER(base, SC_CPRES_INT    |
				SC_ATRS_INT     |
				SC_RXP_INT      |
				SC_RCV_INT      |
				SC_RX_FOVER_INT);
		break;

	default:
	case IFD_FSM_UNKNOWN:
		break;
	}

	p_this->fsm = fsm;

	p_this->card_status_change = 1;
	wake_up(&p_this->wq);

	return 0;
}

/*------------------------------------------------------------------
 * Func : mars_scd_fsm_reset
 *
 * Desc :
 *
 * Parm : p_this : handle of mars cec
 *
 * Retn : N/A
 *------------------------------------------------------------------*/
void mars_scd_fsm_reset(mars_scd* p_this)
{
	unsigned long event  = GET_SCIRSR(p_this->base);
	unsigned long rx_len = 0;
	unsigned long i      = 0;
	int ret = 0;

	SC_INT_DBG("mars_scd_fsm_reset : SC_RISR=%08lx\n", event);

	if ((event & SC_RXP_INT) || !(event & SC_PRES)|| time_after(jiffies, p_this->atr_timeout))
		goto err_atr_failed;

	if (event & SC_ATRS_INT) {
		SC_INT_DBG("mars_scd_fsm_reset : Got ATR INT, reset ATR buffer\n");

		if (p_this->atr.length== -1) {
			p_this->atr.length = 0;         // got ATR

#ifdef CONFIG_SMARTCARD_NX_ATR_TIMEOUT_CHECK
		unsigned long cur_time_us = mars_scd_get_time_counter(p_this);
		if (cur_time_us > p_this->atr_activation_timeout) {
			//GPIO_PULSE(CARD_DETECT_GPIO, 20);
			SC_WARNING("SC%d - ATR Activation timeout. cur=%lu us (%lu clock) > timeout=%lu us (%lu clk)\n",
			p_this->id, cur_time_us,
			mars_scd_us_to_clock(p_this, cur_time_us),
			p_this->atr_activation_timeout,
			mars_scd_us_to_clock(p_this, p_this->atr_activation_timeout));
			p_this->g_error_activation_timeout++;
		}
		else {
			//GPIO_PULSE(CARD_DETECT_GPIO, 20);
			SC_INFO("SC%d - ATR @ cur=%lu us (%lu clock) < timeout=%lu us (%lu clk)\n",
			p_this->id, cur_time_us,
			mars_scd_us_to_clock(p_this, cur_time_us),
			p_this->atr_activation_timeout,
			mars_scd_us_to_clock(p_this, p_this->atr_activation_timeout));
		}
#endif // CONFIG_SMARTCARD_NX_ATR_TIMEOUT_CHECK
		}
	}

	if (event & SC_RCV_INT) {
#ifdef CONFIG_SMARTCARD_NX_ATR_TIMEOUT_CHECK
		unsigned long current_time_counter = mars_scd_get_time_counter(p_this);
		mars_scd_reset_time_counter(p_this);   // restart clock counter
#endif
		SC_INT_DBG("mars_scd_fsm_reset : Got RCV INT, p_this->atr.length==%d, rx_len=%d\n", p_this->atr.length, GET_SC_RXLENR(p_this->base));

#ifdef CONFIG_SMARTCARD_NX_ATR_TIMEOUT_CHECK
		// oo>> ????
		p_this->atr_timeout = jiffies + WAIT_ATR_TIMEOUT;

		if (current_time_counter > p_this->atr_char2char_timeout) {
			SC_WARNING("SC%d - wait ATR ch2ch timeout. cur_time=%lu us (%lu etu), timeout=%lu us (%lu etu)\n",
				p_this->id,
				current_time_counter,
				mars_scd_us_to_etu(p_this, current_time_counter),
				p_this->atr_char2char_timeout,
				mars_scd_us_to_etu(p_this, p_this->atr_char2char_timeout));

			p_this->g_error_char2char_timeout++;
		}
		else if(current_time_counter > p_this->atr_max_char2char) {
			if(0 != p_this->atr_max_char2char) {
		// oo>> dump max ch2ch interval after 1st byte in latest reset. for pairs 5150 info
		// it dumps again if this interval more than last byte.
				SC_WARNING("SC%d - max ch2ch =%lu us (%lu etu), timeout=%lu us (%lu etu)\n",
					p_this->id,
					current_time_counter,
					mars_scd_us_to_etu(p_this, current_time_counter),
					p_this->atr_char2char_timeout,
					mars_scd_us_to_etu(p_this, p_this->atr_char2char_timeout));
			}
			p_this->atr_max_char2char = current_time_counter;
		}
#endif // CONFIG_SMARTCARD_NX_ATR_TIMEOUT_CHECK

		if (p_this->atr.length<0) {
			// unwanted data, drop it
			rx_len = GET_SC_RXLENR(p_this->base);

			for (i=0; i<rx_len; i++)
				GET_SC_RXFIFO(p_this->base);

			goto end_proc;
		}

		// receive atr :
		while(GET_SC_RXLENR(p_this->base)) {
			if (p_this->atr.length >= MAX_ATR_SIZE)
				goto err_atr_failed;

			p_this->atr.data[p_this->atr.length] = GET_SC_RXFIFO(p_this->base);

			if (p_this->atr.length==0) {
				switch(p_this->atr.data[0]) {
				case 0x3B:
					SC_INFO("SC%d - Direct Convention (%02x)\n", p_this->id, p_this->atr.data[0]);
					break;

				case 0x03:
					p_this->atr.data[0] = 0x3F;
					SC_INFO("SC%d - Inverse Convention (%02x)\n", p_this->id, p_this->atr.data[0]);
					break;

				default:
					SC_WARNING("SC%d - unknown TS (%02x)\n", p_this->id, p_this->atr.data[0]);
					break;
				}
			}

			p_this->atr.length++;

#ifdef  ENABLE_ATR_DYNAMIC_TIMEOUT
			if (ATR_PHASE_ap_reset == p_this->e_atr_phase) {
				p_this->atr_timeout = jiffies + ATR_PHASE_TIMEOUT_JIFFIES_int_2;
				p_this->e_atr_phase = ATR_PHASE_isr_get;
			}
#endif  // ENABLE_ATR_DYNAMIC_TIMEOUT

	     //wake_up(&p_this->wq);  //ATR do not use wq
		}

		SC_INT_DBG("mars_scd_fsm_reset : p_this->atr.length=%d\n", p_this->atr.length);

#ifdef CONFIG_SMARTCARD_NX_ATR_TIMEOUT_CHECK
		if (p_this->g_error_activation_timeout) {
			SC_WARNING("SC%d - RESET ICC failed, error_activation_timeout\n", p_this->id);
			goto err_atr_failed;
		}

		if (p_this->g_error_char2char_timeout) {
			SC_WARNING("SC%d - RESET ICC failed, error_char2char_timeout\n", p_this->id);
			goto err_atr_failed;
		}
#endif // CONFIG_SMARTCARD_NX_ATR_TIMEOUT_CHECK

		// check atr
		if (is_atr_complete(&p_this->atr)) {
			SC_INFO("SC%d - Got ATR Completed\n", p_this->id);

			if (decompress_atr(&p_this->atr, &p_this->atr_info)<0)
				goto err_atr_failed;

			MARS_SCD_SET_STATE(p_this, IFD_FSM_ACTIVE, ret);       // jump to active state
		}
	}

end_proc:
	SET_SCIRSR(p_this->base, event);
	return;

err_atr_failed:
	if (!(event & SC_PRES))
		SC_WARNING("SC%d - RESET ICC failed, no ICC detected\n", p_this->id);

	if (event & SC_RXP_INT)
		SC_WARNING("SC%d - RESET ICC failed, RX Parity Error\n", p_this->id);

	if (time_after(jiffies, p_this->atr_timeout))
		SC_WARNING("SC%d - RESET ICC failed, timeout(%lu, %lu)\n",
				p_this->id, jiffies, p_this->atr_timeout);

	if (p_this->atr.length <0)
		SC_WARNING("SC%d - RESET ICC failed, wait ATR failed\n", p_this->id);
	else if (p_this->atr.length >= MAX_ATR_SIZE)
		SC_WARNING("SC%d - RESET ICC failed, atr length %d more then %d\n",
				p_this->id, p_this->atr.length, MAX_ATR_SIZE);
	else if (!is_atr_complete(&p_this->atr))
		SC_WARNING("SC%d - RESET ICC failed, incomplete atr\n", p_this->id);
	else if (decompress_atr(&p_this->atr, &p_this->atr_info)<0)
		SC_WARNING("SC%d - RESET ICC failed, decompress atr failed\n", p_this->id);
	else
		SC_WARNING("SC%d - RESET ICC failed, parse protocol faield\n", p_this->id);

	MARS_SCD_SET_STATE(p_this, IFD_FSM_DEACTIVATE, ret);
	goto end_proc;
}

/*------------------------------------------------------------------
 * Func : mars_scd_fsm_active
 *
 * Desc :
 *
 * Parm : p_this : handle of mars cec
 *
 * Retn : N/A
 *------------------------------------------------------------------*/
void mars_scd_fsm_active(mars_scd* p_this)
{
	int ret;
#ifdef CONFIG_SCD_TX_DBG
	unsigned long status = GET_SCIRSR(p_this->base);
	unsigned long int_en = GET_SCIRER(p_this->base);
#endif
	unsigned long event = GET_SCIRSR(p_this->base) & GET_SCIRER(p_this->base);
	unsigned char i;
	unsigned char len;
	unsigned char buff[TX_RX_DEPTH];

	if (mars_scd_card_detect(p_this)==0) {
		MARS_SCD_SET_STATE(p_this, IFD_FSM_DEACTIVATE, ret);
		goto end_proc;
	}

	if (event & SC_RCV_INT && p_this->tx_status == SC_TX_DONE) { //RX received at least one character
		if ((len = GET_SC_RXLENR(p_this->base))) {
			for (i=0; i<len; i++)
				buff[i] = GET_SC_RXFIFO(p_this->base);

			SC_INFO("RX Receive~~  event=%lu  len=%d  buf=0x%x \n",
					event,len,buff[0]);

			if (kfifo_in_locked(&p_this->rx_fifo, buff, len, &p_this->rx_fifo_lock)<len) {
				SC_WARNING("mars_scd_fsm_active : fifo over flow... flush data...\n");
				kfifo_reset(&p_this->rx_fifo);
			}

			wake_up(&p_this->wq);
		}

		if (event & SC_RX_FOVER_INT) {
			SC_WARNING("Rx over flow!\n");
			SET_SCCR(p_this->base, GET_SCCR(p_this->base) | SC_FIFO_RST(1));
		}
	}

#if 1
	if (event & SC_RCV_INT && p_this->tx_status != SC_TX_DONE) {
		SC_WARNING("RX Receive without TX done. Reduce dump first! event=0x%lx\n",event);
		SET_SCCR(p_this->base, GET_SCCR(p_this->base) | SC_FIFO_RST(1));
	}
#endif

#if defined (SCD_MARS_TX_INT)
	if (event & SC_TXEMPTY_INT) {
		int tx_data_len = kfifo_len(&p_this->tx_fifo); // get number of data to xmit
		if (tx_data_len) {
			unsigned char tx_buff[TX_RX_DEPTH];

			tx_data_len = kfifo_out(&p_this->tx_fifo, tx_buff, sizeof(tx_buff));

			SC_INFO("Send %d Bytes!\n", tx_data_len);

			for(i=0; i<tx_data_len; i++)
				SET_SC_TXFIFO(p_this->base, tx_buff[i]);

			SET_SCCR(p_this->base, GET_SCCR(p_this->base) | SC_TX_GO(1));          // Start Xmit
			SC_TX_INT_DBG(p_this->tx_int_log_fifo, "ev=0x%08x, s,en=(0x%08x,0x%08x), tx_data_len=%d, [0]=0x%02x\n",
					event, status, int_en, tx_data_len, tx_buff[0]);
		}
		else {
			SC_TX_INT_DBG(p_this->tx_int_log_fifo, "TX Done~, ev=0x%08x, s,en=(0x%08x,0x%08x), be00=0x%08x, 0x%08x, 0x%08x\n",
					event, status, int_en, GET_SCFP(p_this->base),GET_SCCR(p_this->base),GET_SCPCR(p_this->base));
			SET_SCIRER(p_this->base, GET_SCIRER(p_this->base) & ~SC_TXEMPTY_INT);
			p_this->tx_status = SC_TX_DONE;
			wake_up(&p_this->wq); // wakeup queue
		}
	}
#elif defined (SCD_MARS_TX_INT_TXDONE)
	if (event & SC_TXDONE_INT) {
		int tx_data_len = kfifo_len(&p_this->tx_fifo);  // get number of data to xmit
		if (tx_data_len) {
			unsigned char tx_buff[TX_RX_DEPTH];

			int tx_fifo_remain = TX_RX_DEPTH - (GET_SC_RXLENR(p_this->base) & SC_RXLENR_RXLEN_MASK);
			int data_remain = tx_data_len;

			tx_data_len = kfifo_out(&p_this->tx_fifo, tx_buff, MIN(tx_fifo_remain, sizeof(tx_buff)) );

			SC_TX_INT_DBG(p_this->tx_int_log_fifo, "Send %d/%d Bytes! (0x%02x, 0x%02x)\n",
					tx_data_len, data_remain, tx_buff[0], tx_buff[tx_data_len-1]);

			for(i=0; i<tx_data_len; i++)
				SET_SC_TXFIFO(p_this->base, tx_buff[i]);

			// oo>> no need to trigger SC_TX_GO
			//SET_SCCR(p_this->base, GET_SCCR(p_this->base) | SC_TX_GO(1));          // Start Xmit
			//SC_TX_INT_DBG(p_this->tx_int_log_fifo, "ev=0x%08x, s,en=(0x%08x,0x%08x), tx_data_len=%d, [0]=0x%02x\n", event, status, int_en, tx_data_len, tx_buff[0]);
		}
		else {
			// enable Tx empty interrupt, disable TX done
			SET_SCIRER(p_this->base, (GET_SCIRER(p_this->base) | SC_TXEMPTY_INT) & ~SC_TXDONE_INT);
			SC_TX_INT_DBG(p_this->tx_int_log_fifo, "Wait tx empty!\n");
			//SC_TX_INT_DBG(p_this->tx_int_log_fifo, "TX Done~, ev=0x%08x, s,en=(0x%08x,0x%08x), be00=0x%08x, 0x%08x, 0x%08x\n", event, status, int_en, GET_SCFP(p_this->base),GET_SCCR(p_this->base),GET_SCPCR(p_this->base));
		}
	}

	if (event & SC_TXEMPTY_INT) {
		// disable SC_TXEMPTY_INT
		SET_SCIRER(p_this->base, GET_SCIRER(p_this->base) & ~SC_TXEMPTY_INT);

		SC_TX_INT_DBG(p_this->tx_int_log_fifo, "Got tx empty!\n");
		p_this->tx_status = SC_TX_DONE;
		wake_up(&p_this->wq);                               // wakeup queue
	}
#endif // SCD_MARS_TX_INT, SCD_MARS_TX_INT_TXDONE

end_proc:
	SET_SCIRSR(p_this->base, event);
}

/*------------------------------------------------------------------
 * Func : mars_scd_work
 *
 * Desc : mars scd work routine
 *
 * Parm : p_this : handle of mars cec
 *
 * Retn : N/A
 *------------------------------------------------------------------*/
void mars_scd_work(mars_scd* p_this)
{
	int ret;
	void __iomem *base = p_this->base;
	unsigned long status = GET_SCIRSR(base) & GET_SCIRER(base);

	SC_INFO("SC%d - work!!  FSM- %d  \n", p_this->id,p_this->fsm);

	if (status & SC_CPRES_INT) {
		if (mars_scd_card_detect(p_this)) {
			SC_INFO("SC%d - ICC detected!! fsm=%d\n",
				p_this->id, p_this->fsm);
		}
		else {
			MARS_SCD_SET_STATE(p_this, IFD_FSM_DEACTIVATE, ret);
			SC_INFO("SC%d - ICC removed!!\n", p_this->id);
		}

		p_this->card_status_change = 1;
		wake_up(&p_this->wq);
	}

	switch (p_this->fsm) {
	case IFD_FSM_RESET:
		mars_scd_fsm_reset(p_this);
		break;
	case IFD_FSM_ACTIVE:
		mars_scd_fsm_active(p_this);
		break;
	default:
		break;
	}

	SET_SCIRSR(base, status);

	SC_INFO("SC%d - work end!!  FSM- %d\n", p_this->id,p_this->fsm);
}

#ifdef ISR_POLLING
/*------------------------------------------------------------------
 * Func : mars_scd_timer
 *
 * Desc : timer of mars scd
 *
 * Parm : arg : input param
 *
 * Retn : IRQ_NONE, IRQ_HANDLED
 *------------------------------------------------------------------*/
static void mars_scd_timer(unsigned long arg)
{
	mars_scd* p_this = (mars_scd*) arg;

	unsigned long event = GET_MIS_ISR(dts_info.misc+0xC) & MIS_SC_ISR[p_this->id];

	if (event)
		mars_scd_work(p_this);

	SET_MIS_ISR(dts_info.misc+0xC, event);

	mod_timer(&p_this->timer, jiffies + ISR_POLLING_INTERVAL);
}
#else

/*------------------------------------------------------------------
 * Func : mars_scd_isr
 *
 * Desc : isr of mars scd
 *
 * Parm : p_this : handle of mars scd
 *        dev_id : handle of the mars_scd
 *        regs   :
 *
 * Retn : IRQ_NONE, IRQ_HANDLED
 *------------------------------------------------------------------*/
static irqreturn_t mars_scd_isr(int this_irq, void *dev_id)
{
	//printk(KERN_ERR "[%s]:[%s] this_irq:[%d] -- [0x%x]\n", __FILE__, __func__, this_irq, readl(dts_info.misc + 0x80));
	mars_scd* p_this = (mars_scd*) dev_id;
	unsigned long flag;
	unsigned long event = GET_MIS_ISR(dts_info.misc+0xC) & MIS_SC_ISR[p_this->id];

	spin_lock_irqsave(&p_this->lock, flag);

	if (event)
		mars_scd_work(p_this);

	SC_INT_DBG("MIS ISR=%08x\n", GET_MIS_ISR(dts_info.misc+0xC));

	SET_MIS_ISR(dts_info.misc+0xC, event);

	spin_unlock_irqrestore(&p_this->lock, flag);

	return IRQ_HANDLED;
}
#endif

/*------------------------------------------------------------------
 * Func : mars_scd_enable
 *
 * Desc : enable mars smart card interface
 *
 * Parm : p_this   : handle of mars scd
 *
 * Retn : 0 : for success, others : fail
 *------------------------------------------------------------------*/
int mars_scd_enable(mars_scd* p_this, unsigned char on_off)
{
	int ret = 0;
	if (on_off) {
		if (p_this->fsm==IFD_FSM_DISABLE) {
			MARS_SCD_SET_STATE(p_this, IFD_FSM_DEACTIVATE, ret);
			msleep(200);   // fix bug of get card status fail on darwin platform....
		}
	}
	else
		MARS_SCD_SET_STATE(p_this, IFD_FSM_DISABLE, ret);

	return ret;
}

int mars_scd_set_vcc_lvl(mars_scd *p_this, unsigned long vcc)
{
	p_this->vcc = vcc;

	mars_scd_power_select(p_this, p_this->vcc);

	return 0;
}

int mars_scd_get_vcc_lvl(mars_scd *p_this, unsigned long *vcc)
{
	*vcc = p_this->vcc;

	return 0;
}

/*------------------------------------------------------------------
 * Func : mars_scd_set_clock
 *
 * Desc : set clock frequency of mars smart card interface
 *
 * Parm : p_this   : handle of mars scd
 *        clk      : clock (in HZ)
 *
 * Retn : 0 : for success, others : fail
 *------------------------------------------------------------------*/
int mars_scd_set_clock(mars_scd* p_this, unsigned long clk)
{
#if defined(ENABLE_CLK_ETU_LOOKUP_TABLE)
	return mars_scd_set_clock_by_table(p_this, clk);
#else
	unsigned long val;
	unsigned long div = p_this->sys_clk / clk / p_this->clock_div;

	if (clk > MAX_SC_CLK) {
		SC_WARNING("clock %lu out of range, using minimum value to instead %lu\n",
				(unsigned long)clk, (unsigned long)MAX_SC_CLK);
		clk = MAX_SC_CLK;
	}

	if (clk < MIN_SC_CLK) {
		SC_WARNING("clock %lu out of range, using minimum value to instead %lu\n",
				(unsigned long)clk, (unsigned long)MIN_SC_CLK);
		clk = MIN_SC_CLK;
	}

	p_this->pre_clock_div = div;

	val  = GET_SCFP(p_this->base) & ~SC_PRE_CLKDIV_MASK;

	val |= SC_PRE_CLKDIV(p_this->pre_clock_div);  // oo add to sync clk config

	SET_SCFP(p_this->base, val);

	return 0;
#endif // ENABLE_CLK_ETU_LOOKUP_TABLE
}

/*------------------------------------------------------------------
 * Func : mars_scd_get_clock
 *
 * Desc : get clock frequency of mars smart card interface
 *
 * Parm : p_this   : handle of mars scd
 *        p_clock  : clock (in HZ)
 *
 * Retn : 0 : for success, others : fail
 *------------------------------------------------------------------*/
int mars_scd_get_clock(mars_scd* p_this, unsigned long* p_clock)
{
	*p_clock = p_this->sys_clk / (p_this->pre_clock_div) / (p_this->clock_div);
	return 0;
}

/*------------------------------------------------------------------
 * Func : mars_scd_set_etu
 *
 * Desc : set etu of mars smart card interface
 *
 * Parm : p_this   : handle of mars scd
 *        etu      : cycles
 *
 * Retn : 0 : for success, others : fail
 *------------------------------------------------------------------*/
int mars_scd_set_etu(mars_scd* p_this, unsigned long etu)
{
#if defined(ENABLE_CLK_ETU_LOOKUP_TABLE)
	return mars_scd_set_etu_by_table(p_this, etu);
#else
	unsigned long val = (etu % 31);
	unsigned char div2 = 0;

	p_this->baud_div2 = 31;

	if ((etu % 39) < val) {
		div2 = 2;
		p_this->baud_div2 = 39;
		val = (etu % 39);
	}

	if ((etu % 32) < val) {
		div2 = 1;
		p_this->baud_div2 = 32;
		val = (etu % 32);
	}

	p_this->baud_div1 = etu * p_this->clock_div / p_this->baud_div2;
	val = GET_SCFP(p_this->base) & ~SC_BAUDDIV_MASK;
	val |= SC_BAUDDIV1(p_this->baud_div1) | SC_BAUDDIV2(div2);

	SC_INFO("ETU = %lu. Baud Div2=%lu, Div1 = %lu\n",
		etu, p_this->baud_div2 ,p_this->baud_div1);

	SET_SCFP(p_this->base, val);

	return 0;
#endif // ENABLE_CLK_ETU_LOOKUP_TABLE
}

/*------------------------------------------------------------------
 * Func : mars_scd_get_etu
 *
 * Desc : set etu of mars smart card interface
 *
 * Parm : p_this   : handle of mars scd
 *        etu      : cycles
 *
 * Retn : 0 : for success, others : fail
 *------------------------------------------------------------------*/
int mars_scd_get_etu(mars_scd* p_this, unsigned long* p_etu)
{
	*p_etu = (p_this->baud_div2 * p_this->baud_div1) / p_this->clock_div;
	//SC_TRACE("oo>> sw(%d,%d,%d), hw(FR=0x%08x, PRE_D=%d, CLK_DIV=%d, BD1=%d, BD2=%d => etu=%lu\n",
	//    p_this->baud_div2, p_this->baud_div1, p_this->clock_div,
	//    GET_SCFP(p_this->base),
	//    SC_PRE_CLKDIV_get(GET_SCFP(p_this->base)),
	//    SC_CLKDIV_get(GET_SCFP(p_this->base)),
	//    SC_BAUDDIV1_get(GET_SCFP(p_this->base)),
	//    SC_BAUDDIV2_get(GET_SCFP(p_this->base)),
	//    *p_etu);

	return 0;
}

int mars_scd_set_convention(mars_scd* p_this, SC_CONV conv)
{
	/* we use auto conv , if necessary , adding it in the future */

        return SC_SUCCESS;
}

int mars_scd_get_convention(mars_scd* p_this, SC_CONV* p_conv)
{
        *p_conv = SC_AUTO_CONV;

        return SC_SUCCESS;
}

/*------------------------------------------------------------------
 * Func : mars_scd_set_parity
 *
 * Desc : set parity of mars smart card interface
 *
 * Parm : p_this   : handle of mars scd
 *        parity   : 0 : off,  others on
 *
 * Retn : 0 : for success, others : fail
 *------------------------------------------------------------------*/
int mars_scd_set_parity(mars_scd* p_this, SC_PARITY parity)
{
	switch(parity) {
	case SC_PARITY_NONE:
		p_this->parity = 0;
		break;

	case SC_PARITY_ODD:
	case SC_PARITY_EVEN:
		p_this->parity = 1;
		break;
	default:
		return SC_FAIL;
	}

	if (p_this->parity)
		SET_SCCR(p_this->base, GET_SCCR(p_this->base) | SC_PS(p_this->parity));
	else
		SET_SCCR(p_this->base, GET_SCCR(p_this->base) & ~SC_PS(p_this->parity));

	return 0;
}

/*------------------------------------------------------------------
 * Func : mars_scd_get_parity
 *
 * Desc : get parity setting of mars smart card interface
 *
 * Parm : p_this   : handle of mars scd
 *        p_parity   : 0 : off,  others on
 *
 * Retn : 0 : for success, others : fail
 *------------------------------------------------------------------*/
int mars_scd_get_parity(mars_scd* p_this, SC_PARITY* p_parity)
{
	*p_parity = (p_this->parity) ? SC_PARITY_EVEN : SC_PARITY_NONE;
	return 0;
}

/*------------------------------------------------------------------
 * Func : mars_scd_set_bgt
 *
 * Desc : set Block Guard Time
 *
 * Parm : p_this   : handle of mars scd
 *        bgt : Block Guard Time
 *
 * Retn : 0 : for success, others : fail
 *------------------------------------------------------------------*/
int mars_scd_set_bgt(mars_scd* p_this, unsigned long bgt)
{
#if(0)
	// oo>> HW bgt  last bit in previous block to 1st leading edge in next block,
	// so HW(BGT) = 7816-3(BGT) - 11 etu.

	/* oo>> ignore this in register file
	if(bgt < 0xc)
	{
		SC_WARNING("illegal BGT(0x%x), enforce to 0xc\n", bgt);
		bgt = 0xc;
	}*/
	if(bgt > 0x1f)
	{
		SC_WARNING("illegal BGT(0x%lx), enforce to 0x1f\n", bgt);
		bgt = 0x1f;
	}
#endif
	// notice that it will minus 12 in macro....
	SET_SCPCR(p_this->base, (GET_SCPCR(p_this->base) & ~MASK_SC_BGT) | SC_BGT(bgt));

	return 0;
}

int mars_scd_set_wwi(mars_scd* p_this, unsigned long wwi)
{
        SET_SCPCR(p_this->base, (GET_SCPCR(p_this->base) & ~MASK_SC_WWI) | SC_WWI(wwi));

        return 0;
}

int mars_scd_set_pin(mars_scd* p_this, unsigned long pin)
{
	p_this->pin = pin;

	switch (pin) {
	case SC_C7:
		return pinctrl_select_state(p_this->pinctrl, p_this->pins_data0);
	case SC_C4:
		return pinctrl_select_state(p_this->pinctrl, p_this->pins_data1);
	case SC_C8:
		return pinctrl_select_state(p_this->pinctrl, p_this->pins_data2);
	default:
		return pinctrl_select_state(p_this->pinctrl, p_this->pins_data0);
	}
}

int mars_scd_get_pin(mars_scd* p_this, unsigned long *pin)
{
	*pin = p_this->pin;

	return 0;
}

int mars_scd_set_pwr(mars_scd* p_this, unsigned long pwr)
{
	MARS_SCD_POWER_ENABLE(p_this, pwr);
	
	return 0;
}

int mars_scd_get_pwr(mars_scd* p_this, unsigned long *pwr)
{
        *pwr = p_this->pwr_on;

        return 0;
}

/*------------------------------------------------------------------
 * Func : mars_scd_get_bgt
 *
 * Desc : get Block Guard Time
 *
 * Parm : p_this   : handle of mars scd
 *        p_bgt : Block Guard Time
 *
 * Retn : 0 : for success, others : fail
 *------------------------------------------------------------------*/
int mars_scd_get_bgt(mars_scd* p_this, unsigned long* p_bgt)
{
	//*p_bgt = (GET_SCPCR(p_this->base) >> 7) & 0x1F;
	*p_bgt = SC_BGT_get(GET_SCPCR(p_this->base));

	return 0;
}

int mars_scd_get_wwi(mars_scd* p_this, unsigned long* p_wwi)
{
	*p_wwi = (GET_SCPCR(p_this->base) >> 12) & 0xF;

	return 0;
}

int mars_scd_set_protocol(mars_scd* p_this, unsigned int val)
{
	if (val > 1) {
		SC_WARNING("protocol=%lu enforce to 0\n", val);
		val = 0;
	}

	SET_SCPCR(p_this->base, (GET_SCPCR(p_this->base) & ~MASK_SC_PROTOCOL) | SC_PROTOCOL_T(val));

	SC_TRACE("be00=0x%08x, 0x%08x, 0x%08x\n",
			GET_SCFP(p_this->base),
			GET_SCCR(p_this->base),
			GET_SCPCR(p_this->base));

	return 0;
}

int mars_scd_get_protocol(mars_scd* p_this, unsigned int* p_val)
{
	*p_val = SC_PROTOCOL_T_get(GET_SCPCR(p_this->base));

	return 0;
}

/*------------------------------------------------------------------
 * Func : mars_scd_set_bwi
 *
 * Desc : set Block Waiting Integer
 *
 * Parm : p_this   : handle of mars scd
 *        bwi : Block Waiting Integer
 *
 * Retn : 0 : for success, others : fail
 *------------------------------------------------------------------*/
int mars_scd_set_bwi(mars_scd* p_this, unsigned long bwi)
{
#if(0)
	if(bwi>0xF)
		bwi = 0xF;

	SET_SCPCR(p_this->base, (GET_SCPCR(p_this->base) & ~SC_BWI(0xF)) | SC_BWI(bwi));
#else
	// sw: 11+960*2^BWI            11+1024*2^BWI
	// 7816 BWI:    SW BWT:        HW BWI:   HW BWT:
	//      0       971                 0       1035
	//      1      1931                 1       2059
	//      2      3851                 2       4107
	//      3      7691                 3       8203
	//      4     15371                 4      16395
	//      5     30731                 5      32779
	//      6     61451                 6      65547
	//      7    122891                 7     131083
	//      8    245771                 8     262155
	//      9    491531                 9     524299
	SET_SCPCR(p_this->base, (GET_SCPCR(p_this->base) & ~MASK_SC_BWI) | SC_BWI(bwi));
#endif

	return 0;
}

/*------------------------------------------------------------------
 * Func : mars_scd_get_bwi
 *
 * Desc : set Block Waiting Integer
 *
 * Parm : p_this : handle of mars scd
 *        p_bwi : Block Waiting Integer
 *
 * Retn : 0 : for success, others : fail
 *------------------------------------------------------------------*/
int mars_scd_get_bwi(mars_scd* p_this, unsigned long* p_bwi)
{
	//*p_bwi = (GET_SCPCR(p_this->base) >> 16) & 0x0F;
	*p_bwi = SC_BWI_get(GET_SCPCR(p_this->base));
	return 0;
}

/*------------------------------------------------------------------
 * Func : mars_scd_set_cwi
 *
 * Desc : set Character Waiting Integer
 *
 * Parm : p_this   : handle of mars scd
 *        cwi : Character Waiting Integer
 *
 * Retn : 0 : for success, others : fail
 *------------------------------------------------------------------*/
int mars_scd_set_cwi(mars_scd* p_this, unsigned long cwi)
{
	//if(cwi>0xF)
	//	cwi = 0xF;

	// oo>> SW: CWT: 11+2^cwi
	// oo>> HW: CWT: 11.7+2^cwi, how to mapping??
	SET_SCPCR(p_this->base, (GET_SCPCR(p_this->base) & ~MASK_SC_CWI) | SC_CWI(cwi));

	return 0;
}

/*------------------------------------------------------------------
 * Func : mars_scd_get_cwi
 *
 * Desc : set Character Waiting Integer
 *
 * Parm : p_this : handle of mars scd
 *        p_cwi : Character Waiting Integer
 *
 * Retn : 0 : for success, others : fail
 *------------------------------------------------------------------*/
int mars_scd_get_cwi(mars_scd* p_this, unsigned long* p_cwi)
{
	//*p_cwi = (GET_SCPCR(p_this->base) >> 20) & 0x0F;
	*p_cwi = SC_CWI_get(GET_SCPCR(p_this->base));

	return 0;
}

/*------------------------------------------------------------------
 * Func : mars_scd_set_guard_interval
 *
 * Desc : set guard interval of icam smart card interface
 *
 * Parm : p_this   : handle of mars scd
 *        guard_interval : guard interval in etu
 *
 * Retn : 0 : for success, others : fail
 *------------------------------------------------------------------*/
int mars_scd_set_guard_interval(mars_scd* p_this, unsigned long guard_interval)
{
	// oo >> the same definition between HW and 7816-3.
	// 0~254: extra GT=12 + GI
	// 0 and 255 are 12etu@T=0, 11etu@T=1
	SET_SCPCR(p_this->base, (GET_SCPCR(p_this->base) & ~MASK_SC_TXGRDT) | SC_TXGRDT(guard_interval));
	return 0;
}

/*------------------------------------------------------------------
 * Func : mars_scd_get_guard_interval
 *
 * Desc : set guard interval
 *
 * Parm : p_this : handle of mars scd
 *        p_guard_interval : guard interval output
 *
 * Retn : 0 : for success, others : fail
 *------------------------------------------------------------------*/
int mars_scd_get_guard_interval(mars_scd* p_this, unsigned long* p_guard_interval)
{
	//unsigned long guard_time = (GET_SCPCR(p_this->base) >> 24) & 0xFF;
	*p_guard_interval = SC_TXGRDT_get(GET_SCPCR(p_this->base));

	return 0;
}

/*------------------------------------------------------------------
 * Func : mars_scd_activate
 *
 * Desc : activate an ICC
 *
 * Parm : p_this   : handle of mars scd
 *
 * Retn : 0 : for success, others : fail
 *------------------------------------------------------------------*/
int mars_scd_activate(mars_scd* p_this)
{
	switch(p_this->fsm) {
	case IFD_FSM_DISABLE:
		SC_WARNING("activate ICC failed, please enable IFD first\n");
		return SC_ERR_IFD_DISABLED;

	case IFD_FSM_DEACTIVATE:
		if (mars_scd_reset(p_this)==0) {
			SC_INFO("activate ICC success\n");
			return SC_SUCCESS;
		}
		else {
			SC_WARNING("activate ICC failed\n");
			return SC_ERR_NO_ICC;
		}
		break;

	case IFD_FSM_RESET:
		SC_INFO("ICC has is reseting\n");
		return SC_SUCCESS;

	case IFD_FSM_ACTIVE:
		SC_INFO("ICC has been activated already\n");
		return SC_SUCCESS;

	default:
		SC_WARNING("activate ICC failed, unknown state\n");
		return SC_FAIL;
	}
}

/*------------------------------------------------------------------
 * Func : mars_scd_deactivate
 *
 * Desc : deactivate an ICC
 *
 * Parm : p_this   : handle of mars scd
 *
 * Retn : 0 : for success, others : fail
 *------------------------------------------------------------------*/
int mars_scd_deactivate(mars_scd* p_this)
{
	int ret;
	if (p_this->fsm != IFD_FSM_DISABLE) {
		// SA5-995, IFD_FSM_DISABLE stop clk before deactivate waveform...
		//MARS_SCD_SET_STATE(p_this, IFD_FSM_DISABLE, ret);
		MARS_SCD_SET_STATE(p_this, IFD_FSM_DEACTIVATE, ret);
	}

	return 0;
}

/*------------------------------------------------------------------
 * Func : mars_scd_reset
 *
 * Desc : reset mars smart card interface
 *
 * Parm : p_this   : handle of mars scd
 *
 * Retn : 0 : for success, others : fail
 *------------------------------------------------------------------*/
int mars_scd_reset(mars_scd* p_this)
{
	unsigned long t;
	int ret;
	// wait for DEACTIVATE state , otherwise reset will be failed
	wait_event_interruptible_timeout(p_this->wq,  p_this->fsm == IFD_FSM_DEACTIVATE, HZ>>1);

#ifdef ENABLE_ATR_DYNAMIC_TIMEOUT
	p_this->e_atr_phase = ATR_PHASE_ap_init;
	t = ATR_PHASE_TIMEOUT_JIFFIES_ap_1;
#else
	t = (HZ<<1);
#endif  // ENABLE_ATR_DYNAMIC_TIMEOUT

	MARS_SCD_SET_STATE(p_this, IFD_FSM_RESET, ret);
	if (ret) {
		ret = SC_FAIL;
		goto mars_scd_reset_end;
	}

	t += jiffies;

	while (!time_after(jiffies,t) && p_this->fsm == IFD_FSM_RESET) {
		//printk(KERN_ERR "...");
		msleep(100);

#ifdef ENABLE_ATR_DYNAMIC_TIMEOUT
		if (ATR_PHASE_isr_get == p_this->e_atr_phase) {
			t = jiffies + ATR_PHASE_TIMEOUT_JIFFIES_ap_2;
			p_this->e_atr_phase = ATR_PHASE_ap_ack;
		}
#endif // ENABLE_ATR_DYNAMIC_TIMEOUT
	}
	//printk(KERN_ERR "\n");

	if (p_this->fsm != IFD_FSM_ACTIVE) {
		SC_WARNING("SC%d - Reset ICC failed\n", p_this->id);
		MARS_SCD_SET_STATE(p_this, IFD_FSM_DEACTIVATE, ret);
		ret = SC_ERR_ATR_TIMEOUT;
		goto mars_scd_reset_end;
	}

	SC_INFO("SC%d - Reset ICC Complete, atr_len=%d\n", p_this->id, p_this->atr.length);
	ret = SC_SUCCESS;

mars_scd_reset_end:
#ifdef ENABLE_ATR_DYNAMIC_TIMEOUT
	p_this->e_atr_phase = ATR_PHASE_unused;
#endif // ENABLE_ATR_DYNAMIC_TIMEOUT

	return ret;
}

/*------------------------------------------------------------------
 * Func : mars_scd_get_atr
 *
 * Desc : s mars smart card interface
 *
 * Parm : p_this   : handle of mars scd
 *        p_atr    : atr output buffer
 *
 * Retn : 0 : for success, others : fail
 *------------------------------------------------------------------*/
int mars_scd_get_atr(mars_scd* p_this, scd_atr* p_atr)
{
	if (p_this->fsm != IFD_FSM_ACTIVE) {
		p_atr->length = -1;
		return -1;
	}

	p_atr->length = p_this->atr.length;
	memcpy(p_atr->data, p_this->atr.data, p_this->atr.length);

	return 0;
}

/*------------------------------------------------------------------
 * Func : mars_scd_card_detect
 *
 * Desc : get card status
 *
 * Parm : p_this   : handle of mars scd
 *
 * Retn : 0 : no icc exists, others : icc exists
 *------------------------------------------------------------------*/
int mars_scd_card_detect(mars_scd* p_this)
{
	return (GET_SCIRSR(p_this->base) & SC_PRES) ? 1 : 0;
}

/*------------------------------------------------------------------
 * Func : mars_scd_get_card_status
 *
 * Desc : get card status
 *
 * Parm : p_this   : handle of mars scd
 *
 * Retn : 0 : no icc exists, others : icc exists
 *------------------------------------------------------------------*/
int mars_scd_get_card_status(mars_scd* p_this)
{
	switch(p_this->fsm) {
	case IFD_FSM_DISABLE:
		return SC_CARD_REMOVED;
	case IFD_FSM_DEACTIVATE:
		return mars_scd_card_detect(p_this) ? SC_CARD_DEACTIVATE : SC_CARD_REMOVED;
	case IFD_FSM_RESET:
		return SC_CARD_RESET;
	case IFD_FSM_ACTIVE:
		return SC_CARD_ACTIVATE;
	default:
		return SC_CARD_UNKNOWN;
	}
}

/*------------------------------------------------------------------
 * Func : mars_scd_poll_card_status
 *
 * Desc : poll card status change
 *
 * Parm : p_this   : handle of mars scd
 *
 * Retn : 0 : for success, others : fail
 *------------------------------------------------------------------*/
int mars_scd_poll_card_status(mars_scd* p_this)
{
	p_this->card_status_change = 0;
	wait_event_interruptible(p_this->wq, p_this->card_status_change);

	return mars_scd_get_card_status(p_this);
}

/*------------------------------------------------------------------
 * Func : mars_scd_xmit
 *
 * Desc : xmit data via smart card bus
 *
 * Parm : p_this   : handle of mars scd
 *        scb      : data to be xmit
 *
 * Retn : SC_SUCCESS / SC_FAIL
 *------------------------------------------------------------------*/
#ifdef SCD_MARS_TX_INT
static int mars_scd_xmit_int(mars_scd* p_this, unsigned char* p_data, unsigned int len)
{
	unsigned char id = p_this->id;
	void __iomem *base = p_this->base;
	unsigned char tx_buff[TX_RX_DEPTH];
	unsigned char tx_data_len;
	int i;
	//SC_WARNING("0x%08x:0x%08x:0x%08x:  \n",GET_SCFP(base),GET_SCCR(base),GET_SCPCR(base));
	if (mars_scd_card_detect(p_this) == 0)
		return SC_ERR_NO_ICC;

	if (p_this->fsm != IFD_FSM_ACTIVE)
		return SC_ERR_ICC_DEACTIVATE;

	kfifo_reset(&p_this->rx_fifo);

	kfifo_reset(&p_this->tx_fifo);

	//kfifo_reset(&p_this->tx_int_log_fifo);

	if (kfifo_in(&p_this->tx_fifo, p_data, len)<len) {
		SC_WARNING("[SC%d] xmit data failed, tx fifo overflow\n", id);
		return SC_FAIL;
	}

	// load tx data
	tx_data_len = kfifo_out(&p_this->tx_fifo, tx_buff, sizeof(tx_buff));

	for(i=0; i<tx_data_len; i++)
		SET_SC_TXFIFO(base, tx_buff[i]);

	//SC_WARNING("Send %d Bytes!\n", tx_data_len);

	// kick-off
	p_this->tx_status = 0;

	SET_SCIRER(base, GET_SCIRER(base) | SC_TXEMPTY_INT);   // enable Tx empty interrupt

	SET_SCCR(base, GET_SCCR(base) | SC_TX_GO(1));          // Start Xmit

	//for the most INF of 254byte(add NAD,PCB,LEN,EDC is 258byte<32byte*16),the timeout must be 16 times of timer
	wait_event_interruptible_timeout(p_this->wq, (p_this->tx_status & SC_TX_DONE), HZ);

	SET_SCIRER(base, GET_SCIRER(base) & ~SC_TXEMPTY_INT);   // disable Tx empty interrupt

	SC_TX_INT_DUMP(p_this->tx_int_log_fifo, len);

	if ((p_this->tx_status & SC_TX_DONE) == 0) {
		SC_WARNING("[SC%d] xmit data failed, tx timeout\n", id);
		SET_SCCR(base, GET_SCCR(base) & ~SC_TX_GO(1));  // stop Xmit
		return SC_FAIL;
	}

	return SC_SUCCESS;
}
#elif defined (SCD_MARS_TX_INT_TXDONE)
static int mars_scd_xmit_int_txdone(mars_scd* p_this, unsigned char* p_data, unsigned int len)
{
	unsigned char id = p_this->id;
	void __iomem *base = p_this->base;
	unsigned char tx_buff[TX_RX_DEPTH];
	unsigned char tx_data_len;
	int i;
	int ret;
	//SC_WARNING("0x%08x:0x%08x:0x%08x:  \n",GET_SCFP(base),GET_SCCR(base),GET_SCPCR(base));

	if (mars_scd_card_detect(p_this) == 0)
		return SC_ERR_NO_ICC;

	if (p_this->fsm != IFD_FSM_ACTIVE)
		return SC_ERR_ICC_DEACTIVATE;

	kfifo_reset(&p_this->rx_fifo);

	kfifo_reset(&p_this->tx_fifo);

	kfifo_reset(&p_this->tx_int_log_fifo);

	if (kfifo_in(&p_this->tx_fifo, p_data, len) < len) {
		SC_WARNING("[SC%d] xmit data failed, tx fifo overflow\n", id);
		return SC_FAIL;
	}

	// load tx data
	tx_data_len = kfifo_out(&p_this->tx_fifo, tx_buff, sizeof(tx_buff));

	//GPIO_OUT(CARD_DETECT_GPIO, 1);

	for(i=0; i<tx_data_len; i++)
		SET_SC_TXFIFO(base, tx_buff[i]);

	SC_TX_INT_DBG(p_this->tx_int_log_fifo, "Send %d Bytes!, remain=%d for INT\n",
					tx_data_len, (len-tx_data_len) );

	// kick-off
	p_this->tx_status = 0;

	SET_SCIRER(base, GET_SCIRER(base) | SC_TXDONE_INT);   // enable Tx done interrupt

	SET_SCCR(base, GET_SCCR(base) | SC_TX_GO(1));          // Start Xmit

	//for the most INF of 254byte(add NAD,PCB,LEN,EDC is 258byte<32byte*16),the timeout must be 16 times of timer
	wait_event_interruptible_timeout(p_this->wq, (p_this->tx_status & SC_TX_DONE), HZ);

	// oo>> ISR should disable SC_TXDONE_INT and SC_TXEMPTY_INT....
	//SET_SCIRER(base, GET_SCIRER(base) & ~SC_TXDONE_INT);   // disable Tx done interrupt

	SET_SCCR(base, GET_SCCR(base) & ~SC_TX_GO(1));      // stop Xmit

	//GPIO_OUT(CARD_DETECT_GPIO, 0);

	if ((p_this->tx_status & SC_TX_DONE) == 0) {
		// oo>> something wrong, ISR did not chage tx_status,
		// disable INTs
		SET_SCIRER(p_this->base, (GET_SCIRER(p_this->base) & ~SC_TXEMPTY_INT) & ~SC_TXDONE_INT);

		SC_TX_INT_DBG(p_this->tx_int_log_fifo, "[SC%d] xmit data failed, tx timeout\n", id);
		ret = SC_FAIL;
	}
	else {
		// oo>> do nothing, SC_TXDONE_INT, SC_TXEMPTY_INT should be disable in ISR in normal case.
		SC_TX_INT_DBG(p_this->tx_int_log_fifo, "[SC%d] xmit data(%d) done\n", id, len);
		ret = SC_SUCCESS;
	}

mars_scd_xmit_int_txdone_end:
	SC_TX_INT_DUMP(p_this->tx_int_log_fifo, len);

	return ret;
}
#else

static int mars_scd_xmit_polling(mars_scd* p_this, unsigned char* p_data, unsigned int len)
{
	unsigned char id = p_this->id;
	void __iomem *base = p_this->base;
	unsigned char tx_buff[TX_RX_DEPTH];
	int tx_data_len;
	int tx_remain_len = len;
	int i;

	if (mars_scd_card_detect(p_this) == 0)
		return SC_ERR_NO_ICC;

	if (p_this->fsm != IFD_FSM_ACTIVE)
		return SC_ERR_ICC_DEACTIVATE;

	kfifo_reset(&p_this->rx_fifo);

	kfifo_reset(&p_this->tx_fifo);

	if (kfifo_in(&p_this->tx_fifo, p_data, len) < len) {
		SC_WARNING("[SC%d] xmit data failed, tx fifo overflow\n", id);
		return SC_FAIL;
	}

	SC_INFO("len=%d, s,en=(0x%08x,0x%08x), be00=0x%08x, 0x%08x, 0x%08x\n",
			len, GET_SCIRSR(base), GET_SCIRER(base),
			GET_SCFP(base),GET_SCCR(base),GET_SCPCR(base));

	while(tx_remain_len > 0) {
		// load tx data
		tx_data_len = kfifo_out(&p_this->tx_fifo, tx_buff, sizeof(tx_buff));

		for(i=0; i<tx_data_len; i++)
			SET_SC_TXFIFO(base, tx_buff[i]);

		// kick-off
		p_this->tx_status = 0;

		SET_SCCR(base, GET_SCCR(base) | SC_TX_GO(1)); // Start Xmit

		tx_remain_len = tx_remain_len - tx_data_len;

		while(1) {
			if (GET_SCIRSR(p_this->base) & SC_TXEMPTY_INT) {
				// clear TX_EMPTY status to avoid missed data in next turn.
				SET_SCIRSR(base, GET_SCIRSR(base) & (SC_TXEMPTY_INT | SC_TXDONE_INT) );
				// any dump to stdout in this loop causes confused timing\n");
				break;
			}
		}
	}

	SET_SCCR(base, GET_SCCR(base) & ~SC_TX_GO(1));      // stop Xmit
	p_this->tx_status = SC_TX_DONE;

	SC_INFO("Send %d byte done\n", len);

	return SC_SUCCESS;
}
#endif // SCD_MARS_TX_INT, SCD_MARS_TX_INT_TXDONE

int mars_scd_xmit(mars_scd* p_this, unsigned char* p_data, unsigned int len)
{
	int ret;
#if defined (SCD_MARS_TX_INT)
	ret = mars_scd_xmit_int(p_this, p_data, len);
#elif defined (SCD_MARS_TX_INT_TXDONE)
	ret = mars_scd_xmit_int_txdone(p_this, p_data, len);
#else
	ret = mars_scd_xmit_polling(p_this, p_data, len);
#endif // SCD_MARS_TX_INT
	return ret;
}

/*------------------------------------------------------------------
 * Func : mars_scd_read
 *
 * Desc : read data via smart card bus
 *
 * Parm : p_this   : handle of mars scd
 *
 * Retn : sc_buff
 *------------------------------------------------------------------*/
int mars_scd_read(mars_scd* p_this, unsigned char* data, unsigned int len)
{
	if (mars_scd_card_detect(p_this) == 0)
		return SC_ERR_NO_ICC;

	wait_event_interruptible_timeout(p_this->wq, kfifo_len(&p_this->rx_fifo) >=len || (p_this->fsm != IFD_FSM_ACTIVE), HZ>>2);

	if (p_this->fsm != IFD_FSM_ACTIVE)
		return SC_ERR_ICC_DEACTIVATE;

	return  kfifo_out_locked(&p_this->rx_fifo, data, len, &p_this->rx_fifo_lock);
}

/*------------------------------------------------------------------
 * Func : mars_scd_load_gpio_config
 *
 * Desc : Load Gpio Config
 *
 * Parm : p_this   : handle of mars scd
 *
 * Retn : sc_buff
 *------------------------------------------------------------------*/
static void mars_scd_load_gpio_config(mars_scd* p_this)
{
	/**************************************
	* SCx Command format
	**************************************
	: cmd_vcc_en        (0: no cmd_vcc_gpio, 1: with cmd_vcc_gpio)
        : cmd_vcc_polarity  (0: negative, 1: positive)
        : cmd_vcc_gpio_type (0: MISC, 1: ISO)
        : --
        : pwr_sel_en        (0: no cmd_vcc_gpio, 1: with cmd_vcc_gpio)
        : pwr_sel_polarity  (0: negative, 1: positive)
        : pwr_sel_gpio_type (0: MISC, 1: ISO)
        : --
        : cmd_vcc_gpio
        : pwr_sel_gpio
	**************************************/

	p_this->cmd_vcc_en       = dts_info.cmd_vcc_en;
	p_this->cmd_vcc_polarity = dts_info.cmd_vcc_polarity;
	p_this->pin_cmd_vcc      = dts_info.pin_cmd_vcc;

	p_this->pwr_sel_en       = dts_info.pwr_sel_en;
	p_this->pwr_sel_polarity = dts_info.pwr_sel_polarity;
	p_this->pin_pwr_sel0	 = dts_info.pin_pwr_sel0;
	if (p_this->chip_id == CHIP_ID_RTD1319D)
		p_this->pin_pwr_sel1 = dts_info.pin_pwr_sel1;

#if(1)
	if (p_this->vcc != p_this->pwr_sel_polarity) {
		SC_WARNING("SC%d = default pwr changed to %s\n",
			p_this->id, p_this->pwr_sel_polarity ? "5v" : "3v");
			p_this->vcc = p_this->pwr_sel_polarity;
	}
#endif

	SC_INFO("SC%d - cmd_vcc(en=%d, polarity=%d, pin=%s_GPIO[%d]), pwr_sel(en=%d,polarity=%d,pin=%s_GPIO[%d])\n",
		p_this->id,
		p_this->cmd_vcc_en,
		p_this->cmd_vcc_polarity,
		gpio_type(gpio_group(p_this->pin_cmd_vcc)),
		gpio_idx(p_this->pin_cmd_vcc),
		p_this->pwr_sel_en,
		p_this->pwr_sel_polarity,
		gpio_type(gpio_group(p_this->pin_pwr_sel0)),
		gpio_idx(p_this->pin_pwr_sel0));
}

/*------------------------------------------------------------------
 * Func : mars_clk_reset_ctrl
 *
 * Desc :
 *
 * Parm :
 *
 * Retn :
 *------------------------------------------------------------------*/
static int mars_clk_reset_ctrl(mars_scd* p_this, SCD_CLK_CTL enable)
{
	if (enable == CTL_ENABLE) {
		reset_control_deassert(p_this->rstc);
		clk_prepare_enable(p_this->clk);
	}
	else {
		clk_disable_unprepare(p_this->clk);
		reset_control_assert(p_this->rstc);
	}

	return 0;
}

/*------------------------------------------------------------------
 * Func : mars_scd_open
 *
 * Desc : open a mars scd device
 *
 * Parm : id  : channel id
 *
 * Retn : handle of mars scd
 *------------------------------------------------------------------*/
mars_scd* mars_scd_open(unsigned char id)
{
	mars_scd* p_this;
	//void __iomem *gpio = ioremap(0x9801B000, 0x120);
	int ret = 0;
	unsigned int bd2;

	if (id >= MAX_IFD_CNT) {
		SC_WARNING("scd : open %s scd failed, invalid id - %d\n",
				IFD_MODOLE, id);
		return NULL;
	}

	p_this = (mars_scd*)kmalloc(sizeof(mars_scd), GFP_KERNEL);
	if (!p_this) {
		SC_WARNING("scd : kmalloc fail.\n");
		return NULL;
	}


        memset(p_this, 0, sizeof(mars_scd));

	p_this->chip_id = dts_info.chip_id;
        p_this->base    = dts_info.base;
        p_this->id      = dts_info.id;
	p_this->pinctrl = dts_info.pinctrl;
	p_this->pins_default = dts_info.pins_default;
	p_this->pins_data0 = dts_info.pins_data0;
	p_this->pins_data1 = dts_info.pins_data1;
	p_this->pins_data2 = dts_info.pins_data2;
        p_this->fsm     = IFD_FSM_UNKNOWN;
        p_this->atr.length = -1;

	p_this->clk = dts_info.clk;
	p_this->clk_sel = dts_info.clk_sel;
	p_this->rstc = dts_info.rstc;

        if (kfifo_alloc(&p_this->rx_fifo, 1024, GFP_KERNEL) < 0) {
		SC_WARNING("scd : open %s scd(%d) failed, create rx fifo failed\n",
				IFD_MODOLE, id);
		kfree(p_this);
		return NULL;
        }

        if (kfifo_alloc(&p_this->tx_fifo, 1024, GFP_KERNEL) < 0) {
		SC_WARNING("scd : open %s scd(%d) failed, create tx fifo failed\n",
				IFD_MODOLE, id);
		kfifo_free(&p_this->rx_fifo);
		kfree(p_this);
		return NULL;
        }

#if defined  (SCD_MARS_TX_INT) || defined (SCD_MARS_TX_INT_TXDONE)
        if (kfifo_alloc(&p_this->tx_int_log_fifo, TX_INT_LOG_MAX_LEN, GFP_KERNEL) < 0) {
		SC_WARNING("scd : open %s scd(%d) failed, create tx int log fifo failed\n",
				IFD_MODOLE, id);
		kfifo_free(&p_this->tx_int_log_fifo);
		kfree(p_this);
		return NULL;
	}
#endif

        mars_clk_reset_ctrl(p_this, CTL_ENABLE);
        /* set misc interrupt */
        writel(readl(dts_info.misc + 0x80) | (0x1 << 24), dts_info.misc + 0x80);

        spin_lock_init(&p_this->lock);
        spin_lock_init(&p_this->rx_fifo_lock);
        init_waitqueue_head(&p_this->wq);

        //oo>> it cannot call mars_scd_set_etu() due to no initial value in p_this.
        p_this->clock_div        = SCD_MARS_DEFAULT_CLKDIV;	// 1
        p_this->pre_clock_div    = SCD_MARS_DEFAULT_PRE_CLKDIV; //(unsigned char)(SYSTEM_CLK_27M / SCD_MARS_DEFAULT_CLK);
        p_this->baud_div1        = SCD_MARS_DEFAULT_BAUDDIV1;   // default etu = 372 = 31 * 12
        p_this->baud_div2        = SCD_MARS_DEFAULT_BAUDDIV2_e;
        p_this->parity           = 1;
        p_this->vcc		 = 0;
	p_this->pin		 = SC_C7;
	p_this->sys_clk		 = SYSTEM_CLK_27M;

        mars_scd_load_gpio_config(p_this);

	if (p_this->cmd_vcc_en)
		MARS_SCD_POWER_ENABLE(p_this, 0);

        if (p_this->pwr_sel_en)
		mars_scd_power_select(p_this, p_this->vcc);

        // Set All Register to the initial value
        GET_BAUDDIV2_HW_VALUE(bd2, (MAR_SCD_BAUDDIV2_LIST_e)p_this->baud_div2);
        SET_SCFP(p_this->base, SC_CLK_EN(1)    |
                            SC_CLKDIV(p_this->clock_div)|
                            SC_BAUDDIV2(bd2)  |
                            SC_BAUDDIV1(p_this->baud_div1)|
                            SC_PRE_CLKDIV(p_this->pre_clock_div) );

        SET_SCCR(p_this->base, SC_FIFO_RST(1)  |
                            SC_SCEN(1)      |
                            SC_AUTO_ATR(1)  |
                            SC_CONV(0)      |
                            SC_PS(0));

        SET_SCPCR(p_this->base,SC_TXGRDT(SCD_MARS_DEFAULT_EXTRA_GI)    |
                            SC_CWI(SCD_MARS_DEFAULT_CWI)       |
                            SC_BWI(SCD_MARS_DEFAULT_BWI)       |
                            SC_WWI(0)       |
                            SC_BGT(SCD_MARS_DEFAULT_BGT)    |
                            SC_EDC_EN(0)    |
                            SC_CRC(0)       |
                            SC_PROTOCOL_T(SCD_MARS_DEFAULT_PROTOCOL)|
                            SC_T0RTY(0)     |
                            SC_T0RTY_CNT(0) );

        SET_SCIRER(p_this->base, 0);

        SET_SCIRSR(p_this->base, 0xFFFFFFFF);

#ifdef ISR_POLLING
        init_timer(&p_this->timer);
        p_this->timer.data     = (unsigned long)p_this;
        p_this->timer.function = mars_scd_timer;
        p_this->timer.expires  = jiffies + ISR_POLLING_INTERVAL;
        add_timer(&p_this->timer);      // register timer
#else
        p_this->irq = dts_info.irq;
        //if (request_irq(p_this->irq, mars_scd_isr, SA_SHIRQ, "MARS SCD", (void *) p_this) < 0)
        if (request_irq(p_this->irq, mars_scd_isr, IRQF_SHARED, "MARS SCD", (void *) p_this) < 0) {
		SC_WARNING("scd : open %s scd failed, unable to request irq#%d\n",
				IFD_MODOLE, p_this->irq);
		kfifo_free(&p_this->tx_fifo);
		kfifo_free(&p_this->rx_fifo);
		kfree(p_this);
		return NULL;
        }

        printk(KERN_ERR "scd : open %s scd and request irq#%d successfully.\n",
				IFD_MODOLE, p_this->irq);
#endif
        MARS_SCD_SET_STATE(p_this, IFD_FSM_DISABLE, ret);

	return p_this;
}

/*------------------------------------------------------------------
 * Func : mars_scd_close
 *
 * Desc : close a mars scd device
 *
 * Parm : p_this  : mars scd to be close
 *
 * Retn : N/A
 *------------------------------------------------------------------*/
void mars_scd_close(mars_scd* p_this)
{
	int ret =0;
#ifdef ISR_POLLING
	del_timer(&p_this->timer);      // register timer
#else
	free_irq(MISC_IRQ ,p_this);
#endif

	kfifo_free(&p_this->rx_fifo);
	kfifo_free(&p_this->tx_fifo);
	MARS_SCD_SET_STATE(p_this, IFD_FSM_DISABLE, ret);
	wake_up(&p_this->wq);
	kfree(p_this);
}

