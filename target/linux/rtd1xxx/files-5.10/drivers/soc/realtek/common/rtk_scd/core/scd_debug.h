// SPDX-License-Identifier: (GPL-2.0-or-later OR BSD-2-Clause)
/*
 * Copyright (c) 2019 Realtek Semiconductor Corp.
 */

#ifndef __SCD_DEBUG_H__
#define __SCD_DEBUG_H__

//-- scd debug messages
//#define CONFIG_SMARTCARD_DBG
//#define CONFIG_SCD_INT_DBG
//#define CONFIG_SCD_TX_DBG      // oo>> dump TX INT log may causes RX overflow if Rx more than FIFO depth
//#define CONFIG_SCD_RX_DBG
//#define CONFIG_SCD_INFO
//#define CONFIG_SCD_WARNING

#ifdef  CONFIG_SMARTCARD_DBG
#define SC_TRACE(fmt, args...)		printk(KERN_EMERG "[SCD] TRACE,(%s,%d)" fmt, __FILE__, __LINE__, ## args)
#else
#define SC_TRACE(args...)
#endif


#ifdef  CONFIG_SCD_INT_DBG
#define SC_INT_DBG(fmt, args...)	printk(KERN_EMERG "[SCD] INT, " fmt, ## args)
#else
#define SC_INT_DBG(args...)
#endif

#ifdef CONFIG_SCD_INFO
#define SC_INFO(fmt, args...)		printk(KERN_EMERG "[SCD] Info,(%d) " fmt, __LINE__, ## args)
#else
#define SC_INFO(fmt, args...)
#endif

#ifdef CONFIG_SCD_WARNING
#define SC_WARNING(fmt, args...)	printk(KERN_EMERG "[SCD] Warning,(%s,%d)" fmt, __FILE__, __LINE__, ## args)
#else
#define SC_WARNING(fmt, args...)
#endif

#ifdef CONFIG_SCD_TX_DBG
#define SC_TX_INT_DBG(fd, fmt, args...)  \
	do  { \
		unsigned char temp[TX_INT_LOG_MAX_LEN]; \
		sprintf(temp, "[SCD] INT(%d), " fmt, __LINE__, ## args); \
		kfifo_in(&(fd), temp, strlen(temp)); \
	} while(0);

#define SC_TX_INT_DUMP(fd, tx_len) \
	do { \
		unsigned char temp[TX_INT_LOG_MAX_LEN]; \
		unsigned char *pCur = temp; \
		int log_len = kfifo_len(&(fd)); \
		if(log_len) { \
			log_len = kfifo_out(&(fd), temp, sizeof(temp)); \
			printk(KERN_EMERG "\n[SCD][INT](%d) begin dump INT LOG: tx(%d), log(%d):\n%s[SCD][INT] end dump INT LOG\n", __LINE__, tx_len, log_len, temp); \
		} \
	} while(0);
#else

#define SC_TX_INT_DBG(fd, fmt, args...)
#define SC_TX_INT_DUMP(fd, tx_len)

#endif // CONFIG_SCD_TX_DBG

#endif  //__SCD_DEBUG_H__
