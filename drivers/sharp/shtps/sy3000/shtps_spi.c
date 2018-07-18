/* drivers/sharp/shtps/sy3000/shtps_spi.c
 *
 * Copyright (c) 2014, Sharp. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>

/* -------------------------------------------------------------------------- */
#include <sharp/shtps_dev.h>

#include "shtps_param_extern.h"
#include "shtps_log.h"
#include "shtps_spi.h"

/* -------------------------------------------------------------------------- */
#if defined( SHTPS_LOG_ERROR_ENABLE )
	#define SHTPS_LOG_SPI_FUNC_CALL()	// SHTPS_LOG_FUNC_CALL()
#else
	#define SHTPS_LOG_SPI_FUNC_CALL()
#endif
/* -------------------------------------------------------------------------- */
struct shtps_rmi_spi;

static int shtps_spi_write_fw_data(void *spi, u16 addr, u8 *buf, u16 blockSize);
static void shtps_spi_write_async_spicomplete(void *arg);
static int shtps_spi_write_async(void *spi, u16 addr, u8 data);
static int shtps_spi_write_block(void *spi, u16 addr, u8 *data, u32 size);
static int shtps_spi_write(void *spi, u16 addr, u8 data);

static int shtps_spi_read_block(void *spi, u16 addr, u8 *readbuf, u32 size, u8 *buf);
static int shtps_spi_read(void *spi, u16 addr, u8 *buf, u32 size);
#if	defined( SHTPS_SPI_BLOCKACCESS_ENABLE )
	static void shtps_spi_one_cs_spicomplete(void *arg);
#else	/* SHTPS_SPI_BLOCKACCESS_ENABLE */
	static void shtps_spi_readb_async_comp_1(void *arg);
	static void shtps_spi_readb_async_comp_2(void *arg);
	static int shtps_spi_readb(void *spi, u16 addr, u8 *buf);
#endif /* #if defined( SHTPS_SPI_BLOCKACCESS_ENABLE ); */

static int shtps_spi_write_packet(void *spi, u16 addr, u8 *data, u32 size);
static int shtps_spi_read_packet (void *spi, u16 addr, u8 *buf,  u32 size);
static int shtps_spi_active(void *spi);
static int shtps_spi_standby(void *spi);

/* -------------------------------------------------------------------------- */
//#if defined( SHTPS_SPI_FWBLOCKWRITE_ENABLE )
static int shtps_spi_write_fw_data(void *spi, u16 addr, u8 *buf, u16 blockSize)
{
	struct spi_message	message;
#ifdef	SHTPS_SPI_FAST_FW_TRANSFER_ENABLE
	struct spi_transfer	t;
	u8					tx_buf[20];
#else
	struct spi_transfer	t[17];
	u8					tx_buf[2];
#endif	/* SHTPS_SPI_FAST_FW_TRANSFER_ENABLE */
	int					status = 0;
	int					i;
	int					retry = SHTPS_SPI_RETRY_COUNT;

	SHTPS_LOG_SPI_FUNC_CALL();
	do{
#ifdef	SHTPS_SPI_FAST_FW_TRANSFER_ENABLE
		memset(&t, 0, sizeof(t));

		tx_buf[0] = (addr >> 8) & 0xff;
		tx_buf[1] = addr & 0xff;

		for(i = 0; i < blockSize; i++){
			tx_buf[2+i] = buf[i];
		}

		spi_message_init(&message);

		t.tx_buf 		= tx_buf;
		t.rx_buf 		= NULL;
		t.len    		= 2+blockSize;
		t.speed_hz		= SHTPS_SY3X00_SPI_CLOCK_WRITE_SPEED;
		#if defined(CONFIG_SPI_DEASSERT_WAIT_SH)
			t.deassert_wait	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
		#else
			t.delay_usecs	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
		#endif /* CONFIG_SPI_DEASSERT_WAIT_SH */
		spi_message_add_tail(&t, &message);

		#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
			_log_msg_sync(LOGMSG_ID__SPI_WRITE, "0x%04X|0x%02X", addr, data);
		#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
			status = spi_sync((struct spi_device *)spi, &message);
		#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
			_log_msg_sync(LOGMSG_ID__SPI_WRITE_COMP, "%d", status);
		#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
#else
		memset(&t, 0, sizeof(t));

		tx_buf[0] = (addr >> 8) & 0xff;
		tx_buf[1] = addr & 0xff;

		spi_message_init(&message);

		t[0].tx_buf 		= tx_buf;
		t[0].rx_buf 		= NULL;
		t[0].len    		= 2;
		t[0].speed_hz		= SHTPS_SY3X00_SPI_CLOCK_WRITE_SPEED;
		#if defined(CONFIG_SPI_DEASSERT_WAIT_SH)
			t[0].deassert_wait	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
		#else
			t[0].delay_usecs	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
		#endif /* CONFIG_SPI_DEASSERT_WAIT_SH */
		spi_message_add_tail(&t[0], &message);

		for(i = 0; i < blockSize; i++){
			t[i+1].tx_buf 		= &buf[i];
			t[i+1].rx_buf 		= NULL;
			t[i+1].len    		= 1;
			t[i+1].speed_hz		= SHTPS_SY3X00_SPI_CLOCK_WRITE_SPEED;
			#if defined(CONFIG_SPI_DEASSERT_WAIT_SH)
				t[i+1].deassert_wait	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
			#else
				t[i+1].delay_usecs	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
			#endif /* CONFIG_SPI_DEASSERT_WAIT_SH */

			spi_message_add_tail(&t[i+1], &message);
		}

		#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
			_log_msg_sync(LOGMSG_ID__SPI_WRITE, "0x%04X|0x%02X", addr, data);
		#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
		status = spi_sync((struct spi_device *)spi, &message);
		#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
			_log_msg_sync(LOGMSG_ID__SPI_WRITE_COMP, "%d", status);
		#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
#endif	/* SHTPS_SPI_FAST_FW_TRANSFER_ENABLE */
		if(status){
			struct timespec tu;
			tu.tv_sec = (time_t)0;
			tu.tv_nsec = SHTPS_SPI_RETRY_WAIT * 1000000;
			hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
		}
	}while(status != 0 && retry-- > 0);

	if(status){
		SHTPS_LOG_ERR_PRINT("spi write error\n");
	}
	#if defined( SHTPS_LOG_DEBUG_ENABLE )
	else if(retry < (SHTPS_SPI_RETRY_COUNT)){
		SHTPS_LOG_DBG_PRINT("spi retry num = %d\n", SHTPS_SPI_RETRY_COUNT - retry);
	}
	#endif /* SHTPS_LOG_DEBUG_ENABLE */
	return status;
}
//#endif /* #if defined( SHTPS_SPI_FWBLOCKWRITE_ENABLE ) */

/* -----------------------------------------------------------------------------------
 */
static void shtps_spi_write_async_spicomplete(void *arg)
{
	complete(arg);
	udelay(SHTPS_SY3X00_SPI_TANSACTION_WRITE_WAIT);
}
static int shtps_spi_write_async(void *spi, u16 addr, u8 data)
{
	DECLARE_COMPLETION_ONSTACK(done);
	struct spi_message	message;
	struct spi_transfer	t;
	u8					tx_buf[3];
	int					status;

	SHTPS_LOG_SPI_FUNC_CALL();
	memset(&t, 0, sizeof(t));

	spi_message_init(&message);
	spi_message_add_tail(&t, &message);
	t.tx_buf 		= tx_buf;
	t.rx_buf 		= NULL;
	t.len    		= 3;
	t.speed_hz		= SHTPS_SY3X00_SPI_CLOCK_WRITE_SPEED;
	#if defined(CONFIG_SPI_DEASSERT_WAIT_SH)
		t.deassert_wait	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
	#else
		t.delay_usecs	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
	#endif /* CONFIG_SPI_DEASSERT_WAIT_SH */

	tx_buf[0] = (addr >> 8) & 0xff;
	tx_buf[1] = addr & 0xff;
	tx_buf[2] = data;

	#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
		_log_msg_sync(LOGMSG_ID__SPI_WRITE, "0x%04X|0x%02X", addr, data);
	#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */

	message.complete = shtps_spi_write_async_spicomplete;
	message.context  = &done;
	status = spi_async((struct spi_device *)spi, &message);
	if(status == 0){
		wait_for_completion(&done);
		status = message.status;
	}
	message.context = NULL;

	#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
		_log_msg_sync(LOGMSG_ID__SPI_WRITE_COMP, "%d", status);
	#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */

	return status;
}

#if defined(SHTPS_SPI_BLOCKACCESS_ENABLE)
static int shtps_spi_write_block(void *spi, u16 addr, u8 *data, u32 size)
{
	struct spi_message	message;
	struct spi_transfer	t_local[1 + SHTPS_SY3X00_SPIBLOCKWRITE_BUFSIZE];
	struct spi_transfer	*t;
	u8					tx_buf[2];
	int					status;
	int 				i;
	int 				allocSize = 0;

	SHTPS_LOG_SPI_FUNC_CALL();
	if(size > SHTPS_SY3X00_SPIBLOCKWRITE_BUFSIZE){
		allocSize = sizeof(struct spi_transfer) * (1 + size);
		t = (struct spi_transfer *)kzalloc(allocSize, GFP_KERNEL);
		if(t == NULL){
			SHTPS_LOG_DBG_PRINT("shtps_spi_write_block() alloc error. size = %d\n", allocSize);
			return -ENOMEM;
		}
		memset(t, 0, allocSize);
	} else {
		t = t_local;
		memset(t, 0, sizeof(t_local));
	}

	tx_buf[0] = (addr >> 8) & 0xff;
	tx_buf[1] = addr & 0xff;

	spi_message_init(&message);

	t[0].tx_buf 		= tx_buf;
	t[0].rx_buf 		= NULL;
	t[0].len    		= 2;
	t[0].speed_hz		= SHTPS_SY3X00_SPI_CLOCK_WRITE_SPEED;
	#if defined(CONFIG_SPI_DEASSERT_WAIT_SH)
		t[0].deassert_wait	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
	#else
		t[0].delay_usecs	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
	#endif /* CONFIG_SPI_DEASSERT_WAIT_SH */

	spi_message_add_tail(&t[0], &message);

	for(i = 0; i < size; i++){
		t[i+1].tx_buf 		= &data[i];
		t[i+1].rx_buf 		= NULL;
		t[i+1].len    		= 1;
		t[i+1].speed_hz		= SHTPS_SY3X00_SPI_CLOCK_WRITE_SPEED;
		#if defined(CONFIG_SPI_DEASSERT_WAIT_SH)
			t[i+1].deassert_wait	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
		#else
			t[i+1].delay_usecs	= SHTPS_SY3X00_SPI_CLOCK_WRITE_WAIT;
		#endif /* CONFIG_SPI_DEASSERT_WAIT_SH */
		spi_message_add_tail(&t[i+1], &message);
	}

	status = spi_sync((struct spi_device *)spi, &message);

	if(allocSize > 0){
		kfree(t);
	}

	return status;
}
#else
static int shtps_spi_write_block(void *spi, u16 addr, u8 *data, u32 size)
{
	SHTPS_LOG_ERR_PRINT("%s() : no function!\n", __func__);
}
#endif /* SHTPS_SPI_BLOCKACCESS_ENABLE */

static int shtps_spi_write(void *spi, u16 addr, u8 data)
{
	int retry = SHTPS_SPI_RETRY_COUNT;
	int err;

	SHTPS_LOG_SPI_FUNC_CALL();
	do{
		err = shtps_spi_write_async(spi, addr, data);
		if(err){
			struct timespec tu;
			tu.tv_sec = (time_t)0;
			tu.tv_nsec = SHTPS_SPI_RETRY_WAIT * 1000000;
			hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
		}
	}while(err != 0 && retry-- > 0);

	if(err){
		SHTPS_LOG_ERR_PRINT("spi write error\n");
	}
	#if defined( SHTPS_LOG_DEBUG_ENABLE )
	else if(retry < (SHTPS_SPI_RETRY_COUNT)){
		SHTPS_LOG_DBG_PRINT("spi retry num = %d\n", SHTPS_SPI_RETRY_COUNT - retry);
	}
	#endif /* SHTPS_LOG_DEBUG_ENABLE */

	return err;
}

#if	defined( SHTPS_SPI_BLOCKACCESS_ENABLE )
static void shtps_spi_one_cs_spicomplete(void *arg)
{
	complete(arg);
	udelay(SHTPS_SY3X00_SPI_TANSACTION_READ_WAIT);
}
static int shtps_spi_read_block(void *spi, u16 addr, u8 *readbuf, u32 size, u8 *buf)
{
	DECLARE_COMPLETION_ONSTACK(done);
	struct spi_message	message;
	struct spi_transfer	t;
	u8 *txbuf_p;
	u8 *rxbuf_p;
	int status;
	int i;

	SHTPS_LOG_SPI_FUNC_CALL();
	memset(&t, 0, sizeof(t));
	memset(buf, 0, (size+2*2));

	txbuf_p = buf;
	rxbuf_p = buf + sizeof(u8)*(size + 2);

	txbuf_p[0] = ((addr >> 8) & 0xff) | 0x80;
	txbuf_p[1] = addr & 0xff;

	spi_message_init(&message);
	spi_message_add_tail(&t, &message);

	t.tx_buf		= txbuf_p;
	t.rx_buf  		= rxbuf_p;
	t.len			= size + 2;
	t.bits_per_word	= 8;
	t.speed_hz		= SHTPS_SY3X00_SPI_CLOCK_READ_SPEED;
	#if defined(CONFIG_SPI_DEASSERT_WAIT_SH)
		t.deassert_wait	= SHTPS_SY3X00_SPI_CLOCK_READ_WAIT;
	#else
		t.delay_usecs	= SHTPS_SY3X00_SPI_CLOCK_READ_WAIT;
	#endif /* CONFIG_SPI_DEASSERT_WAIT_SH */

	message.complete = shtps_spi_one_cs_spicomplete;
	message.context  = &done;
	status = spi_async((struct spi_device *)spi, &message);
	if(status == 0){
		wait_for_completion(&done);
		status = message.status;
	}
	message.context = NULL;

	rxbuf_p += 2;
	for(i = 0;i < size;i++){
		*readbuf++ = *rxbuf_p++;
	}

	return status;
}
static int shtps_spi_read(void *spi, u16 addr, u8 *buf, u32 size)
{
	int status = 0;
	int	i;
	u32	s;
	u8 transbuf[(SHTPS_SY3X00_SPIBLOCK_BUFSIZE+2)*2];
	int retry = SHTPS_SPI_RETRY_COUNT;

	SHTPS_LOG_SPI_FUNC_CALL();
	do{
		s = size;
		for(i=0;i<size;i+=SHTPS_SY3X00_SPIBLOCK_BUFSIZE){
			status = shtps_spi_read_block(spi,
				addr+i,
				buf+i,
				(s>SHTPS_SY3X00_SPIBLOCK_BUFSIZE)?(SHTPS_SY3X00_SPIBLOCK_BUFSIZE):(s),
				transbuf);
			//
			if(status){
				struct timespec tu;
				tu.tv_sec = (time_t)0;
				tu.tv_nsec = SHTPS_SPI_RETRY_WAIT * 1000000;
				hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
				break;
			}
			s -= SHTPS_SY3X00_SPIBLOCK_BUFSIZE;
		}
	}while(status != 0 && retry-- > 0);

	if(status){
		SHTPS_LOG_ERR_PRINT("spi read error\n");
	}
	#if defined( SHTPS_LOG_DEBUG_ENABLE )
	else if(retry < (SHTPS_SPI_RETRY_COUNT)){
		SHTPS_LOG_DBG_PRINT("spi retry num = %d\n", SHTPS_SPI_RETRY_COUNT - retry);
	}
	#endif /* SHTPS_LOG_DEBUG_ENABLE */
	
	return status;
}
#else	/* SHTPS_SPI_BLOCKACCESS_ENABLE */
/* -----------------------------------------------------------------------------------
 */
struct shtps_spi_async_ctrl_tbl{
	struct completion		work;
	struct spi_message		message;
	struct spi_transfer		t;
	int						wait_utime;
	u8						tx_buf[2];
	u8						rx_buf[2];
};

/* -----------------------------------------------------------------------------------
 */
static void shtps_spi_readb_async_comp_1(void *arg)
{
	struct shtps_spi_async_ctrl_tbl	*ptr_p;
	int	wait_utime;

	ptr_p = (struct shtps_spi_async_ctrl_tbl *)arg;
	wait_utime = ptr_p->wait_utime;

	if(wait_utime){
		udelay( wait_utime );
	}
}

/* -----------------------------------------------------------------------------------
 */
static void shtps_spi_readb_async_comp_2(void *arg)
{
	struct shtps_spi_async_ctrl_tbl	*ptr_p;
	int	wait_utime;

	ptr_p = (struct shtps_spi_async_ctrl_tbl *)arg;
	wait_utime = ptr_p->wait_utime;

	complete( &(ptr_p->work) );

	if(wait_utime){
		udelay( wait_utime );
	}
}
/* -----------------------------------------------------------------------------------
 */
static int shtps_spi_readb(void *spi, u16 addr, u8 *buf)
{
	struct	shtps_spi_async_ctrl_tbl	spi_work[2], *spi_p;

	int	status;

	*buf = 0;
	memset(spi_work, 0, sizeof(spi_work));

	/* Write Reg.Addr. */
	spi_p = &spi_work[0];

	spi_message_init( &(spi_p->message) );
	spi_message_add_tail( &(spi_p->t), &(spi_p->message));

	spi_p->wait_utime		= SHTPS_SY3X00_SPI_TANSACTION_READ_WAIT;
	spi_p->tx_buf[0]		= ((addr >> 8) & 0xff) | 0x80;
	spi_p->tx_buf[1]		= addr & 0xff;
	spi_p->t.tx_buf			= spi_p->tx_buf;
	spi_p->t.rx_buf			= NULL;
	spi_p->t.bits_per_word	= 8;
	spi_p->t.len			= 2;
	spi_p->t.speed_hz		= SHTPS_SY3X00_SPI_CLOCK_READ_SPEED;
	#if defined(CONFIG_SPI_DEASSERT_WAIT_SH)
		spi_p->t.deassert_wait	= SHTPS_SY3X00_SPI_CLOCK_READ_WAIT;
	#else
		spi_p->t.delay_usecs	= SHTPS_SY3X00_SPI_CLOCK_READ_WAIT;
	#endif /* CONFIG_SPI_DEASSERT_WAIT_SH */
	spi_p->message.complete	= shtps_spi_readb_async_comp_1;
	spi_p->message.context	= spi_p;

	#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
		_log_msg_sync(LOGMSG_ID__SPI_WRITEADDR, "0x%04X", addr);
	#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
	status = spi_async((struct spi_device *)spi, &(spi_p->message));
	#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
		_log_msg_sync(LOGMSG_ID__SPI_WRITEADDR_COMP, "%d", status);
	#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
	if(status){
		return status;
	}

	/* Read data */
	spi_p = &spi_work[1];

	init_completion( &(spi_p->work) );

	spi_message_init( &(spi_p->message) );
	spi_message_add_tail( &(spi_p->t), &(spi_p->message));

	spi_p->wait_utime		= SHTPS_SY3X00_SPI_TANSACTION_READ_WAIT;
	spi_p->t.tx_buf			= NULL;
	spi_p->t.rx_buf			= spi_p->rx_buf;
	spi_p->t.bits_per_word	= 8;
	spi_p->t.len			= 1;
	spi_p->t.speed_hz		= SHTPS_SY3X00_SPI_CLOCK_READ_SPEED;
	#if defined(CONFIG_SPI_DEASSERT_WAIT_SH)
		spi_p->t.deassert_wait	= SHTPS_SY3X00_SPI_CLOCK_READ_WAIT;
	#else
		spi_p->t.delay_usecs	= SHTPS_SY3X00_SPI_CLOCK_READ_WAIT;
	#endif /* CONFIG_SPI_DEASSERT_WAIT_SH */
	spi_p->message.complete	= shtps_spi_readb_async_comp_2;
	spi_p->message.context	= spi_p;

	#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
		_log_msg_sync(LOGMSG_ID__SPI_READ, "0x%04X", addr);
	#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
	status = spi_async((struct spi_device *)spi, &(spi_p->message));
	#ifdef SHTPS_LOG_SPIACCESS_SEQ_ENABLE
		_log_msg_sync(LOGMSG_ID__SPI_READ_COMP, "0x%04X|0x%02X", addr, rx_buf[0]);
	#endif /* SHTPS_LOG_SPIACCESS_SEQ_ENABLE */
	if(status == 0){
		wait_for_completion( &(spi_p->work) );
		status = spi_p->message.status;

	}else{
		return status;

	}

	*buf = spi_p->rx_buf[0];

	return status;
}
/* -----------------------------------------------------------------------------------
 */
static int shtps_spi_read_block(void *spi, u16 addr, u8 *readbuf, u32 size, u8 *buf)
{
	SHTPS_LOG_ERR_PRINT("%s() : no function!\n", __func__);
}
/* -----------------------------------------------------------------------------------
 */
static int shtps_spi_read(void *spi, u16 addr, u8 *buf, u32 size)
{
	int i;
	int status = 0;

	SHTPS_LOG_SPI_FUNC_CALL();
	for(i = 0;i < size;i++){
		int retry;
		retry = SHTPS_SPI_RETRY_COUNT;
		do{
			status = shtps_spi_readb(spi, addr + i, buf + i);
			if(status){
				struct timespec tu;
				tu.tv_sec = (time_t)0;
				tu.tv_nsec = SHTPS_SPI_RETRY_WAIT * 1000000;
				hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
			}
		}while(status != 0 && retry-- > 0);

		if(status){
			SHTPS_LOG_ERR_PRINT("spi read error\n");
			goto err_exit;
		}
		#if defined( SHTPS_LOG_DEBUG_ENABLE )
		else if(retry < (SHTPS_SPI_RETRY_COUNT)){
			SHTPS_LOG_DBG_PRINT("spi retry num = %d\n", SHTPS_SPI_RETRY_COUNT - retry);
		}
		#endif /* SHTPS_LOG_DEBUG_ENABLE */
	}
	return 0;

err_exit:
	return status;
}
#endif /* #if defined( SHTPS_SPI_BLOCKACCESS_ENABLE ) */

static int shtps_spi_write_packet(void *spi, u16 addr, u8 *data, u32 size)
{
	int status = 0;
	int retry = SHTPS_SPI_RETRY_COUNT;

	SHTPS_LOG_SPI_FUNC_CALL();
	do{
		status = shtps_spi_write_block(spi,
			addr,
			data,
			size);
		//
		if(status){
			struct timespec tu;
			tu.tv_sec = (time_t)0;
			tu.tv_nsec = SHTPS_SPI_RETRY_WAIT * 1000000;
			hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
		}
	}while(status != 0 && retry-- > 0);

	if(status){
		SHTPS_LOG_ERR_PRINT("spi write error\n");
	}
	#if defined( SHTPS_LOG_DEBUG_ENABLE )
	else if(retry < (SHTPS_SPI_RETRY_COUNT)){
		SHTPS_LOG_DBG_PRINT("spi retry num = %d\n", SHTPS_SPI_RETRY_COUNT - retry);
	}
	#endif /* SHTPS_LOG_DEBUG_ENABLE */

	return status;
}

static int shtps_spi_read_packet(void *spi, u16 addr, u8 *buf, u32 size)
{
	int status = 0;
	int	i;
	u32	s;
	u8 transbuf[(SHTPS_SY3X00_SPIBLOCK_BUFSIZE+2)*2];
	int retry = SHTPS_SPI_RETRY_COUNT;
	u16 addr_org = addr;

	SHTPS_LOG_SPI_FUNC_CALL();
	do{
		s = size;
		for(i=0;i<size;i+=SHTPS_SY3X00_SPIBLOCK_BUFSIZE){
			status = shtps_spi_read_block(spi,
				addr,
				buf+i,
				(s>SHTPS_SY3X00_SPIBLOCK_BUFSIZE)?(SHTPS_SY3X00_SPIBLOCK_BUFSIZE):(s),
				transbuf);
			//
			if(status){
				struct timespec tu;
				tu.tv_sec = (time_t)0;
				tu.tv_nsec = SHTPS_SPI_RETRY_WAIT * 1000000;
				hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
				addr = addr_org;
				break;
			}
			s -= SHTPS_SY3X00_SPIBLOCK_BUFSIZE;
			addr = 0x7fff;	/* specifying no address after first read */
		}
	}while(status != 0 && retry-- > 0);

	if(status){
		SHTPS_LOG_ERR_PRINT("spi read error\n");
	}
	#if defined( SHTPS_LOG_DEBUG_ENABLE )
	else if(retry < (SHTPS_SPI_RETRY_COUNT)){
		SHTPS_LOG_DBG_PRINT("spi retry num = %d\n", SHTPS_SPI_RETRY_COUNT - retry);
	}
	#endif /* SHTPS_LOG_DEBUG_ENABLE */

	return status;
}


/* -----------------------------------------------------------------------------------
*/
static int shtps_spi_active(void *spi)
{
	#if defined( SHTPS_SPICLOCK_CONTROL_ENABLE )
		extern	void sh_spi_tps_active(struct spi_device *spi);

		sh_spi_tps_active((struct spi_device *)spi);
		SHTPS_LOG_DBG_PRINT("spi active\n");
	#endif /* SHTPS_SPICLOCK_CONTROL_ENABLE */

	return 0;
}

/* -----------------------------------------------------------------------------------
*/
static int shtps_spi_standby(void *spi)
{
	#if defined( SHTPS_SPICLOCK_CONTROL_ENABLE )
		extern	void sh_spi_tps_standby(struct spi_device *spi);

		sh_spi_tps_standby((struct spi_device *)spi);
		SHTPS_LOG_DBG_PRINT("spi standby\n");
	#endif /* SHTPS_SPICLOCK_CONTROL_ENABLE */

	return 0;
}

/* -----------------------------------------------------------------------------------
*/
static const struct shtps_ctrl_functbl TpsCtrl_SpiFuncTbl = {
	.firmware_write_f = shtps_spi_write_fw_data,
	.block_write_f    = shtps_spi_write_block,
	.block_read_f     = shtps_spi_read_block,
	.write_f          = shtps_spi_write,
	.read_f           = shtps_spi_read,
	.packet_write_f   = shtps_spi_write_packet,
	.packet_read_f    = shtps_spi_read_packet,
	.active_f         = shtps_spi_active,
	.standby_f        = shtps_spi_standby,
};

/* -----------------------------------------------------------------------------------
*/
static int __devinit shtps_spi_probe(struct spi_device *spi_p)
{
	extern	int shtps_rmi_core_probe( struct device *ctrl_dev_p,
									struct shtps_ctrl_functbl *func_p,
									void *ctrl_p,
									char *modalias,
									int gpio_irq);
	int result;

	_log_msg_sync( LOGMSG_ID__PROBE, "");
	SHTPS_LOG_SPI_FUNC_CALL();

	/* ---------------------------------------------------------------- */
	result = spi_setup(spi_p);
	if(result < 0){
		SHTPS_LOG_DBG_PRINT("spi_setup fail\n");
		goto fail_err;
	}

	/* ---------------------------------------------------------------- */
	result = shtps_rmi_core_probe(
							&spi_p->dev,
							(struct shtps_ctrl_functbl*)&TpsCtrl_SpiFuncTbl,
							(void *)spi_p,
							spi_p->modalias,
							spi_p->irq );
	if(result){
		goto fail_err;
	}

	/* ---------------------------------------------------------------- */
	/*	no available
		spi_set_drvdata(spi_p, ts_p); -> dev_set_drvdata(&spi->dev, ts_p); 
	*/

	_log_msg_sync( LOGMSG_ID__PROBE_DONE, "");
	return 0;
	/* ---------------------------------------------------------------- */


fail_err:
	_log_msg_sync( LOGMSG_ID__PROBE_FAIL, "");
	return result;
}

/* -----------------------------------------------------------------------------------
*/
static int __devexit shtps_spi_remove(struct spi_device *spi_p)
{
	extern int shtps_rmi_core_remove(struct shtps_rmi_spi *, struct device *);
	struct shtps_rmi_spi *ts_p = spi_get_drvdata(spi_p);
	int ret = shtps_rmi_core_remove(ts_p, &spi_p->dev);

	_log_msg_sync( LOGMSG_ID__REMOVE_DONE, "");
	SHTPS_LOG_DBG_PRINT("shtps_spi_remove() done\n");
	return ret;
}

/* -----------------------------------------------------------------------------------
*/
static int shtps_spi_suspend(struct spi_device *spi_p, pm_message_t mesg)
{
	extern int shtps_rmi_core_suspend(struct shtps_rmi_spi *, pm_message_t);
	struct shtps_rmi_spi *ts_p = spi_get_drvdata(spi_p);
	int ret = shtps_rmi_core_suspend(ts_p, mesg);

	_log_msg_sync( LOGMSG_ID__SUSPEND, "");
	return ret;
}

/* -----------------------------------------------------------------------------------
*/
static int shtps_spi_resume(struct spi_device *spi_p)
{
	extern int shtps_rmi_core_resume(struct shtps_rmi_spi *);
	struct shtps_rmi_spi *ts_p = spi_get_drvdata(spi_p);
	int ret = shtps_rmi_core_resume(ts_p);

	_log_msg_sync( LOGMSG_ID__RESUME, "");
	return ret;
}

/* -----------------------------------------------------------------------------------
*/
#ifdef CONFIG_OF // Open firmware must be defined for dts useage
static struct of_device_id shtps_rmi_table[] = {
	{ . compatible = "sharp,shtps_rmi" ,}, // Compatible node must match dts
	{ },
};
#endif

static struct spi_driver shtps_spi_driver = {
	.probe		= shtps_spi_probe,
	.remove		= __devexit_p(shtps_spi_remove),
	.suspend	= shtps_spi_suspend,
	.resume		= shtps_spi_resume,
	.driver		= {
		#ifdef CONFIG_OF // Open firmware must be defined for dts useage
			.of_match_table = shtps_rmi_table,
		#endif
		.name = "shtps_rmi",	// SH_TOUCH_DEVNAME,
		.owner = THIS_MODULE,
	},
};

/* -----------------------------------------------------------------------------------
*/
static int __init shtps_spi_init(void)
{
	SHTPS_LOG_DBG_PRINT("shtps_spi_init() start\n");
	_log_msg_sync( LOGMSG_ID__INIT, "");

	return spi_register_driver(&shtps_spi_driver);
}
module_init(shtps_spi_init);

/* -----------------------------------------------------------------------------------
*/
static void __exit shtps_spi_exit(void)
{
	spi_unregister_driver(&shtps_spi_driver);

	SHTPS_LOG_DBG_PRINT("shtps_spi_exit() done\n");
	_log_msg_sync( LOGMSG_ID__EXIT, "");
}
module_exit(shtps_spi_exit);

/* -----------------------------------------------------------------------------------
*/
