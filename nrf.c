#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "nrf.h"
#include "nrf24l01.h"

#ifdef NRF_CFG_IRQ_MODE
volatile uint8_t nrf_irq_flag;
#endif /* NRF_CFG_IRQ_MODE */

#ifdef NRF_CFG_IRQ_MODE
/*
 * Interrupt service routine
 *
 * Sets nrf_irq_flag only so nrf_irq() can handle this later.
 */
ISR(NRF_CFG_INT_VEC)
{
	nrf_irq_flag = 1;
}

/*
 * Handle nRF interrupt
 *
 * Resets the nRF's status register and goes back to standby I
 * and, depending on configuration, powers down.
 * Check the returned status-byte if you need more control,
 * this also contains the index of the rx-pipe.
 */
uint8_t nrf_irq()
{
	uint8_t irq = 0;

	if(nrf_irq_flag) {
		nrf_irq_flag = 0;
	
		uint8_t fifo;

		SPI_CS_LOW();
		irq = spi_rwb(NRF_C_R_REGISTER | NRF_R_FIFO_STATUS);
		fifo = spi_rwb(0);
		SPI_CS_HIGH();
		
		if(irq & NRF_R_STATUS_MAX_RT) {
			SPI_CE_LOW();
#ifdef NRF_CFG_IRQ_TE_FLUSH
			nrf_cmd(NRF_C_FLUSH_TX);
#endif
#ifdef NRF_CFG_IRQ_TE_PWR_DOWN
			nrf_power(0);
#endif
		} else if(irq & NRF_R_STATUS_TX_DS) {
			if(fifo & NRF_R_FIFO_STATUS_TX_EMPTY) {
				SPI_CE_LOW();
#ifdef NRF_CFG_IRQ_TX_PWR_DOWN
				nrf_power(0);
#endif
			}
		}

#ifdef NRF_CFG_IRQ_RX_PWR_DOWN
		if(irq & NRF_R_STATUS_RX_DR) {
			SPI_CE_LOW();
			nrf_power(0);
		}
#endif
		nrf_rwcmd(NRF_C_W_REGISTER | NRF_R_STATUS,
				irq & (NRF_R_STATUS_MAX_RT
				| NRF_R_STATUS_TX_DS
				| NRF_R_STATUS_RX_DR));
	}

	return irq;
}
#endif /* NRF_CFG_IRQ_MODE */


#if defined(__AVR_ATtiny2313__)
/*
 * Write + read one byte on SPI
 */
uint8_t spi_rwb(uint8_t in)
{
	USIDR = in;
	USISR = (1 << USIOIF);
	do {
		USICR |= (1 << USITC);
	} while(!(USISR & (1 << USIOIF)));
	return USIDR;
}

#elif defined(__AVR_ATmega8__)

uint8_t spi_rwb(uint8_t in)
{
	SPDR = in;
	while(!(SPSR & (1 << SPIF)));
	return SPDR;
}

#endif

/*
 * Write/read larger chuck of data on SPI
 */
void spi_rw(uint8_t cmd, uint8_t *in, uint8_t *out, uint8_t len)
{
	uint8_t wp = 0;

	SPI_CS_LOW();
	spi_rwb(cmd);
	while(wp < len) {
		cmd = spi_rwb((in)?(in[wp]):(0x0));
		if(out) out[wp] = cmd;
		wp++;
	}
	SPI_CS_HIGH();
}

/*
 * Single-byte command for nRF (NOP, FLUSH_RX/TX)
 */
uint8_t nrf_cmd(uint8_t cmd)
{
	uint8_t reg;

	SPI_CS_LOW();
	reg = spi_rwb(cmd);
	SPI_CS_HIGH();

	return reg;
}

/*
 * Two-byte command (eg. set registers)
 */
uint8_t nrf_rwcmd(uint8_t cmd, uint8_t val)
{
	SPI_CS_LOW();
	spi_rwb(cmd);
	val = spi_rwb(val);
	SPI_CS_HIGH();

	return val;
}

/*
 * Power up/down chip
 */
void nrf_power(uint8_t flag)
{
	uint8_t reg;

	/* power down <-> standby I */
	reg = nrf_rwcmd(NRF_C_R_REGISTER | NRF_R_CONFIG, 0);
	if(flag)
		reg |= NRF_R_CONFIG_PWR_UP;
	else
		reg &= ~NRF_R_CONFIG_PWR_UP;
	nrf_rwcmd(NRF_C_W_REGISTER | NRF_R_CONFIG, reg);
}

#ifndef NRF_CFG_IRQ_MODE
/* 
 * In non-IRQ-mode this is needed to make sure all data
 * is sent before going back to standby I.
 */
void nrf_flush()
{
	uint8_t irq, fifo;
	
	SPI_CS_LOW();
	irq = spi_rwb(NRF_C_R_REGISTER | NRF_R_FIFO_STATUS);
	fifo = spi_rwb(0);
	SPI_CS_HIGH();

	while(!(fifo & NRF_R_FIFO_STATUS_TX_EMPTY)) {
		if(irq & NRF_R_STATUS_MAX_RT) {
			nrf_rwcmd(NRF_C_W_REGISTER | NRF_R_STATUS,
				  NRF_R_STATUS_MAX_RT);
			
			nrf_cmd(NRF_C_FLUSH_TX);
		}
		
		SPI_CS_LOW();
		irq = spi_rwb(NRF_C_R_REGISTER | NRF_R_FIFO_STATUS);
		fifo = spi_rwb(0);
		SPI_CS_HIGH();
	}
	
	SPI_CE_LOW();
}
#endif /* NRF_CFG_IRQ_MODE */

/*
 * Put nRF into active PRX mode.
 */
void nrf_recv_start()
{
	uint8_t reg;

	/* configure for rx mode */
	reg = nrf_rwcmd(NRF_C_R_REGISTER | NRF_R_CONFIG, 0);
	reg |= NRF_R_CONFIG_PRIM_RX;
	nrf_rwcmd(NRF_C_W_REGISTER | NRF_R_CONFIG, reg);

	/* standby II? */
	SPI_CE_HIGH();
}

void nrf_recv_stop()
{
	uint8_t reg;

	SPI_CE_LOW();
	
	reg = nrf_rwcmd(NRF_C_R_REGISTER | NRF_R_CONFIG, 0);
	reg &= ~NRF_R_CONFIG_PRIM_RX;
	nrf_rwcmd(NRF_C_W_REGISTER | NRF_R_CONFIG, reg);
}
/*
 * Send data to chip and start active PTX mode.
 *
 * If no buffer space is left returns 0, else 1.
 *
 * If NRF_CFG_DYN_ACK is set a negative length will modify
 * this particular packet to not require an ACK even if
 * Auto-ACK feature is configured.
 */
uint8_t nrf_write(uint8_t *data, int8_t len)
{
	uint8_t irq;
	
	/* standby II */
	SPI_CE_HIGH();	

	irq = nrf_cmd(NRF_C_NOP);

	/* write new data if there's some space in the FIFOs */
	if(!(irq & NRF_R_STATUS_TX_FULL)) {
#ifdef NRF_CFG_DYN_ACK
		if(len > 0)
			spi_rw(NRF_C_W_TX_PAYLOAD, data, NULL, len);
		else
			spi_rw(NRF_C_W_TX_PAYLOAD_NOACK, data, NULL, -len);
#else
		spi_rw(NRF_C_W_TX_PAYLOAD, data, NULL, len);
#endif /* NRF_CFG_DYN_ACK */

		return 1;
	}

	return 0;
}

/*
 * Read RX data from chip
 *
 * Returns length of data packet.
 */
uint8_t nrf_read(uint8_t *data)
{
	uint8_t fifo, len = 0;

	fifo = nrf_rwcmd(NRF_C_R_REGISTER | NRF_R_FIFO_STATUS, 0);

	/* if there's data in the fifos... */
	if(!(fifo & NRF_R_FIFO_STATUS_RX_EMPTY)) {
		/* get length of the top fifo element */
		len = nrf_rwcmd(NRF_C_R_RX_PL_WID, 0);
		
		/* if len > 32 something odd happened */
		if(len > 32) {
			nrf_cmd(NRF_C_FLUSH_RX);
			len = 0;
		} else
			spi_rw(NRF_C_R_RX_PAYLOAD, NULL, data, len);
	}

	return len;
}

#ifdef NRF_CFG_ADDR_FAST
void nrf_tx_addr(uint8_t *mac, uint8_t len)
{
	spi_rw(NRF_C_W_REGISTER | NRF_R_TX_ADDR, mac, NULL,
		len);
	
	nrf_rwcmd(NRF_C_W_REGISTER | NRF_R_SETUP_AW,
		NRF_R_SETUP_AW_AW(len));
}

void nrf_rx_addr(uint8_t *mac, uint8_t len, uint8_t pipe)
{
	spi_rw(NRF_C_W_REGISTER | NRF_R_RX_ADDR_P(pipe), mac, NULL,
		NRF_R_RX_ADDR_P_LEN(pipe, len));
}
#endif /* NRF_CFG_ADDR_FAST */


#ifdef NRF_CFG_EEPROM_INIT
void nrf_init(uint8_t *data)
{
	uint8_t reg, val, off = 0;

	while((reg = eeprom_read_byte(&data[off++])) != 0xff)
	{
		val = eeprom_read_byte(&data[off++]);
		nrf_rwcmd(NRF_C_W_REGISTER | reg, val);
	}

	nrf_cmd(NRF_C_FLUSH_TX);
	nrf_cmd(NRF_C_FLUSH_RX);
}

#ifndef NRF_CFG_ADDR_FAST
void nrf_tx_addr(uint8_t *buf)
{
	uint8_t addr[NRF_CFG_ADDR_WIDTH];

	/* read and set tx-addr */
	eeprom_read_block(addr, buf,
		NRF_CFG_ADDR_WIDTH);
	
	spi_rw(NRF_C_W_REGISTER | NRF_R_TX_ADDR, addr, NULL,
		NRF_CFG_ADDR_WIDTH);
}

void nrf_rx_addr(uint8_t *buf, uint8_t pipe)
{
	uint8_t addr[NRF_CFG_ADDR_WIDTH];

	eeprom_read_block(addr, buf,
		NRF_R_RX_ADDR_P_LEN(pipe, NRF_CFG_ADDR_WIDTH));

	spi_rw(NRF_C_W_REGISTER | NRF_R_RX_ADDR_P(pipe), addr, NULL,
		NRF_R_RX_ADDR_P_LEN(pipe, NRF_CFG_ADDR_WIDTH));
}
#endif /* NRF_CFG_ADDR_FAST */


#define NRF_SET_CHANNEL(v)	\
	nrf_rwcmd(NRF_C_W_REGISTER | NRF_R_RF_CH, NRF_R_RF_CH_RF_CH(v));
#else /* NRF_CFG_EEPROM_INIT */

void nrf_init_pipe(uint8_t num)
{
	uint8_t reg;

	/* set max packet size for pipe */
	nrf_rwcmd(NRF_C_W_REGISTER | NRF_R_RX_PW_P(num),
		nrf_pipe.rxlen);

	/* set receive address */
	spi_rw(NRF_C_W_REGISTER | NRF_R_RX_ADDR_P(num),
		nrf_pipe.rxmac, NULL,
		nrf_pipe.maclen);
	
	/* enable/disable pipe */
	reg = nrf_rwcmd(NRF_C_R_REGISTER | NRF_R_EN_RXADDR, 0);
	if(nrf_pipe.flags & NRF_PIPE_EN)
		reg |= NRF_R_EN_RXADDR_ERX_P(num);
	else
		reg &= ~NRF_R_EN_RXADDR_ERX_P(num);
	nrf_rwcmd(NRF_C_W_REGISTER | NRF_R_EN_RXADDR, reg);
	
	/* enable/disable auto-ack */
	reg = nrf_rwcmd(NRF_C_R_REGISTER | NRF_R_EN_AA, 0);
	if(nrf_pipe.flags & NRF_PIPE_AA)
		reg |= NRF_R_EN_AA_ENAA_P(num);
	else
		reg &= ~NRF_R_EN_AA_ENAA_P(num);
	nrf_rwcmd(NRF_C_W_REGISTER | NRF_R_EN_AA, reg);
	
	/* enable/disable dynamic packet lengths */
	reg = nrf_rwcmd(NRF_C_R_REGISTER | NRF_R_DYNPD, 0);
	if(nrf_pipe.flags & NRF_PIPE_DPL) {
		reg |= NRF_R_DYNPD_DPL_P(num);
	} else
		reg &= ~NRF_R_DYNPD_DPL_P(num);
	nrf_rwcmd(NRF_C_W_REGISTER | NRF_R_DYNPD, reg);
}

void nrf_init()
{
	/* set global address width */
	nrf_rwcmd(NRF_C_W_REGISTER | NRF_R_SETUP_AW,
		NRF_R_SETUP_AW_AW(nrf_dev.maclen));
	
	/* set tx address */
	spi_rw(NRF_C_W_REGISTER | NRF_R_TX_ADDR,
		nrf_dev.txmac, NULL,
		nrf_dev.maclen);	

	/* set channel */
	nrf_rwcmd(NRF_C_W_REGISTER | NRF_R_RF_CH,
		nrf_dev.channel);
	
	/* set auto ack timeout and maximum retries */
	nrf_rwcmd(NRF_C_W_REGISTER | NRF_R_SETUP_RETR,
		  NRF_R_SETUP_RETR_ARD(nrf_dev.ard)
		| NRF_R_SETUP_RETR_ARC(nrf_dev.arc));
	
	/* set data rate and tx power */
	nrf_rwcmd(NRF_C_W_REGISTER | NRF_R_RF_SETUP,
		nrf_dev.rate
		| NRF_R_RF_SETUP_RF_PWR(nrf_dev.txpwr));
	
	/* set crc mode */
	nrf_rwcmd(NRF_C_W_REGISTER | NRF_R_CONFIG,
		((nrf_dev.crc > 0)?(NRF_R_CONFIG_EN_CRC):0)
		| ((nrf_dev.crc > 1)?(NRF_R_CONFIG_CRCO):0));

	/* disable all data pipes */
	nrf_rwcmd(NRF_C_W_REGISTER | NRF_R_EN_RXADDR,
		NRF_R_EN_RXADDR_ERX_NONE);
	
	/* disable auto ack */
	nrf_rwcmd(NRF_C_W_REGISTER | NRF_R_EN_AA,
		NRF_R_EN_AA_ENAA_NONE);
	
	/* disable dynamic payload length */
	nrf_rwcmd(NRF_C_W_REGISTER | NRF_R_DYNPD,
		NRF_R_DYNPD_DPL_NONE);
	
	/* just in case: clear max_rt... */
	nrf_rwcmd(NRF_C_W_REGISTER | NRF_R_STATUS,
		NRF_R_STATUS_MAX_RT);

	nrf_rwcmd(NRF_C_W_REGISTER | NRF_R_FEATURE,
		  ((nrf_dev.flags & NRF_DPL)?NRF_R_FEATURE_EN_DPL:0)
		| ((nrf_dev.flags & NRF_DYN_ACK)?NRF_R_FEATURE_EN_DYN_ACK:0)
		| ((nrf_dev.flags & NRF_ACK_PAY)?NRF_R_FEATURE_EN_ACK_PAY:0));

	/* ...and flush the buffers */
	nrf_cmd(NRF_C_FLUSH_TX);
	nrf_cmd(NRF_C_FLUSH_RX);
}
#endif /* NRF_CFG_EEPROM_INIT */
