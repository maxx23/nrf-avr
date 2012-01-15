#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>

#include "nrfcfg.h"

#ifdef NRF_CFG_EEPROM_INIT
#include <avr/eeprom.h>
#include "nrf24l01.h"
#endif

#ifndef __NRF_H__
#define __NRF_H__

#ifdef NRF_CFG_IRQ_MODE
#define NRF_IRQ_TE	0x10
#define NRF_IRQ_DS	0x20
#define NRF_IRQ_DR	0x40
#endif

#ifndef NRF_CFG_EEPROM_INIT
/* Define RAM-structure and option data for
 * RAM configuration mode
 */

/* features */
#define NRF_DYN_ACK	1
#define NRF_ACK_PAY	2
#define NRF_DPL		4

#define NRF_PIPE_EN	1
#define NRF_PIPE_DPL	2
#define NRF_PIPE_AA	4

/* data rates */
#define NRF_RATE_1M	0x00
#define NRF_RATE_2M	0x08
#define NRF_RATE_250K	0x20

/* pipe data structure */
struct nrf_pipe_s {
	uint8_t rxmac[5];			/* receive mac */
	uint8_t maclen;				/* length, must be 1 for pipe 2-6 */
	uint8_t flags;				/* en-/disable and special features */
	uint8_t rxlen;				/* max packet length */
};

/* nRF data structure */
struct nrf_context_s {
	uint8_t channel;			/* rf channel */
	uint8_t txmac[5];			/* transmit mac */
	uint8_t maclen;				/* length of tx-mac */
	uint8_t rate;				/* data rate */
	uint8_t txpwr;				/* transmit power */
	uint8_t ard;				/* auto retransmit delay */
	uint8_t arc;				/* auto retransmit count */
	uint8_t crc;				/* 0, 1 or 2 bytes crc */
	uint8_t flags;
};

extern struct nrf_context_s nrf_dev;
extern struct nrf_pipe_s    nrf_pipe;
#endif

/* EEPROM configuration is unstructured ;) */


#ifdef NRF_CFG_IRQ_MODE
#define NRF_INT_RX_P_NO(x)	(((x) >> 1) & 0x7)
#define NRF_INT_RX_EMPTY(x)	(((x) & 0x0e) == 0x0e)
#define NRF_INT_RX_NOT_USED(x)	(((x) & 0x0e) == 0x0c)

extern volatile uint8_t nrf_int_flag;
#endif /* NRF_IRQ_MODE */

/* public exports */
void nrf_send_start();

#ifdef NRF_CFG_IRQ_MODE
uint8_t nrf_write(uint8_t *data, int8_t len);
uint8_t nrf_int();
#else
void nrf_write(uint8_t *data, int8_t len);
void nrf_send_stop();
#endif

void nrf_recv_start();
#define nrf_recv_stop()	SPI_CE_LOW();

uint8_t nrf_read(uint8_t *data);

#ifdef NRF_CFG_EEPROM_INIT
void nrf_init(uint8_t *buf);
void nrf_tx_addr(uint8_t *buf);
void nrf_rx_addr(uint8_t *buf, uint8_t pipe);
#else
void nrf_init();
void nrf_init_pipe(uint8_t num);
#endif

#ifdef NRF_CFG_ADDR_FAST
void nrf_tx_addr(uint8_t *mac, uint8_t len)
void nrf_rx_addr(uint8_t *mac, uint8_t len, uint8_t pipe)
#endif

void nrf_power(uint8_t flag);

#endif
