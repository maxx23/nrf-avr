/*
 * nRF24L01+ library configuration.
 *
 * Set compile time parameters.
 */

#ifndef __NRFCFG_H__
#define __NRFCFG_H__

/* Error indicator (LED, piezo, ...)
 */
#define NRF_CFG_ERROR_IND() \
		PORTB |= (_BV(PB4))

/* Reads configuraion data from EEPROM
 *
 * This not only saves a lot of space, it also enables
 * you to change RF-configuration by reflashing eeprom
 * only! Maybe via wireless itself?
 */
#define NRF_CFG_EEPROM_INIT 1

#ifdef NRF_CFG_EEPROM_INIT
/* Global address-width
 *
 * Match your configuration data with this!
 */
#define NRF_CFG_ADDR_WIDTH 5
#endif

/* Change addresses from buffer in RAM
 *
 * Changes nrf_tx_addr() and nrf_rx_addr(), see nrf.h
 */
// #define NRF_CFG_ADDR_FAST 1

/* Enable dynamic no-acks
 *
 * Enables you to selectively use the auto-ack feature
 * by passing negated length to nrf_write()
 */
#define NRF_CFG_DYN_ACK 1

/* Use IRQs instead of polling.
 *
 * You can put the MCU to sleep while waiting for transmission
 * (rx/tx).
 */
//#define NRF_CFG_IRQ_MODE 1

#ifdef NRF_CFG_IRQ_MODE
/* IRQ vector */
#if defined(__AVR_ATtiny2313__)
#define NRF_CFG_INT_VEC PCINT_vect
#elif defined(__AVR_ATmega8__)
#define NRF_CFG_INT_VEC INT0_vect
#endif

/* Power down after tx-error right in nrf_irq(). */
#define NRF_CFG_IRQ_TE_PWR_DOWN 1

/* Power down after transmission in nrf_irq() */
#define NRF_CFG_IRQ_TX_PWR_DOWN 1

/* Power down right after packet received. */
#define NRF_CFG_IRQ_RX_PWR_DOWN 1

/* Flush buffers when tx-error occurs */
// #define NRF_CFG_IRQ_TE_FLUSH 1
#endif


/* SPI stuff */
#if defined(__AVR_ATtiny2313__)

#define SPI_PORT	PORTB
#define SPI_DDR		DDRB
#define SPI_PIN		PINB
#define SPI_IRQ		0
#define SPI_CS		1
#define SPI_CE		3
#define SPI_DO		5
#define SPI_DI		6
#define SPI_SCK		7
#define SPI_SPEED	1

#elif defined(__AVR_ATmega8__)

#define SPI_PORT	PORTB
#define SPI_DDR		DDRB
#define SPI_PIN		PINB
#define SPI_CS		2
#define SPI_DO		3
#define SPI_DI		4
#define SPI_SCK		5

#define SPI_AUX_DDR		DDRD
#define SPI_AUX_PORT	PORTD
#define SPI_AUX_PIN		PIND
#define SPI_AUX_CE		5
#define SPI_AUX_IRQ		6

#endif


#endif
