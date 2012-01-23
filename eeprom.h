/*
 * nRF24L01+  EEPROM configuration.
 *
 * Sets up data pipe 0 for Enhanced ShockBurst operation.
 */

#include <avr/eeprom.h>
#include "nrf.h"

#ifndef __EEPROM_H__
#define __EEPROM_H__

/* 
 * RF channel
 *
 * 0-126
 */
#define NRF_CHANNEL	120

/* 
 * Data rate
 *
 * NRF_R_RF_SETUP_DR_250K
 * NRF_R_RF_SETUP_DR_1M
 * NRF_R_RF_SETUP_DR_2M
 */
#define NRF_RATE	(NRF_R_RF_SETUP_DR_250K)

/* 
 * TX power
 *
 * 0-3
 */
#define NRF_TXPWR	3

/* 
 * Maximum ACK wait time, see product specs
 *
 * 0x0 - 0xf
 */
#define NRF_ARD		0x3

/*
 * Maximum number of retransmits if no ACK received
 *
 * 0x0 - 0xf
 */
#define NRF_ARC		0xf

/*
 * CRC mode
 *
 * single byte crc: NRF_R_CONFIG_EN_CRC
 * two byte crc: NRF_R_CONFIG_EN_CRC | NRF_R_CONFIG_CRCO
 */
#define NRF_CRC		(NRF_R_CONFIG_EN_CRC | NRF_R_CONFIG_CRCO)

/*
 * Features:
 *
 * dynamic payload length: NRF_R_FEATURE_EN_DPL
 * dynamic auto ack: NRF_R_FEATURE_EN_DYN_ACK
 */
#define NRF_FEATURES	(NRF_R_FEATURE_EN_DPL | NRF_R_FEATURE_EN_DYN_ACK)

/* One address in EEPROM (used for rx and tx)
 *
 * length is NRF_CFG_ADDR_WIDTH
 */
uint8_t nrf_addr[] EEMEM = {
	0xde, 0xad, 0xbe, 0xef, 0x00
};


/************************************************************************/


/* 
 * Configuration structure in EEPROM
 *
 * nRF-register, value
 * nRF-register, value
 * ...
 */
uint8_t nrf_dev[] EEMEM = {
	/* Set address width (3 - 5) */
	NRF_R_SETUP_AW, NRF_R_SETUP_AW_AW(NRF_CFG_ADDR_WIDTH),
	
	/* Set RF-channel */
	NRF_R_RF_CH, NRF_CHANNEL,
	
	/* Set rate and tx-power */
	NRF_R_RF_SETUP,
		  NRF_RATE
		| NRF_R_RF_SETUP_RF_PWR(NRF_TXPWR),

	/* Set ACK timeout and max retries */
	NRF_R_SETUP_RETR,
		  NRF_R_SETUP_RETR_ARD(NRF_ARD)
		| NRF_R_SETUP_RETR_ARC(NRF_ARC),
	
	/* Set CRC-mode
	 * And mask all interrupts if not in use
	 */
	NRF_R_CONFIG, NRF_CRC
#ifndef NRF_CFG_IRQ_MODE
		| NRF_R_CONFIG_MASK_TX_DS
		| NRF_R_CONFIG_MASK_RX_DR
		| NRF_R_CONFIG_MASK_MAX_RT
#endif		
		,

	/* Set features */
	NRF_R_FEATURE, NRF_FEATURES,
	
	/* Zero all pipe configurations */
	NRF_R_EN_RXADDR, NRF_R_EN_RXADDR_ERX_NONE,
	NRF_R_EN_AA, NRF_R_EN_AA_ENAA_NONE,
	NRF_R_DYNPD, NRF_R_DYNPD_DPL_NONE,

	/* Reset status reg */
	NRF_R_STATUS, NRF_R_STATUS_MAX_RT,

	/* 
	 * Pipe configuration
	 */
	/* Receive packet length */
	NRF_R_RX_PW_P(0), 32,
	
	/* Enable pipe(0) */
	NRF_R_EN_RXADDR, NRF_R_EN_RXADDR_ERX_P(0),
	
	/* Enable Auto-ACK for pipe(0) */
	NRF_R_EN_AA, NRF_R_EN_AA_ENAA_P(0),
	
	/* Enable dynamic payload lengths */
	NRF_R_DYNPD, NRF_R_DYNPD_DPL_P(0),
	
	/* End of configuration */
	0xff
};

#endif
