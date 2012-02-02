/* internal definitions */
#include <stdint.h>

#ifndef __NRF24L01_H__
#define __NRF24L01_H__

/* Commands */
#define NRF_C_R_REGISTER		0x00
#define NRF_C_W_REGISTER		0x20
#define NRF_C_R_RX_PAYLOAD		0x61
#define NRF_C_W_TX_PAYLOAD		0xA0
#define NRF_C_W_TX_PAYLOAD_NOACK	0xB0
#define NRF_C_FLUSH_TX			0xE1
#define NRF_C_FLUSH_RX			0xE2
#define NRF_C_REUSE_TX_PL		0xE3
#define NRF_C_R_RX_PL_WID		0x60
#define NRF_C_W_ACK_PAYLOAD		0xA8
#define NRF_C_NOP			0xFF

/* Registers */
#define NRF_R_CONFIG			0x00
#define NRF_R_EN_AA			0x01	
#define NRF_R_EN_RXADDR			0x02
#define NRF_R_SETUP_AW			0x03
#define NRF_R_SETUP_RETR		0x04
#define NRF_R_RF_CH			0x05
#define NRF_R_RF_SETUP			0x06
#define NRF_R_STATUS			0x07
#define NRF_R_OBSERVE_TX		0x08
#define NRF_R_RPD			0x09
#define NRF_R_FEATURE			0x1D
#define NRF_R_RX_ADDR_P(x)		(0x0A + (x))
#define NRF_R_RX_ADDR_P_LEN(x, y)	((x < 2)?y:1)
#define NRF_R_TX_ADDR			0x10
#define NRF_R_TX_ADDR_LEN		5
#define NRF_R_RX_PW_P(x)		(0x11 + (x))
#define NRF_R_FIFO_STATUS		0x17
#define NRF_R_DYNPD			0x1c

/* NRF_R_CONFIG */
#define NRF_R_CONFIG_RESERVED		0x80
#define NRF_R_CONFIG_MASK_RX_DR		0x40
#define NRF_R_CONFIG_MASK_TX_DS		0x20
#define NRF_R_CONFIG_MASK_MAX_RT	0x10
#define NRF_R_CONFIG_EN_CRC		0x08
#define NRF_R_CONFIG_CRCO		0x04
#define NRF_R_CONFIG_PWR_UP		0x02
#define NRF_R_CONFIG_PRIM_RX		0x01

/* NRF_R_EN_AA */
#define NRF_R_EN_AA_ENAA_P(x)		(1 << (x))
#define NRF_R_EN_AA_ENAA_NONE		0x00

/* NRF_R_EN_RXADDR */
#define NRF_R_EN_RXADDR_ERX_P(x)	(1 << (x))
#define NRF_R_EN_RXADDR_ERX_NONE	0x00

/* NRF_R_RF_CH */
#define NRF_R_RF_CH_BITS		0x7f

/* NRF_R_RF_SETUP */
#define NRF_R_RF_CONT_WAVE		0x80
#define NRF_R_RF_SETUP_RF_DR_LOW	0x20
#define NRF_R_RF_SETUP_PLL_LOCK		0x10
#define NRF_R_RF_SETUP_RF_DR_HIGH	0x08
#define NRF_R_RF_SETUP_RF_PWR(x)	(((x) & 0x3) << 1)
#define NRF_R_RF_SETUP_DR_1M		0x00
#define NRF_R_RF_SETUP_DR_2M		0x08
#define NRF_R_RF_SETUP_DR_250K		0x20

/* NRF_R_SETUP_AW */
#define NRF_R_SETUP_AW_AW(x)		((x - 2) & 0x3)

/* NRF_R_STATUS */
#define NRF_R_STATUS_RX_DR		0x40
#define NRF_R_STATUS_TX_DS		0x20
#define NRF_R_STATUS_MAX_RT		0x10
#define NRF_R_STATUS_RX_P_NO		0x0E
#define NRF_R_STATUS_GET_RX_P_NO(x)	(((x) & R_STATUS_RX_P_NO) >> 1)
#define NRF_R_STATUS_RX_FIFO_EMPTY	0x0E
#define NRF_R_STATUS_TX_FULL		0x01

/* NRF_R_SETUP_RETR */
#define NRF_R_SETUP_RETR_ARD(x)		(((x) & 0x0f) << 4)
#define NRF_R_SETUP_RETR_ARC(x)		((x) & 0x0f)

/* NRF_R_FEATURE */
#define NRF_R_FEATURE_EN_DPL		0x04
#define NRF_R_FEATURE_EN_ACK_PAY	0x02
#define NRF_R_FEATURE_EN_DYN_ACK	0x01

/* NRF_R_DYNPD */
#define NRF_R_DYNPD_DPL_P(x)		(0x1 << (x))
#define NRF_R_DYNPD_DPL_NONE		0X00

/* NRF_R_FIFO_STATUS */
#define NRF_R_FIFO_STATUS_RX_EMPTY 	0x1
#define NRF_R_FIFO_STATUS_RX_FULL	0x2
#define NRF_R_FIFO_STATUS_TX_EMPTY	0x10
#define NRF_R_FIFO_STATUS_TX_FULL	0x20
#define NRF_R_FIFO_STATUS_TX_REUSE	0x40

/* NRF_R_RF_CH */
#define NRF_R_RF_CH_RF_CH(v)		((v) & 0x7f)

/* delays */
#define NRF_T_SND_DELAY			10		
#define NRF_T_PWR_UP			150		/* External Clock */

/* functions */
#define SPI_CS_HIGH()	SPI_PORT |= (1 << SPI_CS)
#define SPI_CS_LOW()	SPI_PORT &= ~(1 << SPI_CS)

#if defined(__AVR_ATtiny2313__)

#define SPI_CE_HIGH()	SPI_PORT |= (1 << SPI_CE)
#define SPI_CE_LOW()	SPI_PORT &= ~(1 << SPI_CE)

#elif defined(__AVR_ATmega8__)

#define SPI_CE_HIGH()	SPI_AUX_PORT |= (1 << SPI_AUX_CE)
#define SPI_CE_LOW()	SPI_AUX_PORT &= ~(1 << SPI_AUX_CE)

#endif

uint8_t spi_rwb(uint8_t in);
void spi_rw(uint8_t cmd, uint8_t *in, uint8_t *out, uint8_t len);

uint8_t nrf_cmd(uint8_t cmd);
uint8_t nrf_rwcmd(uint8_t cmd, uint8_t val);

#endif
