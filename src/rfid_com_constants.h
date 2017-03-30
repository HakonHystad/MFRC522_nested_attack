/*27-01-17*****************************************************************************************************
***************************************************************************************************************
**                                                                                                           **
**                                           rfid_com_constants.h                                            **
**                                                                                                           **
***************************************************************************************************************
**************************************************************************************************************/

/* constants used when communicating to MFRC522 reader and mifare tags */

#ifndef _RFID_COM_CONST_
#define _RFID_COM_CONST_

/*#############################################################################################################

Section:                                          ~PCD commands

#############################################################################################################*/

/* datasheet section 10.3*/ 
#define IDLE		0x00	// no action, cancels current command exectution
#define MEM		0x01	// store 25 bytes into internal buffer
#define GEN_RAND_ID	0x02	// generates 10-byte random id number
#define CALC_CRC	0x03	// activate on chip CRC calculation or perform self test
#define TRANSMIT	0x04	// transmit data from FIFO buffer
#define NO_CMD_CHANGE	0x07	// modify commandReg without affecting command
#define RECEIVE		0x08	// activate receive cicuit
#define TRANSCEIVE	0x0C	// transmit to antenna, then activate receiver after transmission
#define MF_AUTHENT	0x0E	// perform mifare authentication on chip
#define SOFT_RESET	0x0F	// reset chip

/*#############################################################################################################

Section:                                          ~PCD registers

#############################################################################################################*/

/* PCD SPI does not use the 1st bit for addresses (section 8.1.2.3) so they are shifted */

//TODO: fill out as we use them
#define CMD_REG		0x01<<1	// starts and stops command execution
#define COM_IRQ_REG	0x04<<1	// interrupt register
#define ERROR_REG	0x06<<1	// error status of the last command
#define STATUS2_REG	0x08<<1
#define FIFO_DATA_REG	0x09<<1	// input and output to 64 byte FIFO buffer
#define FIFO_LVL_REG	0x0A<<1	// nr of bytes stored in FIFO buffer
#define BIT_FRAMING_REG	0x0D<<1	// adjustments for bit-oriented frames, also start send when transceiving
#define COLL_REG	0x0E<<1	// defines first bit-collision detected
#define MODE_REG	0x11<<1	// transmit and receive setting
#define TX_CONTROL_REG	0x14<<1	// controls the antenna pins
#define TX_ASK_REG	0x15<<1	// modulation settings
#define TX_SEL_REG	0x16<<1	// handling of data from FIFO
#define MF_RX_REG	0x1D<<1	// parity handeling on/off
#define TMODE_REG	0x2A<<1	// setting of internal timer together with TPRESCALE_REG
#define TPRESCALE_REG	0x2B<<1	// see above
#define TRELOAD_REG_H	0x2C<<1	// timer reload value high byte
#define TRELOAD_REG_L	0x2D<<1	// timer reload value low byte


/*#############################################################################################################

Section:                                          ~PICC commands

#############################################################################################################*/

/*-------------------------------------- TO TAG  ---------------------------------------*/
/* initiating contact */
#define REQ_A		0x26	// activate TAG
#define ANTICOLLISION	0x9320	// request uid
#define SELECT		0x9370	// select TAG

/* authenticating */
#define AUTHENT_A	0x60	// authenticate with key A
#define AUTHENT_B	0x61	// authenticate with key B

/* read */
#define MF_READ		0x30	// reads 16 bytes (1 block)

/* stop */

#define HALT		0x50	// inactivate TAG

/*-------------------------------------- FROM TAG  ---------------------------------------*/
#define ANS_REQ		0x0400	// answer reqA
#define SAK		0x08	// select ack

#endif
