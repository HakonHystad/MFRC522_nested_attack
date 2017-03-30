/*30-03-17*****************************************************************************************************
***************************************************************************************************************
**                                                                                                           **
**                                                  RC522.h                                                  **
**                                                                                                           **
***************************************************************************************************************
**************************************************************************************************************/

/* RFID base class to communicate via the MFRC522 reader */

/* flexibility defines:
 * DBG: set to get debug info in stdout
 * WIRE: reset to compile outside of raspberry, for ease of working only
 */
#define RC522_DBG	0
#define RC522_WIRE	0

#ifndef _RC522_H_
#define _RC522_H_

/*#############################################################################################################

Section:                                          ~libs

#############################################################################################################*/

#include "rfid_com_constants.h"
#include "crapto1.h"

#include <cstdint>
#include <iostream>


#if RC522_WIRE

#include <wiringPi.h>
#include <wiringPiSPI.h>

#endif

/*#############################################################################################################

Section:                                          ~literals and defines

#############################################################################################################*/

/* 
 * raspberry	   (wirepi) pin	      MFRC522
 * ------------------------------------------------------------
 * Reset		5		RST
 * SPI CE0	     	10		SSN (slave select)
 * SPI MOSI	  	12		MOSI
 * SPI MISO		13		MISO
 * SPI SCK		14		SCK
 * 3.3V					Vcc
 * GND					GND
 *
 */
#define RST	5
#define CHANNEL 0// CE0
#define SPI_SPEED 4000000// Hz (500k-32m)


typedef uint8_t byte;


/*#############################################################################################################

Section:                                          ~class definition

#############################################################################################################*/

class RC522
{
public:

    /*-------------------------------------- init PCD  ---------------------------------------*/

    RC522();
    void resetPCD();
    void setupPCD();

    /*-------------------------------------- init PICC  ---------------------------------------*/
    bool sendReqA();// init contact
    bool anticollision();// get uid
    bool select();// select card
    /* the above  gathered in: */
    void initCom();// helper function

    /*-------------------------------------- authenticate  ---------------------------------------*/
    bool authenticateOnChip( byte command, byte blockAddr, byte *key = nullptr );

    /*-------------------------------------- use  ---------------------------------------*/
    bool readBlock( byte blockAddr, byte *data, byte len );

    uint32_t getUID();
    void antennaOn();
    void antennaOff();

    /*-------------------------------------- stop  ---------------------------------------*/
    bool halt();// pause communication
    void stopCrypto();// disable encryption
    /* the above gathered in: */
    void stop();// helper function
    

protected:

    /*-------------------------------------- variables  ---------------------------------------*/
    byte p_keys[16][6];// TODO: make getter/setter
    byte p_uid[4] = {0};
    byte currentlyAuthenticated;// sector
    byte m_val;// bodge for passing args....

    /*-------------------------------------- PCD IO  ---------------------------------------*/
    void writeRegister( byte reg, byte *val, byte len = 1 );
    void readRegister( byte reg, byte *buffer, const byte len = 1 );

    bool piccIO( byte command, byte nrOfBytesToSend, byte *data, byte len, byte nrOfLastBits = 0 );

    void calcCRC( byte *data, byte len, byte *CRC);

    
    uint64_t bytesToInt( byte *ntBuffer, byte nrOfBytes );

    uint32_t UID;

};
    

#endif// header guard
