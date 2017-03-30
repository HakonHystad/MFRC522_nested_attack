/*30-03-17*****************************************************************************************************
***************************************************************************************************************
**                                                                                                           **
**                                                 RC522.cpp                                                 **
**                                                                                                           **
***************************************************************************************************************
**************************************************************************************************************/

#include "RC522.h"


/*#############################################################################################################

Section:                                          ~macros

#############################################################################################################*/

#define LSB( n )	( n & 0xFF )// extract least significant byte
#define MSB( n )	( ( n>>8 ) & 0xFF )// extract most significant byte (from 16 bit nr)


/*#############################################################################################################

Section:                                          ~init PCD

#############################################################################################################*/

/*=============================================================================================================

NAME:                                             ~RC522 (constructor)
ARGUMENT(S):
DESCRIPTION: initiates the reader
RETURN:

==============================================================================================================*/

RC522::RC522()
    : UID(0)
{
#if DBG
    std::cout << "Constructor\n";
#endif

/* initialize to default keys */
    for( int sector = 0; sector<16; sector++ )
    {
	for( int bytePos = 0; bytePos<6; bytePos++ )
	{
	    p_keys[sector][bytePos] = 0xFF;// fill with default key
	}
    }

    
#if RC522_WIRE
    
    wiringPiSetup();

    wiringPiSPISetup( CHANNEL, SPI_SPEED );

#endif

    resetPCD();

    setupPCD();

    antennaOn();

}

/*=============================================================================================================

NAME:                                             ~resetPCD
ARGUMENT(S):
DESCRIPTION: soft or hard reset of the PCD
RETURN:

==============================================================================================================*/

void RC522::resetPCD()
{
#if RC522_WIRE
    
    pinMode( RST, INPUT );

    if (digitalRead(RST) == LOW)
    {	//The MFRC522 chip is in power down mode.
	#if RC522_DBG
	std::cout << "hard reset\n";
	#endif
	
	pinMode( RST, OUTPUT );
	digitalWrite(RST, HIGH);		// Exit power down mode. This triggers a hard reset.

// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
	delay(50);
    }
    else
    { // Perform a soft reset

	#if RC522_DBG
	std::cout << "soft reset\n";
	#endif

	m_val = SOFT_RESET;
	writeRegister( CMD_REG, &m_val );
	//writeRegister( CMD_REG, (byte*)SOFT_RESET );
	delay(50);

	byte cmdStatus;
	
	// wait until power down bit is cleared
	do
	{
	    readRegister( CMD_REG, &cmdStatus );
	}while( cmdStatus & (1<<4) );
    }// end hard/soft reset
    
#endif// wire
}// resetPCD
    

/*=============================================================================================================

NAME:                                             ~setupPCD
ARGUMENT(S):
DESCRIPTION: sets up timeout timer, modulation and CRC
RETURN:

==============================================================================================================*/

void RC522::setupPCD()
{
        
// When communicating with a PICC we need a timeout if something goes wrong.
// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
    m_val = 0x80;
    writeRegister(TMODE_REG, &m_val );			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
    m_val = 0xA9;
    writeRegister(TPRESCALE_REG, &m_val );		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
    m_val = 0x03;
    writeRegister(TRELOAD_REG_H, &m_val );		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
    m_val = 0xE8;
    writeRegister(TRELOAD_REG_L, &m_val );
    m_val = 0x40;
    writeRegister(TX_ASK_REG, &m_val );			// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
    // transmit only in RF field, polarity active HIGH on MFIN pin, CRC preset 0x6363
    m_val = 0x3D;
    writeRegister(MODE_REG, &m_val );			// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
    
}

/*#############################################################################################################

Section:                                          ~init PICC

#############################################################################################################*/

/*=============================================================================================================

NAME:                                             ~sendReqA
ARGUMENT(S):
DESCRIPTION: sends the req_A command to initiate contact with PICC
RETURN: true if TAG answers

==============================================================================================================*/

bool RC522::sendReqA()
{
#if RC522_DBG
    std::cout << "sendReqA\n";
#endif

    byte atqa[3] = {REQ_A};// send and receive buffer, send req A, store answer reguest A

    if( !piccIO( TRANSCEIVE, 1, atqa, 3, 7 ) ) return false;

    return ( atqa[0] == MSB(ANS_REQ) && !atqa[1] );// true if TAG answers 04 00
}

/*=============================================================================================================

NAME:                                             ~anticollision
ARGUMENT(S):
DESCRIPTION: transmits anticollision and receives uid
RETURN:

==============================================================================================================*/    

bool RC522::anticollision()
{
#if RC522_DBG
    std::cout << "anticollision\n";
#endif
    
    byte pack[5];// to receive 4 byte uid + bcc
    pack[0] = MSB(ANTICOLLISION);
    pack[1]  = LSB(ANTICOLLISION);

    if( !piccIO( TRANSCEIVE, 2, pack, 5 ) ) return false;

    for( int i = 0; i<4; i++) p_uid[i] = pack[i];

    //TODO: implement this in inherited class UID = bytesToInt( p_uid, 4 );

    return ( p_uid[0] != MSB( ANTICOLLISION ) );
}

/*=============================================================================================================

NAME:                                             ~select
ARGUMENT(S):
DESCRIPTION: selects TAG with uid to set up communication
RETURN:

==============================================================================================================*/    

bool RC522::select()
{
#if RC522_DBG
    std::cout << "select\n";
#endif
    
    byte pack[9];// select(1), nvb(1), uid(4), bcc(1), crc(2)
    pack[0] = MSB(SELECT);
    pack[1] = LSB(SELECT);

    for( int i=0; i<4; i++ )
    {
	pack[2+i] = p_uid[i];
    }

    //BCC
    pack[6] = pack[2] ^ pack[3] ^ pack[4] ^ pack[5];

    calcCRC( pack, 7, &pack[7] );// append CRC

    if( !piccIO( TRANSCEIVE, 9, pack, 9 ) ) return false;

    return ( pack[0] == SAK );// select ack
}

/*=============================================================================================================

NAME:                                             ~initCom
ARGUMENT(S):
DESCRIPTION: waits for and sets up communication with TAG
RETURN:

==============================================================================================================*/

void RC522::initCom()
{
   
    while( 1 )
    {
	#if RC522_WIRE
	delay(10);
	#endif
	
	if( !sendReqA() ) continue;

	if( !anticollision() ) continue;

	if( !select() ) continue;

	break;
    }
#if RC522_DBG
    std::cout << "Initiated\n";
#endif
}

/*#############################################################################################################

Section:                                          ~authenticate

#############################################################################################################*/

/*=============================================================================================================

NAME:                                             ~authenticateOnChip
ARGUMENT(S): authenticate A or B, block in sector to authenticate, optional 6 byte key for encryption
DESCRIPTION: authenticates and uses encrypted communtication from then on, done on chip
RETURN:

==============================================================================================================*/

bool RC522::authenticateOnChip( byte command, byte blockAddr, byte *key /*=nullptr*/ )
{
#if RC522_DBG
    std::cout << "authenticate\n";
#endif
    
    byte pack[12];// command(1), block(1), key(6), uid(4)

    pack[0] = command;
    pack[1] = blockAddr;

    if( key == nullptr )// use previosly stored key (or default)
    {
	for( int i = 0; i<6; i++ )
	{
	    pack[i+2] = p_keys[blockAddr/4][i];
	}
    }
    else
    {
	for( int i = 0; i<6; i++ )
	{
	    pack[i+2] = key[i];
	}
    }// else key==nullptr

    for( int i = 0; i<4; i++ )
    {
	pack[i+8] = p_uid[i];
    }

    if( piccIO( MF_AUTHENT, 12, pack, 12 ) )
    {
	currentlyAuthenticated = blockAddr/4;

	if( key != nullptr )
	{
	    for( int i = 0; i<4; i++ )// store the validated key
	    {
		p_keys[currentlyAuthenticated][i] = key[i];
	    }
	}
	return true;
    }

    std::cerr << "Could not authenticate\n";
    return false;

}// authenticateOnChip


/*#############################################################################################################

Section:                                          ~use

#############################################################################################################*/


/*=============================================================================================================

NAME:                                             ~readBlock
ARGUMENT(S): addr(0-63), buffer to store data (min 18 bytes), length of buffer
DESCRIPTION: reads a block from the TAG, note: this will authenticate also if not done prior
RETURN:

==============================================================================================================*/

bool RC522::readBlock( byte blockAddr, byte *data, byte len )
{
#if RC522_DBG
    std::cout << "readBlock\n";
#endif
    
    if( len < 18 )
    {
	std::cerr << "Could not read: No storage\n";
	return false;
    }

    if( blockAddr > 63 )
    {
	std::cerr << "Could not read: Invalid address\n";
	return false;
    }

    if( currentlyAuthenticated != blockAddr/4 )// if block is not in currently open sector
    {
	if( !authenticateOnChip( AUTHENT_A, blockAddr) )// authenticate it
	{
	    return false;
	}
    }

    data[0] = MF_READ;
    data[1] = blockAddr;

    calcCRC( data, 2, &data[2] );

    return piccIO( TRANSCEIVE, 4, data, len );
}


/*=============================================================================================================

NAME:                                             ~getUID
ARGUMENT(S):
DESCRIPTION: getter
RETURN:

==============================================================================================================*/

uint32_t RC522::getUID()
{
    return UID;
}


/*=============================================================================================================

NAME:                                             ~antennaOn
ARGUMENT(S):
DESCRIPTION: turns on the antenna of MFRC522
RETURN:

==============================================================================================================*/

void RC522::antennaOn()
{
    byte antennaStatus;
    readRegister( TX_CONTROL_REG, &antennaStatus);

    if( !(antennaStatus & 0x03) )//set pin Tx1 and Tx2 to output signal
    {
	antennaStatus |= 0x03;
	writeRegister( TX_CONTROL_REG, &antennaStatus );
    }
}

/*=============================================================================================================

NAME:                                             ~antennaOff
ARGUMENT(S):
DESCRIPTION: turns off the antenna of MFRC522
RETURN:

==============================================================================================================*/

void RC522::antennaOff()
{
    readRegister( TX_CONTROL_REG, &m_val );
    if( m_val & 0x03 )
    {
	m_val &= ~0x03;
	writeRegister( TX_CONTROL_REG, &m_val );
    }
}


/*#############################################################################################################

Section:                                          ~stop

#############################################################################################################*/

/*=============================================================================================================

NAME:                                             ~halt
ARGUMENT(S):
DESCRIPTION: set TAG from active to halt
RETURN:

==============================================================================================================*/

bool RC522::halt()
{
#if RC522_DBG
    std::cout << "halt\n";
#endif
    
    byte pack[4];// command(1), 0x00, CRC(2)

    pack[0] = HALT;
    pack[1] = 0;

    calcCRC( pack, 2, &pack[2] );

    if( !piccIO( TRANSCEIVE, 4, pack, 4 ) )// timeout is a good thing here
    {
	return true;
    }

    std::cerr << "Could not halt\n";
    return false;
}

/*=============================================================================================================

NAME:                                             ~stopCrypto
ARGUMENT(S):
DESCRIPTION: stops the encryption on chip, call halt before using
RETURN:

==============================================================================================================*/

void RC522::stopCrypto()
{
#if RC522_DBG
    std::cout << "stopCrypto\n";
#endif
    byte tmp;
    readRegister( STATUS2_REG, &tmp );

    tmp &= ~0x08;
    writeRegister( STATUS2_REG, &tmp );// reset cryptoOn bit
}

/*=============================================================================================================

NAME:                                             ~stop
ARGUMENT(S):
DESCRIPTION: helper to halt, then stop crypto
RETURN:

==============================================================================================================*/
    
void RC522::stop()
{
    while( !halt() );

    stopCrypto();
}


/*#############################################################################################################

Section:                                          ~PICC IO

#############################################################################################################*/

/*=============================================================================================================

NAME:                                             ~writeRegister
ARGUMENT(S): register, value to write, optional length of value (>1 byte to set)
DESCRIPTION: sets a value in the specified register
RETURN:

==============================================================================================================*/

void RC522::writeRegister( byte reg, byte *val, const byte len /*=1*/ )
{
    
    byte data[ len + 1];// addr + val
    // select register
    data[0] = ( reg & 0x7E ); // section 8.1.2.3: MSB=0 -> writing register, LSB=0

    for( int i = 0; i<len; i++ )
    {
	data[i+1] = val[i];
    }
    
#if RC522_DBG
    std::cout << "Wrote register " << std::hex << (int)( reg>>1 )
	      << ": ";
    for( int i = 0; i<len+1; i++ )
    {
	std::cout << (int)data[i] << " ";
    }
    std::cout << std::dec << std::endl;
#endif
    
#if RC522_WIRE    
    wiringPiSPIDataRW(CHANNEL, data, len+1 );
#endif
    
}

/*=============================================================================================================

NAME:                                             ~readRegister
ARGUMENT(S): register to read, buffer to store, optional nr of bytes to read
DESCRIPTION: reads a register
RETURN:

==============================================================================================================*/

void RC522::readRegister( byte reg, byte *buffer, byte nrOfBytes /*= 1*/ )
{ 
    // select register
    buffer[0] = (0x80 | (reg & 0x7E));// section 8.1.2.3: MSB=1 -> reading, LSB=0

#if RC522_WIRE
    wiringPiSPIDataRW(CHANNEL, buffer, 1 );// 1st byte back is trash
#endif
    
    for( int i = 0; i<nrOfBytes-1; i++ )// if more than 1 byte is to be read
    {
	buffer[i] = (0x80 | (reg & 0x7E));// section 8.1.2.3: MSB=1 -> reading, LSB=0
    }

    buffer[nrOfBytes-1] = 0;// stop reading

#if RC522_WIRE
    wiringPiSPIDataRW(CHANNEL, buffer, nrOfBytes);
#endif

#if RC522_DBG
    std::cout << "Read register " << std::hex << (int)(reg>>1)
 << ": ";
    for( int i = 0; i<nrOfBytes; i++ )
    {
	std::cout << (int)buffer[i] << " ";
    }
    std::cout << std::dec << std::endl;
#endif
    
}

/*=============================================================================================================

NAME:                                             ~piccIO
ARGUMENT(S): command to pcd, nr of bytes to send, buffer with data to send but also receive to, length of buffer, optional nr of bits sent in the last byte (man parity will need this) 
DESCRIPTION: sends a command and belonging data, receives to the same buffer overwriting the old info
RETURN:

==============================================================================================================*/

bool RC522::piccIO( byte command, byte nrOfBytesToSend, byte *data, byte len, byte nrOfLastBits /*=0*/ )
{
    
#if RC522_DBG
    std::cout << "piccIO\n";
#endif

    m_val = IDLE;
    writeRegister( CMD_REG, &m_val );// stop any ongoing command
    m_val = 0x7F;
    writeRegister( COM_IRQ_REG, &m_val );// clear interrupt register
    m_val = 0x80;
    writeRegister( FIFO_LVL_REG, &m_val );// flush FIFO buffer
    writeRegister( FIFO_DATA_REG, data, nrOfBytesToSend );// send data to FIFO
    m_val = nrOfLastBits;// set the nr of bits to send of the last byte ( REQ_A uses 7, man parity varies 
    writeRegister( BIT_FRAMING_REG, &m_val );
    
    writeRegister( CMD_REG, &command );// ex. tranceive
    

    byte finishFlag = 0x10;// default command finished (idle)

    if( command == RECEIVE )
    {
	finishFlag = 0x30;// finished command (idle) or received the end of a datapacket
    }
    
    if( command == TRANSCEIVE )// set bit 7 of bitframing_reg to start send when transceiving
    {
	finishFlag = 0x30;// finished command (idle) or received the end of a datapacket
	byte tmp;
	#if RC522_WIRE
	delay(1);// give time for the change of bitframing reg
	#endif
	readRegister( BIT_FRAMING_REG, &tmp );
	m_val = tmp | 0x80;
	writeRegister( BIT_FRAMING_REG, &m_val );
    }

    int watchDog = 3000;
    byte status;
    while(1)// wait for completion
    {
	readRegister( COM_IRQ_REG, &status );

	if( status & finishFlag ) break;

	if( status & 0x01 || !(--watchDog) )// chip timer (set in constructor) or safety
	{
	    std::cerr << "Timeout\n";
	    return false;
	}
	#if RC522_WIRE
	#if RC522_DBG
	delay(1);
	#endif
	#endif
    }// while

    // check for errors of the last command
    readRegister( ERROR_REG, &status );
    if( status & 0x11 )// FIFO overflow or protocol error
    {
	std::cerr << "Error occured\n";
	return false;
    }
    
    /*-------------------------------------- receiving  ---------------------------------------*/
    if( command == TRANSCEIVE || command == RECEIVE )
    {
	readRegister( FIFO_LVL_REG, &status );// bytes in current FIFO

	if( status > len )
	{
	    std::cerr << "Not enough room in buffer\n";
	    return false;
	}

	readRegister( FIFO_DATA_REG, data, status );
	
    }// if receive
    
    return true;

}// piccIO



/*=============================================================================================================

NAME:                                             ~calcCRC
ARGUMENT(S): packet and size of packet, CRC buffer (2 bytes)
DESCRIPTION: calculates the crc off the chip
RETURN:

==============================================================================================================*/

void RC522::calcCRC( byte *data, byte len, byte *CRC )//from libnfc/iso14443-subr.c
{
    uint8_t  bt;
    uint32_t wCrc = 0x6363;

    do {
	bt = *data++;
	bt = (bt ^ (uint8_t)(wCrc & 0x00FF));
	bt = (bt ^ (bt << 4));
	wCrc = (wCrc >> 8) ^ ((uint32_t) bt << 8) ^ ((uint32_t) bt << 3) ^ ((uint32_t) bt >> 4);
    } while (--len);

    *CRC++ = (uint8_t)(wCrc & 0xFF);
    *CRC = (uint8_t)((wCrc >> 8) & 0xFF);
}
