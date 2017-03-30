/*30-03-17*****************************************************************************************************
***************************************************************************************************************
**                                                                                                           **
**                                                  MFrec.h                                                  **
**                                                                                                           **
***************************************************************************************************************
**************************************************************************************************************/

/* Inherited class to add key recovery to the MFRC522 class */

#ifndef _MFREC_H_
#define _MFREC_H_

/*#############################################################################################################

Section:                                          ~libs

#############################################################################################################*/

#include "crapto1.h"
#include "rfid_com_constants.h"
#include "RC522.h"

#include <cstdint>
#include <vector>
#include <pthread.h>
#include <algorithm>// sort
#include <time.h>// timing parts of code

#if RC522_WIRE
#include <wiringPi.h>
#include <wiringPiSPI.h>
#endif


/*#############################################################################################################

Section:                                          ~literals and configs

#############################################################################################################*/

/* change these to specify recovering parameters */
#define DIST_NR		15// how many distances taken to find median
#define TOLERANCE	20// accuracy of distance
#define PROBE_NR	150// how many times we try to find key
#define SETS_NR		5// how many times we try recovering each probe

typedef uint8_t byte;

/*#############################################################################################################

Section:                                          ~structs

#############################################################################################################*/

// to store authentication specifics between functions
struct Authentication
{
    byte authCommand[4];
    byte encAuthCommand[4];
    byte authParityBits[4];

};

// store known info between functions 
struct AuthInfo
{
    uint32_t distances[DIST_NR];
    uint32_t medianDist;
    uint64_t key;
};

/*#############################################################################################################

Section:                                          ~class definiton

#############################################################################################################*/

class MFrec: public RC522
{
public:

    MFrec();
    ~MFrec();

    /*-------------------------------------- key recovery  ---------------------------------------*/

    void crackKey( byte command, byte blockAddr_e, byte blockAddr_a, byte *key = nullptr );// helper function

protected:

private:

    /*-------------------------------------- picc IO  ---------------------------------------*/
    bool authenticateManually( byte command, byte blockAddr, uint32_t *n_T, byte *key = nullptr );// with known key
    void parityOff();
    void parityOn();
    void resetPICC( int waitTime );

    /*-------------------------------------- frame manipulators  ---------------------------------------*/
    byte makeRawFrame( byte *data, byte dataLen, byte *parityBits, byte *packet ); 
    void extractData( byte *packet, byte len, byte *parityBits, byte *data );
    

    /*-------------------------------------- crypto  ---------------------------------------*/
    uint32_t nonceDistance( uint32_t *n_T );// done once after manual authentication with known key
    uint32_t medianNonceDist;
    byte oddparity( const byte bt );
    uint32_t median( uint32_t *a , const int len );
    byte isNonce( uint32_t nt, uint32_t encNt, uint32_t ks1, byte *parity );

    struct Crypto1State *keystream;
    struct Authentication *m_auth;
    struct AuthInfo *m_authInfo;

    

    /*-------------------------------------- threading specifics  ---------------------------------------*/
    pthread_t t_id[ SETS_NR ];
    int id_index = 0;
    std::vector<std::vector<uint64_t>>possibleKeys;// each thread will have its own vector
    
    byte t_cmd, t_blockAddr;// bodge way to pass arguments to threaded function
    byte knownBlock;

    
    pthread_mutex_t m_lock;// ensure only 1 thread access piccIO and shared member variables at a time
    void t_recoverKey( );// use thread as non-static member function

    static void* t_recoverKey_wrapper(void* arg)// with the help of a wrapper that takes this-pointer as arg
    {
	reinterpret_cast<MFrec*>(arg)->t_recoverKey();
	return nullptr;
    }


	
};

#endif// header guard
