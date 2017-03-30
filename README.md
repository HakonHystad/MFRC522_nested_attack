# MFRC522_nested_attack
mifare nested authentication attack with the MFRC522 reader

PREFACE:

Mifare classic cards are known to have several vulnerabilities and should not be trusted with any sensitive information. Even so I've seen an estimate of 1B cards that is/has been in use and it just so happens that my NTNU student id is one of them. Naturally I got curious about what was on it and since some sectors where using a default key I went with the nested authentication attack to find out. To do this I wanted to use a raspberry pi and the RC522 RFID reader from an arduino kit.  

ACKNOWLEDGEMENTS:

The communication between the raspberry and the reader is a minimum working class for the mifare classic only and is based on the arduino library: https://github.com/miguelbalboa/rfid. 

The key recovery inherits the reader class and uses the crypto1/crapto1 library (http://crapto1.netgarage.org/) largely based on it's use for USB readers seen here https://github.com/nfc-tools/mfoc.

THE ATTACK:

The mifare classic 1k has a weak random number generator (RNG) which is basically a shift register with a little extra. Its weakness comes from the ability to roll back the 32bit generated nonce (challange). If we authenticate a number of times with a known key we can find the median number of shifts and pretty accurately guess the unencrypted nonce by rolling back the encrypted nonce (we allready authenticated and we know the encryption). Since this nonce is initiallized by the symmetric key XOR'ed with the cards UID we can effectively find 32 bits of the 48 bit key. 

The mifare encryption does not abide the standards and encrypts the parity bits as well, even worse these are encytped (XOR'ed) with the same keystream as the nonce. This drastically reduces the number of valid nonces. Doing the 32 bit recovery several times (generating ~200k possible keys each time) we can look for duplicates and try the most promissing ones. 

MY CONTRIBUTIONS:

All code is written by me except for the crapto1/crypto1 library, although based on other sources. My most original contributions are the use of raw frames on the RC522 reader and threading the key recovery to cut the processing time in half. 

I could find very little mention of using the RC522 solely as a transceiver (sending and receiving raw frames), but a note in its datasheet mentioning turning off parity hints at the possibility. Turns out that it is possible sending little endian frames with parity after every 8th bit and a CRC16 checksum appended at the end to the FIFO buffer. The trick is to turn of every register bit that does something for you automatically as well as handeling arbitrary bitlength. This was a lot trickier to figure out than it sounds.

The key recovery as implemented by me used (on a 150 sample average) 114 seconds to go through one iteration (with 5 recoveries) on the raspberry pi 3 (1.2GHz). By threading the recovery process where possible (locking the parts accessing hardware with a mutex) I managed to get it down to 25 seconds: nearly a 71% reduction.

THE INTERFACE:

As a school project I also made a minimalist webpage to display the results, updating the sectors as more keys gets cracked. This is done using a self written client socket which uses HTTP POST requests to send the read data. Server side I made request handler in php to store the data in an SQL database and a polling script to continously updating with new data using AJAX. 

RESULTS:

A regular mifare classic 1k card has a sector key cracked within the first iteration (i.e 25s on average with 5 recoveries) as long as one of its sectors uses the default (or other know) key. The results are displayed in "real time" on my self made webpage when reading a card. As for my student ID; it does not seem to utilize the vulnarable RNG and is most likely an emulated mifare classic 1k, this attack has no effect on it. 
