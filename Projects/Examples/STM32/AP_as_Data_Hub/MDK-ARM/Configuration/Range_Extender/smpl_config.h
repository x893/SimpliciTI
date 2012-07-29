/* Number of connections supported. Each connection supports bi-directional
 * communication.  Access Points and Range Extenders can set this to 0 if they
 * do not host End Device objects.
 */
#define NUM_CONNECTIONS		0

/*  ***  Size of low level queues for sent and received frames. Affects RAM usage  ***  */

/* AP needs larger input frame queue if it is supporting store-and-forward
 * clients because the forwarded messages are held here.
 */
#define SIZE_INFRAME_Q		2

/* The output frame queue can be small since Tx is done synchronously. Actually
 * 1 is probably enough. If an Access Point device is also hosting an End Device 
 * that sends to a sleeping peer the output queue should be larger -- the waiting 
 * frames in this case are held here. In that case the output frame queue should 
 * be bigger. 
 */
#define SIZE_OUTFRAME_Q		2

/* This device's address. The first byte is used as a filter on the CC1100/CC2500
 * radios so THE FIRST BYTE MUST NOT BE either 0x00 or 0xFF. Also, for these radios
 * on End Devices the first byte should be the least significant byte so the filtering
 * is maximally effective. Otherwise the frame has to be processed by the MCU before it
 * is recognized as not intended for the device. APs and REs run in promiscuous mode so
 * the filtering is not done. This macro intializes a static const array of unsigned
 * characters of length NET_ADDR_SIZE (found in nwk_types.h). The quotes (") are
 * necessary below unless the spaces are removed.
 */
#define THIS_DEVICE_ADDRESS	{0x77, 0x56, 0x34, 0x12}

/* device type */
#define RANGE_EXTENDER
