/* max hop count */
#define MAX_HOPS			3

/* max hops away from and AP. Keeps hop count and therefore replay
 * storms down for sending to and from polling End Devices. Also used
 * when joining since the EDs can't be more than 1 hop away.
 */
#define MAX_HOPS_FROM_AP	1

/* Maximum size of Network application payload. Do not change unless
 * protocol changes are reflected in different maximum network
 * application payload size.
 */
#define MAX_NWK_PAYLOAD		34

/* Maximum size of application payload */
#define MAX_APP_PAYLOAD		10

/* default Link token */
#define DEFAULT_LINK_TOKEN	0x01020304

/* default Join token */
#define DEFAULT_JOIN_TOKEN	0x05060708

/* Remove 'x' corruption to define Frequency Agility as active for this build */
#define xFREQUENCY_AGILITY

/* Remove 'x' corruption to enable application autoacknowledge support. Requires extended API as well */
#define xAPP_AUTO_ACK

/* Remove 'x' corruption to enable Extended API */
#define xEXTENDED_API

/* Remove 'x' corruption to enable security. */
#define xSMPL_SECURE

/* Remove 'x' corruption to enable NV object support. */
#define xNVOBJECT_SUPPORT

/* Remove 'x' corruption to enable software timer. */
#define xSW_TIMER

/* Remove 'x' corruption to enable frequency hopping. */
#define xFREQUENCY_HOPPING

#define BSP_TIMER_USED	BSP_TIMER_A3

/* Remove 'x' corruption to make this device the reference clock. */
#define NWK_PLL_REFERENCE_CLOCK

/* causes leds to blink in 00 -> 01 -> 11 -> 10 -> 00 rotation when FHSS enabled */
#define NWK_PLL_SHOW_LOCATION_INDICATORS

#define I_WANT_TO_CHANGE_DEFAULT_ROM_DEVICE_ADDRESS_PSEUDO_CODE
