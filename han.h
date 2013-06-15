/*
* han.h
*/

#ifndef HAN
#define HAN


/*
* Protocol specific constants
*/

#define MAXPARAMS 	16				// Maximum number of parameter bytes, inclusive of 1 or 2 byte CRC
							// Ex: For 2 byte CRC's, up to 14 bytes are available for parameters
#define PKTIRQLEN	4				// Length of an IRQ packet
#define PKTCTRL		3				// Number of packet control bytes 

#define MAXPACKET	PKTCTRL + MAXPARAMS		// Max packet size excluding byte stuffing and STX/ETX

#define STX		0x02				// Denotes start of frame
#define ETX		0x03				// Denotes end of frame
#define SUBST		0x04				// Substitute next character
#define HDC		0x01				// Header control bits for cmd
#define	HDCIRQ		0x02				// Header control bits for interrupt request
#define HDCIRQ16	0x03				// Header control bits for interrupt request with 16 bit CRC
#define HDC16		0x05				// Header control bits for 16 bit CRC and 8 bit address
#define	HDC_ACK		0xC1				// ACK response
#define HDC_NAK 	0x81				// NAK response
#define	HDC_ACK16 	0xC5				// ACK response CRC16
#define	HDC_NAK16 	0x85				// NAK response CRC16	

#define	POLY		0x8C				// 8 bit CRC polynomial: X^8 + X^5 + X^4 + 1
#define POLY16		0x1021				// 16 bit CRC polynomial: X^16 + X^12 + X^5 + 1

// Common state machine constants
enum {RXI_INIT = 0, RXI_ASSEM, RXI_FINISH};
enum {TXI_INIT=0, TXI_TXC, TXI_TXC_POSTSUB, TXI_FIN};
enum {PHD_START= 0, PHD_PKT_READY, PHD_PKT_DECODE, PHD_PKT_RESP, PHD_TX_START, PHD_WAIT_TX, PHD_WAIT_EMPTY, PHD_FIN};


/*
* Commands
*/

// Required on all nodes

#define NOOP	0				// No Operation
#define	GNID	1				// Return node ID information
#define GCST	2				// Return communications status
#define	GIPL	3				// Poll for interrupt reason
#define GEBL	0x0F				// Enter boot loader

// Implementation specific commands

#define GVLV	0x10				// Set a valve to a specific state(valve,state)
#define GRLY	0x11				// Dry contact relay(channel,state) state: 0 off, FF on, 1-FE pulsed 100ms per count
#define GTMP	0x12				// Return temperature(channel,counts_per_deg_c,sensor_type,templow,temphigh)
#define GOUT	0x13				// Control digital outputs(channel,command, state) command: 0, off 1 on, 2 return state
#define GINP	0x14				// Read digital input (channel,state)
#define	GACD	0x15				// Read AC voltage and frequency (voltlow, volthigh, freqlow, freqhigh)
#define GVLT    0x16                            // Return voltage (channel, volt[2], magnitude, 1lsb[4])
#define GCUR    0x17                            // Return current (channel, current[2], magnitude, 1lsb[4])
#define GPWR    0x18                            // Return power (channel, power[2], magnitude, 1lsb[4])
#define GSCF    0x19                            // Rwad/Write Shunt configuration
#define GPCY	0x1F				// Return power cycle status (state) state: 0, power cycle, nz, power cycle

// Broadcast commands

#define	BCP_ENUM	0x00			// Enumeration response

// Address programming command

#define PADD	0xFF				// Program address

// IRQ reason codes

#define	IRQ_REASON_NONE	0			// No IRQ reason (Used as enumerate response)
#define IRQ_REASON_ATBOOT 1			// IRQ at BOOT
#define IRQ_REASON_ACFAIL 2			// AC Failure during operation
#define IRQ_REASON_ACREST 3			// AC restored


// Misc Constants

#define EEBOOTSIG 0xFE				// Boot loader signature
#define EEADDR	0xFF				// Address eeprom location


/*
* Data structures
*/

// Packet structure
typedef struct {
	uint8_t	hcb;				// Header control
	uint8_t	addr;				// Dest addr
	uint8_t	cmd;				// Command
	uint8_t	params[MAXPARAMS];		// Parameters and CRC
} pkt_t;

// Han Packet State machine
typedef struct {
        struct{
            unsigned rxerr : 1;			// Error flag
            unsigned crcword : 1;		// True if 16 bit CRC's to be used
        };
	uint8_t	*pktb;				// Buffer pointer
	uint8_t	state;				// Packet State
        uint8_t crcerrs;                        // CRC errors
        uint8_t packettimeouts;                 // Packet timeouts

} phd_t;


// Receive data structure
typedef struct {
	uint8_t	c;				// Last char received
	uint8_t	packettimer;			// Packet time out timer
	uint8_t	state;				// State
	uint8_t	index;				// Buffer index
        struct{
            unsigned sub : 1;			// Substitute flag
            unsigned pready : 1;		// Packet Ready
        };
} rxi_t;


// Transmit data structure
typedef struct {
	uint8_t	state;				// TX State
	uint8_t	index;				// Buffer index
	uint8_t	blen;				// Buffer length to transmit
	uint8_t	tchar;
        struct{
            unsigned txbusy : 1;                // Busy flag
        };
} txi_t;


// Han interrupt structure
typedef struct {
	uint8_t	holdoff;			// holdoff time
	uint8_t	timer;				// Holdoff timer
	uint8_t	prescale;			// Mod128 timer
	uint8_t	reason;				// Reason for interrupt
        struct{
            unsigned flag : 1;			// irq flag
        };
} irq_t;


#endif

