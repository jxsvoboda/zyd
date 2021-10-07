#ifndef _SYS_ZYD_IMPL_H
#define	_SYS_ZYD_IMPL_H

#pragma ident	"%Z%%M%	%I%	%E% SMI"

#ifdef __cplusplus
extern "C" {
#endif

#include <sys/ddi.h>
#include <sys/sunddi.h>
#include <sys/mac.h>
#include <sys/mac_wifi.h>
#include <sys/net80211.h>

#define USBDRV_MAJOR_VER 2
#define USBDRV_MINOR_VER 0
#include <sys/usb/usba.h>

/* Packed structures are used to compose/parse messages to the chip */
#define __packed        __attribute__((__packed__))

/* Define to turn debug messages on, undefine to turn them off */
#undef ZYD_DEBUG_ON

/* Debugging macros */
#define	ZYD_WARN(fmt, ...)	cmn_err(CE_WARN,  "%s: " fmt, __func__, ##__VA_ARGS__)
#define	ZYD_FATAL(fmt, ...)	cmn_err(CE_PANIC, "%s: " fmt, __func__, ##__VA_ARGS__)

#ifdef ZYD_DEBUG_ON
#define	ZYD_DEBUG(fmt, ...)	cmn_err(CE_NOTE,  "%s: " fmt, __func__, ##__VA_ARGS__)
#else
#define ZYD_DEBUG(fmt, ...)
#endif

/* Return the number of fields of an array */
#define ZYD_ARRAY_LENGTH(arr) (sizeof (arr) / sizeof ((arr)[0]))

/*
 * Result type: all functions beginning with zyd_
 * should use this to indicate success or failure.
 * (except for public funcions, of course)
 *
 * Detecting error: always use (value != ZYD_SUCCESS)
 * Indicating error: return ZYD_FAILURE
 */
typedef enum {
	ZYD_SUCCESS,
	ZYD_FAILURE
} zyd_res;

/*
 * USB-safe mutual exclusion object.
 */
typedef struct {
	boolean_t initialized;	/* B_TRUE if properly initialized */
	boolean_t held;		/* B_TRUE if the object is held */
	kmutex_t lock;		/* serialize access */
	kcondvar_t wait;	/* for waiting on release */
} zyd_serial_t;

/*
 * Holds an ioread request status.
 */
struct zyd_ioread {
	volatile boolean_t pending;	/* ioread is in progress */
	volatile boolean_t done;	/* response has been received */
	volatile boolean_t exc;		/* an exception has occured */

	void *buffer;			/* response buffer */
	int buf_len;			/* buffer size (bytes) */
};

/*
 * State of data_in requests.
 */
struct zyd_usb_datain {
	boolean_t running;		/* data_in reception is enabled */
};

/*
 * USB state.
 */
struct zyd_usb {
	/* Copy of sc->dip */
	dev_info_t 		*dip;

	/* Device configuration information */
	usb_client_dev_data_t	*cdata;

	/* Communication pipe handles */
	usb_pipe_handle_t	pipe_data_in;
	usb_pipe_handle_t	pipe_data_out;
	usb_pipe_handle_t	pipe_cmd_in;
	usb_pipe_handle_t	pipe_cmd_out;

	/* Communication endpoint data (copied from descriptor tree) */
	usb_ep_data_t		ep_data_in;
	usb_ep_data_t		ep_data_out;
	usb_ep_data_t		ep_cmd_in;
	usb_ep_data_t		ep_cmd_out;

	/* cmd_in transfer enabled */
	boolean_t		cmd_in_running;

	/* Current ioread request (if any) */
	struct zyd_ioread	io_read;

	/* State of the data_in pipe */
	struct zyd_usb_datain	data_in;	
};

/*
 * per-instance soft-state structure
 */
struct zyd_softc {
	/* Serialize access to the soft_state/device */
	zyd_serial_t		serial;

	dev_info_t		*dip;
	
	/* timeout for scanning */
	timeout_id_t		timeout_id;
	
	/* USB-specific data */
	struct zyd_usb		usb;
	
	/* MAC address */
	uint8_t			macaddr[IEEE80211_ADDR_LEN];

	/* net80211 data */
	struct ieee80211com 	ic;

	/* Data from EEPROM */
	uint16_t		fwbase;
	uint8_t			regdomain;
	uint16_t		fw_rev;
	uint8_t			rf_rev;
	uint8_t			pa_rev;
	uint8_t			pwr_cal[14];
	uint8_t			pwr_int[14];
	uint8_t			ofdm36_cal[14];
	uint8_t			ofdm48_cal[14];
	uint8_t			ofdm54_cal[14];

	/* net80211 original state change handler */	
	int			(*newstate)(ieee80211com_t *,
					enum ieee80211_state, int);
};

/*
 * Calculate the byte offset of a struct member
 */
#define STRUCT_MEMBER_OFFSET(struc, member)\
(\
	(size_t) &( ((struc *)(0)) -> member )\
)

/*
 * The 'struct ieee80211com ic' is stored inside 'struct zyd_softc'.
 * Using the knowledge of the ic member position,
 * convert a pointer to 'ic' to a pointer to the zyd_softc.
 */
#define ZYD_IC_TO_SOFTC(ic)\
(\
	(struct zyd_softc *) (\
		(uint8_t *)(ic) - STRUCT_MEMBER_OFFSET(struct zyd_softc, ic)\
	)\
)

/*
 * The 'struct zyd_usb usb' is stored inside 'struct zyd_softc'.
 * Using the knowledge of the usb member position,
 * convert a pointer to 'usb' to a pointer to the zyd_softc.
 */
#define ZYD_USB_TO_SOFTC(usbp)\
(\
	(struct zyd_softc *) (\
		(uint8_t *)(usbp) - STRUCT_MEMBER_OFFSET(struct zyd_softc, usb)\
	)\
)

/* Generic usb command to the ZD chip */
struct zyd_cmd {
	uint16_t cmd_code;
	uint8_t data[64];
} __packed;

/* RF-config request */
struct zyd_rfwrite {
	uint16_t	code;
	uint16_t	width;
	uint16_t	bit[32];
};

/* 16-bit I/O register write request */
struct zyd_iowrite16 {
	uint16_t	reg;
	uint16_t	value;
};

/* ZD prepends this header to an incoming frame. */
struct zyd_plcphdr {
	uint8_t		signal;
	uint8_t		reserved[2];
	uint16_t	service;	/* unaligned! */
} __packed;

/* ZD appends this footer to an incoming frame. */
struct zyd_rx_stat {
	uint8_t	rssi;
	uint8_t	signal_cck;
	uint8_t signal_ofdm;
	uint8_t cipher;
	uint8_t	flags;
} __packed;

/*
 * Prepended to the 802.11 frame when sending to data_out.
 */
struct zyd_tx_header {
	uint8_t rate_mod_flags;
	uint16_t frame_size;
	uint8_t type_flags;
	uint16_t packet_size;
	uint16_t frame_duration;
	uint8_t service;
	uint16_t next_frame_duration;
} __packed;

/* Bits for rate_mod_flags */
#define ZYD_TX_RMF_RATE(rmf)	((rmf) & 0x0f)
#define ZYD_TX_RMF_OFDM		0x10
#define ZYD_TX_RMF_SH_PREAMBLE	0x20	/* CCK */
#define ZYD_TX_RMF_5GHZ		0x40	/* OFDM */

/* Bits for type_flags */
#define ZYD_TX_FLAG_BACKOFF	0x01
#define ZYD_TX_FLAG_MULTICAST	0x02
#define ZYD_TX_FLAG_TYPE(t)	(((t) & 0x3) << 2)
#define ZYD_TX_TYPE_DATA	0
#define ZYD_TX_TYPE_PS_POLL	1
#define ZYD_TX_TYPE_MGMT	2
#define ZYD_TX_TYPE_CTL		3

#define ZYD_TX_FLAG_WAKEUP	0x10
#define ZYD_TX_FLAG_RTS		0x20
#define ZYD_TX_FLAG_ENCRYPT	0x40
#define ZYD_TX_FLAG_CTS_TO_SELF	0x80

#define ZYD_TX_SERVICE_LENGTH_EXTENSION		0x80

/* frame FCS length in bytes.. CRC-32 is used */
#define IEEE80211_FCS_LEN	4

/* 
 * Composite packet signature: Indicates that multiple frames
 * have been merged into a single USB transfer.
 */
#define ZYD_RX_COMPOSITE_SIG_1	0x7E
#define ZYD_RX_COMPOSITE_SIG_2	0x69

/*
 * Time in miliseconds to stay on one channel during scan.
 */
#define ZYD_DWELL_TIME 200000

/*
 * Device operations
 */
int	zyd_attach(dev_info_t *dip, ddi_attach_cmd_t cmd);
int	zyd_detach(dev_info_t *dip, ddi_detach_cmd_t cmd);

/*
 * GLD specific operations
 */
int	zyd_m_stat(void *arg, uint_t stat, uint64_t *val);
int	zyd_m_start(void *arg);
void	zyd_m_stop(void *arg);
int	zyd_m_unicst(void *arg, const uint8_t *macaddr);
int	zyd_m_multicst(void *arg, boolean_t add, const uint8_t *m);
int	zyd_m_promisc(void *arg, boolean_t on);
void	zyd_m_ioctl(void *arg, queue_t *wq, mblk_t *mp);
mblk_t  *zyd_m_tx(void *arg, mblk_t *mp);

/* 
 * net80211 callback functions
 */
int	zyd_send(ieee80211com_t *ic, mblk_t *mp, uint8_t type);
int	zyd_newstate(struct ieee80211com *ic, enum ieee80211_state state, 
	             int arg);

/*
 * Receive packet
 */
void	zyd_receive(struct zyd_softc *sc, const uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* _SYS_ZYD_IMPL_H */
