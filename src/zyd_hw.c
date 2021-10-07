#pragma ident	"%Z%%M%	%I%	%E% SMI"

/*
 * ZD1211 wLAN driver
 * Device hardware control
 *
 * Control the ZD1211 chip and the RF chip.
 */

#include <sys/byteorder.h>
#include <sys/strsun.h>

#include "zyd_usb.h"
#include "zyd_reg.h"
#include "zyd_impl.h"
#include "zyd_hw.h"

/* RF configuration register width (bits) */
#define ZYD_AL2230_RF_BITS 24

/* io write sequences to initialize RF-independent PHY registers */
static const struct zyd_iowrite16 zyd_def_phy[] = ZYD_DEF_PHY;
static const struct zyd_iowrite16 zyd_def_phyB[] = ZYD_DEF_PHYB;

/*
 * Read a 32-bit I/O register.
 *
 *	sc	soft state
 *	reg	register number
 *	*val	place to store the value
 */
static zyd_res
zyd_read32(struct zyd_softc *sc, uint16_t reg, uint32_t *val)
{
	zyd_res result;
	uint16_t tmp[4];
	uint16_t regs[2];

	regs[0] = LE_16(ZYD_REG32_HI(reg));
	regs[1] = LE_16(ZYD_REG32_LO(reg));

	result = zyd_usb_ioread_req(&sc->usb, regs, sizeof (regs),
					tmp, sizeof (tmp));
	
	if (result != USB_SUCCESS)
		return (ZYD_FAILURE);
	
	if (tmp[0] != regs[0] || tmp[2] != regs[1]) {
		ZYD_WARN("ioread response doesn't match request");
		ZYD_WARN("requested regs %04x, %04x; got %04x, %04x",
			LE_16(regs[0]), LE_16(regs[1]),
			LE_16(tmp[0]), LE_16(tmp[2]));
		return (ZYD_FAILURE);
	}

	*val = ((uint32_t)LE_16(tmp[1]) << 16) |
		(uint32_t)LE_16(tmp[3]);

	return (ZYD_SUCCESS);
}

/*
 * Write a 32-bit I/O register.
 *
 *	sc	soft state
 *	reg	register number
 *	val	value to write
 */
static zyd_res
zyd_write32(struct zyd_softc *sc, uint16_t reg, uint32_t val)
{
	zyd_res result;
	uint16_t tmp[4];

	tmp[0] = LE_16(ZYD_REG32_HI(reg));
	tmp[1] = LE_16(val >> 16);
	tmp[2] = LE_16(ZYD_REG32_LO(reg));
	tmp[3] = LE_16(val & 0xffff);

	result = zyd_usb_cmd_send(&sc->usb, ZYD_CMD_IOWR, tmp, sizeof (tmp));

	return (result);
}

/*
 * Read a 16-bit I/O register.
 *
 *	sc	soft state
 *	reg	register number
 *	*val	place to store the value
 */
static zyd_res
zyd_read16(struct zyd_softc *sc, uint16_t reg, uint16_t *val)
{
	zyd_res result;
	uint16_t tmp[2];
	uint16_t regbuf;

	regbuf = LE_16(reg);

	result = zyd_usb_ioread_req(&sc->usb, &regbuf, sizeof (regbuf),
		tmp, sizeof (tmp));

	if (result != USB_SUCCESS)
		return (ZYD_FAILURE);

	if (tmp[0] != regbuf) {
		ZYD_WARN("ioread response doesn't match request");
		ZYD_WARN("requested reg %04x; got %04x",
			LE_16(regbuf), LE_16(tmp[0]));
		return (ZYD_FAILURE);
	}

	if (result != USB_SUCCESS)
		return (ZYD_FAILURE);
	
	*val = LE_16(tmp[1]);

	return (ZYD_SUCCESS);
}

/*
 * Write a 16-bit I/O register.
 *
 *	sc	soft state
 *	reg	register number
 *	val	value to write
 */
static zyd_res
zyd_write16(struct zyd_softc *sc, uint16_t reg, uint16_t val)
{
	zyd_res result;
	uint16_t tmp[2];

	tmp[0] = LE_16(ZYD_REG32_LO(reg));
	tmp[1] = LE_16(val & 0xffff);

	result = zyd_usb_cmd_send(&sc->usb, ZYD_CMD_IOWR, tmp, sizeof (tmp));

	return (result);
}

/*
 * Write an array of 16-bit registers.
 *
 *	sc	soft state
 *	*reqa	array of register-value pairs
 *	n	number of registers
 */
static zyd_res
zyd_write16a(struct zyd_softc *sc, const struct zyd_iowrite16 *reqa, int n)
{
	zyd_res res;
	int i;

	for (i=0; i<n; i++) {
		res = zyd_write16(sc, reqa[i].reg, reqa[i].value);
		if (res != ZYD_SUCCESS)
			return (ZYD_FAILURE);
	}

	return (ZYD_SUCCESS);
}


/*
 * Lock PHY registers.
 */
void
zyd_lock_phy(struct zyd_softc *sc)
{
	uint32_t tmp;

	(void)zyd_read32(sc, ZYD_MAC_MISC, &tmp);
	tmp &= ~ZYD_UNLOCK_PHY_REGS;
	(void)zyd_write32(sc, ZYD_MAC_MISC, tmp);
}

/*
 * Unlock PHY registers.
 */
void
zyd_unlock_phy(struct zyd_softc *sc)
{
	uint32_t tmp;

	(void)zyd_read32(sc, ZYD_MAC_MISC, &tmp);
	tmp |= ZYD_UNLOCK_PHY_REGS;
	(void)zyd_write32(sc, ZYD_MAC_MISC, tmp);
}

/*
 * Read MAC address from EEPROM.
 */
static zyd_res
zyd_read_mac(struct zyd_softc *sc)
{	
	uint32_t tmp;
	
	ZYD_DEBUG("read first part of mac");
	if (zyd_read32(sc, ZYD_EEPROM_MAC_ADDR_P1, &tmp) != ZYD_SUCCESS)
		return (ZYD_FAILURE);
	
	sc->macaddr[0] = tmp & 0xff;
	sc->macaddr[1] = tmp >>  8;
	sc->macaddr[2] = tmp >> 16;
	sc->macaddr[3] = tmp >> 24;

	ZYD_DEBUG("read second part of mac");
	
	if (zyd_read32(sc, ZYD_EEPROM_MAC_ADDR_P2, &tmp) != ZYD_SUCCESS)
		return (ZYD_FAILURE);

	sc->macaddr[4] = tmp & 0xff;
	sc->macaddr[5] = tmp >>  8;

	ZYD_DEBUG("done");

	return (ZYD_SUCCESS);
}

/*
 * Write bits to RF configuration register.
 */
static zyd_res
zyd_rfwrite(struct zyd_softc *sc, uint32_t val, int bits)
{
	uint16_t cr203;
	struct zyd_rfwrite req;
	uint16_t tmp;
	int bit;
	zyd_res res;
	int i;

	if (zyd_read16(sc, ZYD_CR203, &cr203) != ZYD_SUCCESS)
		return (ZYD_FAILURE);

	cr203 &= ~(ZYD_RF_IF_LE | ZYD_RF_CLK | ZYD_RF_DATA);

	req.code = LE_16(ZYD_RFCFG_VALUE);
	req.width = LE_16((uint16_t)bits);

	for (i = 0; i < bits; i++) {
		bit = (val & (1 << (bits - i - 1))) != 0;
		tmp = LE_16(cr203) | (bit ? LE_16(ZYD_RF_DATA) : 0);
		req.bit[i] = tmp;
	}
	res = zyd_usb_cmd_send(&sc->usb, ZYD_CMD_RFCFG, &req,
				sizeof(uint16_t)*(2+bits));

	if (res != ZYD_SUCCESS) {
		ZYD_WARN("failed configuring rf register");
		return (ZYD_FAILURE);
	}

	return (ZYD_SUCCESS);
}

/*
 * Control the LEDs.
 */
static void
zyd_set_led(struct zyd_softc *sc, int which, boolean_t on)
{
	uint32_t tmp;

	(void)zyd_read32(sc, ZYD_MAC_TX_PE_CONTROL, &tmp);
	tmp &= ~which;
	if (on == B_TRUE)
		tmp |= which;
	(void)zyd_write32(sc, ZYD_MAC_TX_PE_CONTROL, tmp);
}

/*
 * Init the RF chip.
 */
static zyd_res
zyd_al2230_rf_init(struct zyd_softc *sc)
{
	static const struct zyd_iowrite16 phyini[] = ZYD_AL2230_PHY;
	static const uint32_t rfini[] = ZYD_AL2230_RF;

	zyd_res res;
	int i;

	zyd_lock_phy(sc);
	
	/* init RF-dependent PHY registers */
	res = zyd_write16a(sc, phyini, ZYD_ARRAY_LENGTH(phyini));
	if (res != ZYD_SUCCESS) {
		zyd_unlock_phy(sc);
		return (ZYD_FAILURE);
	}

	/* init AL2230 radio */
	for (i = 0; i < ZYD_ARRAY_LENGTH(rfini); i++) {
		res = zyd_rfwrite(sc, rfini[i], ZYD_AL2230_RF_BITS);
		if (res != ZYD_SUCCESS) {
			zyd_unlock_phy(sc);
			return (ZYD_FAILURE);
		}
	}

	zyd_unlock_phy(sc);
	ZYD_DEBUG("RF chip initialized");
	
	return (ZYD_SUCCESS);
}

/*
 * Tune RF chip to a specified channel.
 */
static void
zyd_al2230_set_rf_channel(struct zyd_softc *sc, uint8_t chan)
{
	static const struct {
		uint32_t	r1, r2, r3;
	} rfprog[] = ZYD_AL2230_CHANTABLE;

	(void)zyd_rfwrite(sc, rfprog[chan - 1].r1, ZYD_AL2230_RF_BITS);
	(void)zyd_rfwrite(sc, rfprog[chan - 1].r2, ZYD_AL2230_RF_BITS);
	(void)zyd_rfwrite(sc, rfprog[chan - 1].r3, ZYD_AL2230_RF_BITS);

	(void)zyd_write16(sc, ZYD_CR138, 0x28);
	(void)zyd_write16(sc, ZYD_CR203, 0x06);
}

/*
 * Set MAC address.
 */
static void
zyd_set_macaddr(struct zyd_softc *sc, const uint8_t *addr)
{
	uint32_t tmp;

	tmp = addr[3] << 24 | addr[2] << 16 | addr[1] << 8 | addr[0];
	(void)zyd_write32(sc, ZYD_MAC_MACADRL, tmp);

	tmp = addr[5] << 8 | addr[4];
	(void)zyd_write32(sc, ZYD_MAC_MACADRH, tmp);
}

/*
 * Turn the radio transciever on/off.
 */
static void
zyd_al2230_switch_radio(struct zyd_softc *sc, boolean_t on)
{
	zyd_lock_phy(sc);
	
	(void)zyd_write16(sc, ZYD_CR11,  (on == B_TRUE) ? 0x00 : 0x04);
	(void)zyd_write16(sc, ZYD_CR251, (on == B_TRUE) ? 0x3f : 0x2f);
	
	zyd_unlock_phy(sc);
}

/*
 * Read data from EEPROM.
 */
static void
zyd_read_eeprom(struct zyd_softc *sc)
{
	uint32_t tmp;
	uint16_t val;
	int i;

	/* read RF chip type */
	(void)zyd_read32(sc, ZYD_EEPROM_POD, &tmp);
	sc->rf_rev = tmp & 0x0f;
	sc->pa_rev = (tmp >> 16) & 0x0f;
	ZYD_DEBUG("RF chip %x\n", sc->rf_rev);

	/* read regulatory domain (currently unused) */
	(void)zyd_read32(sc, ZYD_EEPROM_SUBID, &tmp);
	sc->regdomain = tmp >> 16;
	ZYD_DEBUG("regulatory domain %x\n", sc->regdomain);

	/* read Tx power calibration tables */
	for (i = 0; i < 7; i++) {
		(void)zyd_read16(sc, ZYD_EEPROM_PWR_CAL + i, &val);
		sc->pwr_cal[i * 2] = val >> 8;
		sc->pwr_cal[i * 2 + 1] = val & 0xff;

		(void)zyd_read16(sc, ZYD_EEPROM_PWR_INT + i, &val);
		sc->pwr_int[i * 2] = val >> 8;
		sc->pwr_int[i * 2 + 1] = val & 0xff;

		(void)zyd_read16(sc, ZYD_EEPROM_36M_CAL + i, &val);
		sc->ofdm36_cal[i * 2] = val >> 8;
		sc->ofdm36_cal[i * 2 + 1] = val & 0xff;

		(void)zyd_read16(sc, ZYD_EEPROM_48M_CAL + i, &val);
		sc->ofdm48_cal[i * 2] = val >> 8;
		sc->ofdm48_cal[i * 2 + 1] = val & 0xff;

		(void)zyd_read16(sc, ZYD_EEPROM_54M_CAL + i, &val);
		sc->ofdm54_cal[i * 2] = val >> 8;
		sc->ofdm54_cal[i * 2 + 1] = val & 0xff;
	}
}

/*
 * Finish ZD chip initialization.
 */
static zyd_res
zyd_hw_configure(struct zyd_softc *sc)
{
	zyd_res res;

	/* specify that the plug and play is finished */
	(void)zyd_write32(sc, ZYD_MAC_AFTER_PNP, 1);

	(void)zyd_read16(sc, ZYD_FIRMWARE_BASE_ADDR, &sc->fwbase);
	ZYD_DEBUG("firmware base address=0x%04x\n", sc->fwbase);

	/* retrieve firmware revision number */
	(void)zyd_read16(sc, sc->fwbase + ZYD_FW_FIRMWARE_REV, &sc->fw_rev);

	(void)zyd_write32(sc, ZYD_CR_GPI_EN, 0);
	(void)zyd_write32(sc, ZYD_MAC_CONT_WIN_LIMIT, 0x7f043f);

	/* disable interrupts */
	(void)zyd_write32(sc, ZYD_CR_INTERRUPT, 0);

	/* Init RF chip-independent PHY registers */
	zyd_lock_phy(sc);
	res = zyd_write16a(sc, zyd_def_phy, ZYD_ARRAY_LENGTH(zyd_def_phy));
	zyd_unlock_phy(sc);
	
	if (res != ZYD_SUCCESS)
		return (ZYD_FAILURE);

	/* HMAC initialization magic */
	zyd_write32(sc, ZYD_MAC_ACK_EXT, 0x00000020);
	zyd_write32(sc, ZYD_CR_ADDA_MBIAS_WT, 0x30000808);
	zyd_write32(sc, ZYD_MAC_RETRY, 0x00000002);
	zyd_write32(sc, ZYD_MAC_SNIFFER, 0x00000000);
	zyd_write32(sc, ZYD_MAC_RXFILTER, 0x00000000);
	zyd_write32(sc, ZYD_MAC_GHTBL, 0x00000000);
	zyd_write32(sc, ZYD_MAC_GHTBH, 0x80000000);
	zyd_write32(sc, ZYD_MAC_MISC, 0x000000a4);
	zyd_write32(sc, ZYD_CR_ADDA_PWR_DWN, 0x0000007f);
	zyd_write32(sc, ZYD_MAC_BCNCFG, 0x00f00401);
	zyd_write32(sc, ZYD_MAC_PHY_DELAY2, 0x00000000);
	zyd_write32(sc, ZYD_MAC_ACK_EXT, 0x00000080);
	zyd_write32(sc, ZYD_CR_ADDA_PWR_DWN, 0x00000000);
	zyd_write32(sc, ZYD_MAC_SIFS_ACK_TIME, 0x00000100);
	zyd_write32(sc, ZYD_MAC_DIFS_EIFS_SIFS, 0x0547c032);
	zyd_write32(sc, ZYD_CR_RX_PE_DELAY, 0x00000070);
	zyd_write32(sc, ZYD_CR_PS_CTRL, 0x10000000);
	zyd_write32(sc, ZYD_MAC_RTSCTSRATE, 0x02030203);
	zyd_write32(sc, ZYD_MAC_RX_THRESHOLD, 0x000c0640);
	zyd_write32(sc, ZYD_MAC_BACKOFF_PROTECT, 0x00000114);
	
	return (ZYD_SUCCESS);
}

/*
 * Set active channel number.
 */
void
zyd_hw_set_channel(struct zyd_softc *sc, uint8_t chan)
{
	zyd_lock_phy(sc);

	zyd_al2230_set_rf_channel(sc, chan);

	/* update Tx power */
	(void)zyd_write32(sc, ZYD_CR31, sc->pwr_int[chan - 1]);
	(void)zyd_write32(sc, ZYD_CR68, sc->pwr_cal[chan - 1]);

	zyd_unlock_phy(sc);
}

/*
 * Initialize hardware.
 */
zyd_res
zyd_hw_init(struct zyd_softc *sc)
{
	if (zyd_usb_init(&sc->usb, sc->dip) != ZYD_SUCCESS)
		return (ZYD_FAILURE);

	if (zyd_read_mac(sc) != ZYD_SUCCESS) {
		ZYD_WARN("failed to read MAC address");
		return (ZYD_FAILURE);
	}

	zyd_read_eeprom(sc);
	if (sc->rf_rev != ZYD_RF_AL2230) {
		ZYD_WARN("unsupported RF chip type %d", sc->rf_rev);
		return (ZYD_FAILURE);
	}
	
	if (zyd_hw_configure(sc) != ZYD_SUCCESS) {
		ZYD_WARN("Failed to configure hardware.");
		return (ZYD_FAILURE);
	}
	
	if (zyd_al2230_rf_init(sc) != ZYD_SUCCESS) {	
		ZYD_WARN("Failed to configure RF chip");
		return (ZYD_FAILURE);
	}
	
	ZYD_DEBUG("Zyd HW inited. MAC=%02X:%02X:%02X:%02X:%02X:%02X",
	          sc->macaddr[0],sc->macaddr[1],sc->macaddr[2],
	          sc->macaddr[3],sc->macaddr[4],sc->macaddr[5]);

	return (ZYD_SUCCESS);
}

/*
 * Deinitialize hardware.
 */
void
zyd_hw_deinit(struct zyd_softc *sc)
{
	zyd_usb_deinit(&sc->usb);
}

/*
 * Activate the device.
 */
zyd_res
zyd_hw_start(struct zyd_softc *sc)
{
	zyd_res res;
	
	if (zyd_usb_data_in_enable(&sc->usb) != ZYD_SUCCESS) {
		ZYD_WARN("error starting rx transfer");
		return (ZYD_FAILURE);
	}

	ZYD_DEBUG("setting MAC address\n");
	zyd_set_macaddr(sc, sc->macaddr);

	/* we'll do software WEP decryption for now */
	ZYD_DEBUG("setting encryption type\n");
	res = zyd_write32(sc, ZYD_MAC_ENCRYPTION_TYPE, ZYD_ENC_SNIFFER);
	if (res != ZYD_SUCCESS)
		return (ZYD_FAILURE);

	/* promiscuous mode */
	(void)zyd_write32(sc, ZYD_MAC_SNIFFER, 0);

	/* try to catch all packets */
	(void)zyd_write32(sc, ZYD_MAC_RXFILTER, ZYD_FILTER_BSS);
	
	/* switch radio transmitter ON */
	zyd_al2230_switch_radio(sc, B_TRUE);

	/* set basic rates */
	(void)zyd_write32(sc, ZYD_MAC_BAS_RATE, 0x0003);

	/* set mandatory rates */
	(void)zyd_write32(sc, ZYD_MAC_MAN_RATE, 0x000f);

	/* enable interrupts */
	(void)zyd_write32(sc, ZYD_CR_INTERRUPT, ZYD_HWINT_MASK);

	zyd_set_led(sc, ZYD_LED2, B_TRUE);
	
	return (ZYD_SUCCESS);
}

/*
 * Deactivate the device.
 */
void
zyd_hw_stop(struct zyd_softc *sc)
{
	/* switch radio transmitter OFF */
	zyd_al2230_switch_radio(sc, B_FALSE);

	/* disable reception */
	(void)zyd_write32(sc, ZYD_MAC_RXFILTER, 0);

	/* disable interrupts */
	(void)zyd_write32(sc, ZYD_CR_INTERRUPT, 0);

	zyd_usb_data_in_disable(&sc->usb);
	zyd_set_led(sc, ZYD_LED2, B_FALSE);
}

/*
 * Compute frame duration in microseconds.
 *
 * Also determine wheter frame should use length-extension service.
 *	rate	in half-MBits/second
 */
int zyd_compute_frame_duration(int rate, uint16_t frame_size, uint8_t *service)
{
	uint32_t bits_2x;
	uint32_t mbps_2x;
	uint32_t usecs;
	uint32_t r;
	
	bits_2x = (uint32_t)frame_size * 8 * 2;
	mbps_2x = rate;
	
	switch(rate) {
	case 11: /* CCK 5.5M */
		bits_2x += 10; /* rounding up */
		break;
	case 22: /* CCK 11M */
		r = bits_2x % 22;
		if (r > 0 && r <= 6)
			*service |= ZYD_TX_SERVICE_LENGTH_EXTENSION;
		else
			*service &= ~ZYD_TX_SERVICE_LENGTH_EXTENSION;
		bits_2x += 21; /* rounding up */
		break;
	}
	
	usecs = bits_2x / mbps_2x;
	
	return (usecs);
}

/*
 * Transmit a 802.11 frame.
 *
 * (FIXME: Fill all fields in properly)
 *
 * Will probably need additional arguments.
 * Constructs a packet from zyd_tx_header and 802.11 frame data
 * and sends it to the chip.
 */
zyd_res
zyd_hw_send_frame(struct zyd_softc *sc, mblk_t *mp)
{
	void *buffer;
	uint8_t *buf_data;
	struct zyd_tx_header *buf_hdr;
	int len, total_len;
	int frame_size, additional_size;
	int bit2x_rate;
	uint8_t service;
	uint16_t frame_duration;

	/* Get total length of frame */
	len = msgsize(mp);
	
	ZYD_DEBUG("frame length %d", len);

	/* Total number of bytes to send to the chip */
	total_len = sizeof (struct zyd_tx_header) + len;

	/* Allocate a buffer to store the packet */
	buffer = kmem_alloc(total_len, KM_NOSLEEP);
	if (buffer == NULL)
		return (ZYD_FAILURE);

	/* Get pointers to header buffer and 802.11 frame buffer */
	buf_hdr = (struct zyd_tx_header *)buffer;
	buf_data = ((uint8_t *)buffer) + sizeof (struct zyd_tx_header);

	/*** Fill header information ***/
	
	/*
	 * Frame size in bytes including CRC32 checksum
	 * (FIXME: include additional fields for WEP)
	 */
	frame_size = len + IEEE80211_FCS_LEN;
	buf_hdr->frame_size = LE_16(frame_size);
	
	/*
	 * Compute "packet size". What the 10 stands for,
	 * nobody knows. FIXME: This should be different for ZD1211B.
	 */
	additional_size = sizeof (struct zyd_tx_header) + 10;
	buf_hdr->packet_size = LE_16(frame_size + additional_size);

	/* FIXME: fill these in properly */
	buf_hdr->rate_mod_flags = LE_8(0x02); /* CCK 11 MBit/s */
	buf_hdr->type_flags = LE_8(ZYD_TX_FLAG_TYPE(ZYD_TX_TYPE_DATA));
	
	/*
	 * Compute frame duration and length-extension service flag.
	 */
	bit2x_rate = 22; /* CCK 11 MBit/s */
	service = 0x00;
	frame_duration = zyd_compute_frame_duration(
		bit2x_rate, frame_size, &service);

	buf_hdr->frame_duration = LE_16(frame_duration);
	buf_hdr->service = LE_8(service);
	buf_hdr->next_frame_duration = LE_16(0);

	/* Copy the 802.11 frame */
	mcopymsg(mp, buf_data);

	/* Send it to the device */
	zyd_usb_send_packet(&sc->usb, buffer, total_len);

	/* Free memory */
	kmem_free(buffer, total_len);
	buffer = NULL;
	buf_hdr = NULL;
	buf_data = NULL;

	return (ZYD_SUCCESS);
}
