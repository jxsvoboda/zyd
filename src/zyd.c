#pragma ident	"%Z%%M%	%I%	%E% SMI"

/*
 * ZD1211 wLAN driver
 * Main module
 *
 * Implement module entry points, MAC entry points; interface
 * with MAC and net80211.
 */

#include <sys/ddi.h>
#include <sys/sunddi.h>
#include <sys/conf.h>
#include <sys/modctl.h>
#include <sys/mac.h>
#include <sys/mac_wifi.h>
#include <sys/net80211.h>
#include <sys/strsun.h>

#include "zyd.h"
#include "zyd_hw.h"
#include "zyd_impl.h"
#include "zyd_util.h"
#include "zyd_reg.h"

/* Global state pointer for managing per-device soft states */
static void *zyd_ssp = NULL;

/* Driver identification */
static char zyd_ident[] = ZYD_DRV_DESC " " ZYD_DRV_REV;

/*
 * Mac Call Back entries
 */
static mac_callbacks_t zyd_m_callbacks = {
        MC_IOCTL,	/* Denotes which callbacks are set */
        zyd_m_stat,	/* Get the value of a statistic */
        zyd_m_start,	/* Start the device */
        zyd_m_stop,	/* Stop the device */
        zyd_m_promisc,	/* Enable or disable promiscuous mode */
        zyd_m_multicst,	/* Enable or disable a multicast addr */
        zyd_m_unicst,	/* Set the unicast MAC address */
        zyd_m_tx,	/* Transmit a packet */
        NULL,		/* Get the device resources */
        zyd_m_ioctl,	/* Process an unknown ioctl */
        NULL		/* Get capability information */
};

/*
 *  Module Loading Data & Entry Points
 */
DDI_DEFINE_STREAM_OPS(
	zyd_devops,	/* name */
	nulldev,	/* identify */
	nulldev,	/* probe */
	zyd_attach,	/* attach */
	zyd_detach,	/* detach */
	nodev,		/* reset */
	NULL,		/* getinfo */
	D_MP,		/* flag */
	NULL		/* stream_tab */
);

static struct modldrv zyd_modldrv = {
	&mod_driverops,	/* drv_modops */
	zyd_ident,	/* drv_linkinfo */
	&zyd_devops	/* drv_dev_ops */
};

static struct modlinkage zyd_ml = {
	MODREV_1,		/* ml_rev */
	{ &zyd_modldrv, NULL }	/* ml_linkage */
};

/*
 * Wireless-specific structures
 */
static const struct ieee80211_rateset zyd_rateset_11b = { 
	4, {2, 4, 11, 22} /* units are 0.5Mbit! */
};

static const struct ieee80211_rateset zyd_rateset_11g = { 
	12, {2, 4, 11, 22, 12, 18, 24, 36, 48, 72, 96, 108}
};

/*
 * Internal functions
 */

/*
 * Register with the MAC layer.
 */
static zyd_res
zyd_mac_init(struct zyd_softc *sc)
{
	struct ieee80211com	*ic = &sc->ic;
	mac_register_t		*macp;
	wifi_data_t		wd = { 0 };
	int			err;
	
	/*
	 * Initialize mac structure
	 */
	macp = mac_alloc(MAC_VERSION);
	if (macp == NULL) {
		ZYD_WARN("mac_alloc() failed");
		return (ZYD_FAILURE);
	}

	/*
	 * Initialize pointer to device specific functions
	 */
	wd.wd_secalloc = WIFI_SEC_NONE;
	wd.wd_opmode = sc->ic.ic_opmode;
	IEEE80211_ADDR_COPY(wd.wd_bssid, ic->ic_macaddr);
	
	macp->m_type_ident	= MAC_PLUGIN_IDENT_WIFI;
	macp->m_driver		= sc;
	macp->m_dip		= sc->dip;
	macp->m_src_addr	= ic->ic_macaddr;
	macp->m_callbacks	= &zyd_m_callbacks;
	macp->m_min_sdu		= 0;
	macp->m_max_sdu		= IEEE80211_MTU;
	macp->m_pdata		= &wd;
	macp->m_pdata_size	= sizeof (wd);
	
	/*
	 * Register the macp to mac
	 */
	err = mac_register(macp, &sc->ic.ic_mach);
	mac_free(macp);
	
	if (err != DDI_SUCCESS) {
		ZYD_WARN("mac_register() failed");
		return (ZYD_FAILURE);
	}
	
	return (ZYD_SUCCESS);
}

/*
 * Register with net80211.
 */
static void
zyd_wifi_init(struct zyd_softc *sc)
{
	struct ieee80211com *ic = &sc->ic;
	int	i;
	
	/*
	 * Initialize the WiFi part, which will be used by generic layer
	 */
	ic->ic_phytype	= IEEE80211_T_OFDM;
	ic->ic_opmode	= IEEE80211_M_STA;
	ic->ic_state	= IEEE80211_S_INIT;
	ic->ic_maxrssi	= 100; /* experimental number */
	ic->ic_xmit	= zyd_send;
	
	/* set device capabilities */
	ic->ic_caps =
	    IEEE80211_C_MONITOR |	/* monitor mode supported */
	    IEEE80211_C_TXPMGT |	/* tx power management */
	    IEEE80211_C_SHPREAMBLE |	/* short preamble supported */
	    IEEE80211_C_WEP;		/* s/w WEP */
	
	/* Copy MAC address */
	IEEE80211_ADDR_COPY(ic->ic_macaddr, sc->macaddr);
	
	/*
	 * set supported .11b and .11g rates
	 */
	ic->ic_sup_rates[IEEE80211_MODE_11B] = zyd_rateset_11b;
	ic->ic_sup_rates[IEEE80211_MODE_11G] = zyd_rateset_11g;
	
	/*
	 * set supported .11b and .11g channels(1 through 14)
	 */
	/* FIXME: Allowed channels depend on region */
	for (i = 1; i <= 14; i++) {
		ic->ic_sup_channels[i].ich_freq  =
		    ieee80211_ieee2mhz(i, IEEE80211_CHAN_2GHZ);
		ic->ic_sup_channels[i].ich_flags =
		    IEEE80211_CHAN_CCK | IEEE80211_CHAN_OFDM |
		    IEEE80211_CHAN_DYN | IEEE80211_CHAN_2GHZ;
	}

	/*
	 * Init generic layer (it cannot fail)
	 */
	ieee80211_attach(ic);
	
	/* Must be after attach! */
	sc->newstate = ic->ic_newstate;
	ic->ic_newstate = zyd_newstate;

	ieee80211_media_init(ic);
}

/*
 * Set the current channel.
 */
static void
zyd_set_channel(struct zyd_softc *sc, struct ieee80211_channel *c)
{
	struct ieee80211com *ic = &sc->ic;
	u_int chan;

	chan = ieee80211_chan2ieee(ic, c);
	if (chan == 0 || chan == IEEE80211_CHAN_ANY)
		return;
	
	ZYD_DEBUG("setting channel %d", chan);
	
	zyd_serial_enter(sc, ZYD_NO_SIG);
	zyd_hw_set_channel(sc, chan);
	zyd_serial_exit(sc);
	
	ZYD_DEBUG("successfuly set channel %d", chan);
}

/*
 * Timeout function for scanning.
 *
 * Called at the end of each scanning round.
 */
void
zyd_next_scan(void *arg)
{
	struct zyd_softc	*sc = arg;
	struct ieee80211com	*ic = &sc->ic;

	sc->timeout_id = 0;
	
	if (ic->ic_state == IEEE80211_S_SCAN) {
		sc->timeout_id = timeout(zyd_next_scan, sc,
					drv_usectohz(ZYD_DWELL_TIME));
		ieee80211_next_scan(ic);
	}
}

/*
 * Debug callback for net80211 to list all discovered nodes.
 */
void
zyd_iter_func(void *arg, ieee80211_node_t *in)
{
	struct ieee80211com *ic = (struct ieee80211com *)arg;
	
	ZYD_DEBUG("Node %02X:%02X:%02X:%02X:%02X:%02X at channel %d",
	          in->in_macaddr[0],in->in_macaddr[1],in->in_macaddr[2],
	          in->in_macaddr[3],in->in_macaddr[4],in->in_macaddr[5],
	          ieee80211_chan2ieee(ic, in->in_chan));
	(void)ic; /* suppress warning when debugging off */
}

/*
 * Extract a 802.11 frame from the received packet and forward it to net80211.
 */
void
zyd_receive(struct zyd_softc *sc, const uint8_t *buf, uint16_t len)
{
	const struct zyd_plcphdr	*plcp;
	const struct zyd_rx_stat	*stat;
	struct ieee80211com		*ic = &sc->ic;
	struct ieee80211_frame		*wh;
	struct ieee80211_node		*in;
	int	rlen; /* Actual frame length */
	mblk_t	*m;
	
	if (len < ZYD_MIN_FRAGSZ) {
		/* Packet is too short, silently drop it */
		return;
	}

	plcp = (const struct zyd_plcphdr *)buf;
	stat = (const struct zyd_rx_stat *)
	       (buf + len - sizeof (struct zyd_rx_stat));

	if (stat->flags & ZYD_RX_ERROR) {
		/* Frame is corrupted, silently drop it */
		return;
	}

	/* compute actual frame length */
	rlen = len - sizeof (struct zyd_plcphdr) -
	       sizeof (struct zyd_rx_stat) - IEEE80211_CRC_LEN;

	m = allocb(rlen, BPRI_MED);
	if (m == NULL)	{
		ZYD_WARN("memory allocation failed");
		return;
	}

	/* Copy frame to new buffer */
	bcopy(buf + sizeof (struct zyd_plcphdr), m->b_wptr, rlen);
	m->b_wptr += rlen;
	
	/* Send frame to net80211 stack */
	wh = (struct ieee80211_frame *)m->b_rptr;
	in = ieee80211_find_rxnode(ic, wh);
	(void)ieee80211_input(ic, m, in, stat->rssi, 0);
	ieee80211_free_node(in);
}

/*
 * Device operations
 */

/*
 * Binding the driver to a device.
 *
 * Concurrency: Until zyd_attach() returns with success,
 * the only other entry point that can be executed is getinfo().
 * Thus no locking here yet.
 */
int
zyd_attach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	struct zyd_softc	*sc;
	int			instance;
	int			err;
	
	ZYD_DEBUG("begin");
	
	if (cmd != DDI_ATTACH)
		return (DDI_FAILURE);
	
	instance = ddi_get_instance(dip);
	err = ddi_soft_state_zalloc(zyd_ssp, instance);
	
	if (err != DDI_SUCCESS) {
		ZYD_WARN("unable to allocate soft state");
		return (DDI_FAILURE);
	}
	
	sc = ddi_get_soft_state(zyd_ssp, instance);
	sc->dip = dip;
	sc->timeout_id = 0;
	
	if (zyd_hw_init(sc) != ZYD_SUCCESS) {
		ddi_soft_state_free(zyd_ssp, instance);
		return (DDI_FAILURE);
	}

	zyd_wifi_init(sc);

	if (zyd_mac_init(sc) != DDI_SUCCESS) {
		ddi_soft_state_free(zyd_ssp, instance);
		return (DDI_FAILURE);
	}
	
	/* Initialize locking */
	zyd_serial_init(sc);
	
	return (DDI_SUCCESS);
}

/*
 * Detach the driver from a device.
 *
 * Concurrency: Will be called only after a successful attach
 * (and not concurrently).
 */
int
zyd_detach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	struct zyd_softc	*sc = NULL;
	int			err;
	
	ZYD_DEBUG("begin");
	
	if (cmd != DDI_DETACH)
		return (DDI_FAILURE);
	
	sc = ddi_get_soft_state(zyd_ssp, ddi_get_instance(dip));
	ASSERT(sc != NULL);

	/* Claim control over the device */
	zyd_serial_enter(sc, ZYD_NO_SIG);
	
	/*
	 * Unregister from the MAC layer subsystem
	 */
	err = mac_unregister(sc->ic.ic_mach);
	if (err != DDI_SUCCESS) {
		zyd_serial_exit(sc);
		return (err);
	}
	/*
	 * Detach ieee80211
	 */
	ieee80211_detach(&sc->ic);

	/* Shutdown hardware */
	zyd_hw_deinit(sc);
	
	/* At this point it should be safe to release & destroy the locks */
	zyd_serial_exit(sc);
	zyd_serial_deinit(sc);
	
	ddi_soft_state_free(zyd_ssp, ddi_get_instance(dip));
	return (DDI_SUCCESS);
}

/*
 * Mac Call Back functions
 */

/*
 * Read device statistic information.
 */
int
zyd_m_stat(void *arg, uint_t stat, uint64_t *val)
{
	ZYD_DEBUG("begin");
	
	return (DDI_FAILURE);
}

/*
 * Start the device.
 *
 * Concurrency: Presumably fully concurrent, must lock.
 */
int
zyd_m_start(void *arg)
{
	struct zyd_softc	*sc = (struct zd1211_softc *)arg;
	
	ZYD_DEBUG("begin");
	
	zyd_serial_enter(sc, ZYD_NO_SIG);
	
	if (zyd_hw_start(sc) != ZYD_SUCCESS) {
		zyd_serial_exit(sc);
		return (DDI_FAILURE);
	}

	sc->ic.ic_state = IEEE80211_S_INIT;

	zyd_serial_exit(sc);

	return (DDI_SUCCESS);
}

/*
 * Stop the device.
 */
void
zyd_m_stop(void *arg)
{
	struct zyd_softc	*sc = (struct zd1211_softc *)arg;
	struct ieee80211com	*ic = &sc->ic;
	
	ZYD_DEBUG("begin");
	
	ieee80211_cancel_scan(ic);

	if (sc->timeout_id != 0)
		untimeout(sc->timeout_id);
	
	zyd_serial_enter(sc, ZYD_NO_SIG);
	zyd_hw_stop(sc);
	zyd_serial_exit(sc);
}

/*
 * Change the MAC address of the device.
 */
int
zyd_m_unicst(void *arg, const uint8_t *macaddr)
{
	ZYD_DEBUG("begin");
	
	return (DDI_FAILURE);
}

/*
 * Enable/disable multicast.
 */
int
zyd_m_multicst(void *arg, boolean_t add, const uint8_t *m)
{
	ZYD_DEBUG("Joining multicast groups is not implemented yet");

	return (DDI_SUCCESS);
}

/*
 * Enable/disable promiscuous mode.
 */
int
zyd_m_promisc(void *arg, boolean_t on)
{
	ZYD_DEBUG("Promiscuous mode is not implemented yet");
	
	return (DDI_SUCCESS);
}

/*
 * IOCTL request.
 */
void
zyd_m_ioctl(void *arg, queue_t *wq, mblk_t *mp)
{
	struct zyd_softc	*sc = (struct zd1211_softc *)arg;
	struct ieee80211com	*ic = &sc->ic;
	
	ZYD_DEBUG("begin");

	/* DEBUG: List access points */
	ieee80211_iterate_nodes(&ic->ic_scan, zyd_iter_func, ic);
	
	if (ieee80211_ioctl(ic, wq, mp) == ENETRESET)
		ieee80211_new_state(ic, IEEE80211_S_SCAN, -1);
}

/*
 * Transmit a data frame.
 */
mblk_t *
zyd_m_tx(void *arg, mblk_t *mp)
{
	struct zyd_softc	*sc = (struct zd1211_softc *)arg;
	struct ieee80211com	*ic = &sc->ic;
	mblk_t *next;
	
	ZYD_DEBUG("begin");

	while (mp != NULL) {
		next = mp->b_next;
		mp->b_next = NULL;

		if (zyd_send(ic, mp, IEEE80211_FC0_TYPE_DATA) != DDI_SUCCESS) {
			mp->b_next = next;
			break;
		}
		mp = next;
	}

	return (mp);
}


/*
 * net80211 callback functions
 */

/*
 * xxx_send callback for net80211.
 *
 * Called when net80211 needs to transmit a frame.
 */
int
zyd_send(ieee80211com_t *ic, mblk_t *mp, uint8_t type)
{
	struct zyd_softc *sc = ZYD_IC_TO_SOFTC(ic);
	int err;

	ASSERT(mp->b_next == NULL);
	
	ZYD_DEBUG("begin");

	err = zyd_hw_send_frame(sc, mp);
	
	return ((err == ZYD_SUCCESS) ? DDI_SUCCESS : DDI_FAILURE);
}

/*
 * xxx_newstate callback for net80211.
 *
 * Called by net80211 whenever the ieee80211 state changes.
 */
int
zyd_newstate(struct ieee80211com *ic, enum ieee80211_state nstate, int arg)
{
	struct zyd_softc *sc = ZYD_IC_TO_SOFTC(ic);
	enum ieee80211_state ostate;
	int res;
	
	ostate = ic->ic_state;
	ZYD_DEBUG("%s -> %s", ieee80211_state_name[ostate],
	                      ieee80211_state_name[nstate]);

	zyd_set_channel(sc, ic->ic_curchan);
	
	ZYD_DEBUG("call sc->newstate");
	res = sc->newstate(ic, nstate, arg);
	ZYD_DEBUG("done");

	if (nstate == IEEE80211_S_SCAN && ostate != nstate) {
		ZYD_DEBUG("set zyd_next_scan as timeout");
		sc->timeout_id = timeout(zyd_next_scan, sc,
	                         drv_usectohz(ZYD_DWELL_TIME));
	}

	return res;
}


/*
 * Loadable module configuration entry points
 */

/*
 * _init module entry point.
 *
 * Called when the module is being loaded into memory.
 */
int
_init(void)
{
	int err;
	
	ZYD_DEBUG("begin");
	
	err = ddi_soft_state_init(&zyd_ssp,
	         sizeof (struct zyd_softc), 1);
	
	if (err != DDI_SUCCESS)
		return (err);
	
	mac_init_ops(&zyd_devops, ZYD_DRV_NAME);
	err = mod_install(&zyd_ml);
	
	if (err != DDI_SUCCESS) {
		mac_fini_ops(&zyd_devops);
		ddi_soft_state_fini(&zyd_ssp);
	}

	return (err);
}

/*
 * _ifo module entry point.
 *
 * Called to obtain information about the module.
 */
int
_info(struct modinfo *modinfop)
{
	ZYD_DEBUG("begin");
	
	return (mod_info(&zyd_ml, modinfop));
}

/*
 * _fini module entry point.
 *
 * Called when the module is being unloaded.
 */
int
_fini(void)
{
	int err;
	
	ZYD_DEBUG("begin");
	
	err = mod_remove(&zyd_ml);
	if (err == DDI_SUCCESS) {
		mac_fini_ops(&zyd_devops);
		ddi_soft_state_fini(&zyd_ssp);
	}
	
	return (err);
}
