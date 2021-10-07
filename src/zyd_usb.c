#pragma ident	"%Z%%M%	%I%	%E% SMI"

/*
 * ZD1211 wLAN driver
 * USB communication
 *
 * Manage USB communication with the ZD-based device.
 */

#include <sys/byteorder.h>
#include <sys/stream.h>
#include <sys/strsun.h>

#define USBDRV_MAJOR_VER 2
#define USBDRV_MINOR_VER 0
#include <sys/usb/usba.h>

#include "zyd_fw.h"
#include "zyd_reg.h"
#include "zyd_impl.h"
#include "zyd_usb.h"
#include "zyd_util.h"

/* Location in the endpoint descriptor tree used by the device */
#define ZYD_USB_CONFIG_NUMBER  1
#define ZYD_USB_IFACE_INDEX    0
#define ZYD_USB_ALT_IF_INDEX   0

static zyd_res zyd_usb_data_in_start_request(struct zyd_usb *uc);


/*
 * Vendor-specific write to the default control pipe.
 */
static zyd_res
zyd_usb_ctrl_send(struct zyd_usb *uc, uint8_t request, uint16_t value,
  uint8_t *data, uint16_t len)
{
	int err;
	mblk_t *msg;
	usb_ctrl_setup_t setup;
	
	/* Always clean structures before use */
	bzero(&setup, sizeof (setup));
	setup.bmRequestType = USB_DEV_REQ_TYPE_VENDOR | USB_DEV_REQ_HOST_TO_DEV;
	setup.bRequest = request;
	setup.wValue = value;
	setup.wIndex = 0;
	setup.wLength = len;
	setup.attrs = USB_ATTRS_NONE;
	
	if ((msg = allocb(len, BPRI_HI)) == NULL)
		return (ZYD_FAILURE);
	
	bcopy (data, msg->b_wptr, len);
	msg->b_wptr += len;

	err = usb_pipe_ctrl_xfer_wait(uc->cdata->dev_default_ph,
	                              &setup, &msg, NULL, NULL, 0);
	freemsg (msg);
	msg = NULL;
	
	if (err != USB_SUCCESS)
	{
		ZYD_WARN("USB transfer failed, error = %d", err);
		return (ZYD_FAILURE);
	}
	
	return (ZYD_SUCCESS);
}

/*
 * Vendor-specific read from the default control pipe.
 */
static zyd_res
zyd_usb_ctrl_recv(struct zyd_usb *uc, uint8_t request, uint16_t value,
  uint8_t *data, uint16_t len)
{
	int err;
	mblk_t *msg, *tmp_msg;
	usb_ctrl_setup_t setup;
	size_t msg_len;
	
	bzero(&setup, sizeof (setup));
	setup.bmRequestType = USB_DEV_REQ_TYPE_VENDOR | USB_DEV_REQ_DEV_TO_HOST;
	setup.bRequest = request;
	setup.wValue = value;
	setup.wIndex = 0;
	setup.wLength = len;
	setup.attrs = USB_ATTRS_NONE;
	
	/* Pointer msg must be either set to NULL or point to a valid mblk! */
	msg = NULL;
	err = usb_pipe_ctrl_xfer_wait(uc->cdata->dev_default_ph,
	                              &setup, &msg, NULL, NULL, 0);
	
	if (err != USB_SUCCESS || data == NULL)
	{
		ZYD_WARN("USB transfer failed, error = %d", err);
		return (ZYD_FAILURE);
	}

	msg_len = msgsize(msg);

	if (msg_len != len)
	{
		ZYD_WARN("USB transfer failed, received %d bytes, %d expected",
                         (int)msg_len, len);
		return (ZYD_FAILURE);
	}

	if(msg->b_cont != NULL) {
		/* Fragmented message, concatenate */
		tmp_msg = msgpullup(msg, -1);
		freemsg(msg);
		msg = tmp_msg;
		tmp_msg = NULL;
	}
	
	/* 
	 * Now we can be sure the message is in a single block
	 * so we can copy it.
	 */
	bcopy (msg->b_rptr, data, len);

	freemsg(msg);
	msg = NULL;

	return (ZYD_SUCCESS);
}

/*
 * Load firmware into the chip.
 */
static zyd_res
zyd_usb_loadfirmware(struct zyd_usb *uc, uint8_t *fw, size_t size)
{
	uint16_t addr;
	uint8_t stat;
	
	ZYD_DEBUG("firmware size = %lu\n", size);
	
	addr = ZYD_FIRMWARE_START_ADDR;
	while (size > 0) {
		const int mlen = min(size, 4096);
	
		if (zyd_usb_ctrl_send(uc, ZYD_DOWNLOADREQ, addr, fw, mlen)
		    != USB_SUCCESS)
		  return (ZYD_FAILURE);
		
		addr += mlen / 2;
		fw   += mlen;
		size -= mlen;
	}
	
	/* check whether the upload succeeded */
	if (zyd_usb_ctrl_recv(uc, ZYD_DOWNLOADSTS, 0, &stat, sizeof (stat))
	    != ZYD_SUCCESS)
	  return (ZYD_FAILURE);
	
	return (stat & 0x80) ? ZYD_FAILURE : ZYD_SUCCESS;
}

/*
 * Return a specific alt_if from the device descriptor tree.
 */
static usb_alt_if_data_t
*usb_lookup_alt_if(usb_client_dev_data_t *cdd, uint_t config,
uint_t interface, uint_t alt)
{
	usb_cfg_data_t *dcfg;
	usb_if_data_t *cfgif;
	usb_alt_if_data_t *ifalt;

	/*
	 * Assume everything is in the tree for now,
	 * (USB_PARSE_LVL_ALL)
	 * so we can directly index the array.
	 */

	/* Descend to configuration, configs are 1-based */
	if(config < 1 || config > cdd->dev_n_cfg)
		return (NULL);
	dcfg = &cdd->dev_cfg[config-1];

	/* Descend to interface */
	if(interface < 0 || interface > dcfg->cfg_n_if-1)
		return (NULL);
	cfgif = &dcfg->cfg_if[interface];

	/* Descend to alt */
	if(alt < 0 || alt > cfgif->if_n_alt-1)
		return (NULL);
	ifalt = &cfgif->if_alt[alt];

	return (ifalt);
}

/*
 * Print all endpoints of an alt_if.
 */
static void
usb_list_all_endpoints(usb_alt_if_data_t *ifalt)
{
	usb_ep_data_t *ep_data;
	usb_ep_descr_t *ep_descr;
	int i;

	for (i=0; i<ifalt->altif_n_ep; i++) {
		ep_data = &ifalt->altif_ep[i];
		ep_descr = &ep_data->ep_descr;
		ZYD_DEBUG("ep address is %u", ep_descr->bEndpointAddress);
	}
}


/*
 * For the given alt_if, find an endpoint with the given
 * address and direction.
 *
 *	ep_direction	USB_EP_DIR_IN or USB_EP_DIR_OUT
 */
static usb_ep_data_t
*usb_find_endpoint(
	usb_alt_if_data_t *alt_if,
	uint_t ep_address,
	uint_t ep_direction)
{
	usb_ep_data_t *ep_data;
	usb_ep_descr_t *ep_descr;
	uint_t ep_addr, ep_dir;
	int i;
	
	for (i=0; i<alt_if->altif_n_ep; i++) {
		ep_data = &alt_if->altif_ep[i];
		ep_descr = &ep_data->ep_descr;
		ep_addr = ep_descr->bEndpointAddress & USB_EP_NUM_MASK;
		ep_dir = ep_descr->bEndpointAddress & USB_EP_DIR_MASK;
		
		if (ep_addr == ep_address && ep_dir == ep_direction) {
			return (ep_data);
		}
	}
	
	ZYD_WARN("not found ep with addr %u, dir %u!", ep_address,
ep_direction);
	return (NULL);
}

enum zyd_usb_use_attr {
	ZYD_USB_USE_ATTR = 1,
	ZYD_USB_NO_ATTR = 0
};

/*
 * Open a pipe to a given endpoint address/direction in the given
 * alt_if. Furthemore, if use_attr == ZYD_USB_USE_ATTR,
 * check whether the endpoint's transfer type is attr.
 */
static zyd_res
zyd_usb_open_pipe(
	struct zyd_usb *uc,
	usb_alt_if_data_t *alt_if,
	uint_t ep_address,
	uint_t ep_direction,
	uint_t attr,
	enum zyd_usb_use_attr use_attr,
	usb_pipe_handle_t *pipe,
	usb_ep_data_t *endpoint)
{
	usb_pipe_policy_t pipe_policy;
	
	*endpoint = *usb_find_endpoint(alt_if, ep_address, ep_direction);

	if ((use_attr == ZYD_USB_USE_ATTR) &&
		(endpoint->ep_descr.bmAttributes & USB_EP_ATTR_MASK) != attr) {

		ZYD_WARN("endpoint %u/%s is not of type %s", ep_address,
			(ep_direction == USB_EP_DIR_IN) ? "IN" : "OUT",
			(attr == USB_EP_ATTR_BULK) ? "bulk" : "intr"
		);
		return (ZYD_FAILURE);
	}

	bzero(&pipe_policy, sizeof (usb_pipe_policy_t));
	pipe_policy.pp_max_async_reqs = 2; /* No idea... */

	if (usb_pipe_open(uc->dip, &endpoint->ep_descr,
	    &pipe_policy, USB_FLAGS_SLEEP, pipe) != USB_SUCCESS) {
		ZYD_WARN("failed to open pipe %u", ep_address);
		return (ZYD_FAILURE);
	}
	
	return (ZYD_SUCCESS);
}

/*
 * Open communication pipes.
 *
 * The following pipes are used by the ZD1211:
 *
 *	1/OUT BULK
 *	2/IN  BULK
 *	3/IN  INTR
 *	4/OUT BULK or INTR
 */
static zyd_res
zyd_usb_open_pipes(struct zyd_usb *uc)
{
	usb_alt_if_data_t *alt_if;

	ZYD_DEBUG("begin zyd_usb_open_pipes");

	alt_if = usb_lookup_alt_if(uc->cdata, ZYD_USB_CONFIG_NUMBER,
		ZYD_USB_IFACE_INDEX, ZYD_USB_ALT_IF_INDEX);

	if (alt_if == NULL) {
		ZYD_WARN("alt_if not found");
		return (ZYD_FAILURE);
	}

	if(0) usb_list_all_endpoints(alt_if);

	if (zyd_usb_open_pipe(uc, alt_if, 1, USB_EP_DIR_OUT, USB_EP_ATTR_BULK,
	    ZYD_USB_USE_ATTR, &uc->pipe_data_out, &uc->ep_data_out) !=
            ZYD_SUCCESS) {
		return (ZYD_FAILURE);
	}

	if (zyd_usb_open_pipe(uc, alt_if, 2, USB_EP_DIR_IN, USB_EP_ATTR_BULK,
	    ZYD_USB_USE_ATTR, &uc->pipe_data_in, &uc->ep_data_in) !=
	    ZYD_SUCCESS) {
		return (ZYD_FAILURE);
	}

	if (zyd_usb_open_pipe(uc, alt_if, 3, USB_EP_DIR_IN, USB_EP_ATTR_INTR,
		ZYD_USB_USE_ATTR, &uc->pipe_cmd_in, &uc->ep_cmd_in) !=
		ZYD_SUCCESS) {
		return (ZYD_FAILURE);
	}
	
	/*
	 * Pipe 4/OUT is either a bulk or interrupt pipe.
	 */
	if (zyd_usb_open_pipe(uc, alt_if, 4, USB_EP_DIR_OUT, 0/* ignored */,
	    ZYD_USB_NO_ATTR, &uc->pipe_cmd_out, &uc->ep_cmd_out) !=
	    ZYD_SUCCESS) {
		return (ZYD_FAILURE);
	}

	return (ZYD_SUCCESS);
}

/*
 * Close communication pipes.
 */
static void
zyd_usb_close_pipes(struct zyd_usb *uc)
{
	ZYD_DEBUG("closing pipes");
	if (uc->pipe_data_out != NULL) {
		usb_pipe_close(uc->dip, uc->pipe_data_out, USB_FLAGS_SLEEP,
				NULL, NULL);
		uc->pipe_data_out = NULL;
	}

	if (uc->pipe_data_in != NULL) {
		usb_pipe_close(uc->dip, uc->pipe_data_in, USB_FLAGS_SLEEP,
				NULL, NULL);
		uc->pipe_data_in = NULL;
	}

	if (uc->pipe_cmd_in != NULL) {
		usb_pipe_close(uc->dip, uc->pipe_cmd_in, USB_FLAGS_SLEEP,
				NULL, NULL);
		uc->pipe_cmd_in = NULL;
	}

	if (uc->pipe_cmd_out != NULL) {
		usb_pipe_close(uc->dip, uc->pipe_cmd_out, USB_FLAGS_SLEEP,
				NULL, NULL);
		uc->pipe_cmd_out = NULL;
	}
}

/*
 * Send a sequence of bytes to a bulk pipe.
 *
 *	uc	pointer to usb module state
 *	data	pointer to a buffer of bytes
 *	len	size of the buffer (bytes)
 */
static int
zyd_usb_bulk_pipe_send(
	struct zyd_usb *uc,
	usb_pipe_handle_t pipe,
	const void *data,
	size_t len)
{
	usb_bulk_req_t *send_req;
	mblk_t *mblk;
	int res;

	send_req = usb_alloc_bulk_req(uc->dip, len, USB_FLAGS_SLEEP);

	send_req->bulk_len = len;
	send_req->bulk_attributes = USB_ATTRS_AUTOCLEARING;
	send_req->bulk_timeout = 5;

	mblk = send_req->bulk_data;
	bcopy(data, mblk->b_wptr, len);
	mblk->b_wptr += len;

	res = usb_pipe_bulk_xfer(pipe, send_req, USB_FLAGS_SLEEP);
	usb_free_bulk_req(send_req);
	send_req = NULL;
	
	if (res != USB_SUCCESS) {
		ZYD_WARN("Error %d writing data to bulk/out pipe", res);
		return (USB_FAILURE);
	}
	
	return (USB_SUCCESS);
}

/*
 * Called when the transfer from zyd_usb_intr_pipe_send() terminates
 * or an exception occurs on the pipe.
 */
static void
zyd_intr_pipe_cb(usb_pipe_handle_t pipe, struct usb_intr_req *req)
{
	struct zyd_cb_lock *lock;
	lock = (struct zyd_cb_lock*)req->intr_client_private;

	/* Just signal that something happened */
	zyd_cb_lock_signal(lock);
}

/*
 * Send a sequence of bytes to an interrupt pipe.
 *
 *	uc	pointer to usb module state
 *	data	pointer to a buffer of bytes
 *	len	size of the buffer (bytes)
 */
static int
zyd_usb_intr_pipe_send(
	struct zyd_usb *uc,
	usb_pipe_handle_t pipe,
	const void *data,
	size_t len)
{
	usb_intr_req_t *send_req;
	mblk_t *mblk;
	int res;
	struct zyd_cb_lock lock;

	send_req = usb_alloc_intr_req(uc->dip, len, USB_FLAGS_SLEEP);
	if (send_req == NULL) {
		ZYD_WARN("Failed to allocate USB request");
		return (USB_FAILURE);
	}

	send_req->intr_len = len;
	send_req->intr_client_private = (usb_opaque_t)&lock;
	send_req->intr_attributes = USB_ATTRS_AUTOCLEARING;
	send_req->intr_timeout = 5;
	send_req->intr_cb = zyd_intr_pipe_cb;
	send_req->intr_exc_cb = zyd_intr_pipe_cb;

	mblk = send_req->intr_data;
	bcopy(data, mblk->b_wptr, len);
	mblk->b_wptr += len;

	zyd_cb_lock_init(&lock);

	res = usb_pipe_intr_xfer(pipe, send_req, 0);

	if (res != USB_SUCCESS) {
		ZYD_WARN("Error %d writing data to intr/out pipe", res);
		usb_free_intr_req(send_req);
		zyd_cb_lock_destroy(&lock);
		return (ZYD_FAILURE);
	}
	
	if (zyd_cb_lock_wait(&lock, 1000000) != ZYD_SUCCESS) {

		ZYD_WARN("Timeout - pipe reset!");

		usb_pipe_reset(uc->dip, pipe, USB_FLAGS_SLEEP, NULL, 0);
		zyd_cb_lock_wait(&lock, -1);

		res = ZYD_FAILURE;

	} else {

		res = (send_req->intr_completion_reason == USB_CR_OK) ?
			ZYD_SUCCESS : ZYD_FAILURE;
	}

	usb_free_intr_req(send_req);
	send_req = NULL;

	zyd_cb_lock_destroy(&lock);
	
	return (res);
}

/*
 * Send a sequence of bytes to the cmd_out pipe. (in a single USB transfer)
 *
 *	uc	pointer to usb module state
 *	data	pointer to a buffer of bytes
 *	len	size of the buffer (bytes)
 */
static zyd_res
zyd_usb_cmd_pipe_send(
	struct zyd_usb *uc,
	const void *data,
	size_t len)
{
	zyd_res res;
	uint8_t type;

	/* Determine the type of cmd_out */
	type = uc->ep_cmd_out.ep_descr.bmAttributes & USB_EP_ATTR_MASK;

	if (type == USB_EP_ATTR_BULK)
		res = zyd_usb_bulk_pipe_send(uc, uc->pipe_cmd_out, data, len);
	else
		res = zyd_usb_intr_pipe_send(uc, uc->pipe_cmd_out, data, len);

	return (res);
}


/*
 * Format and send a command to the cmd_out pipe.
 *
 *	uc	pointer to usb module state
 *	code	ZD command code (16-bit)
 *	data	raw buffer containing command data
 *	len	size of the data buffer (bytes)
 */
zyd_res
zyd_usb_cmd_send(
	struct zyd_usb *uc,
	uint16_t code,
	const void *data,
	size_t len)
{
	zyd_res res;
	struct zyd_cmd cmd;

	cmd.cmd_code = LE_16(code);
	bcopy(data, cmd.data, len);

	res = zyd_usb_cmd_pipe_send(uc, &cmd, sizeof (uint16_t) + len);

	if (res != ZYD_SUCCESS) {
		ZYD_WARN("Error writing command");
		return (ZYD_FAILURE);
	}

	return (ZYD_SUCCESS);
}

/*
 * Issue an ioread request.
 *
 * Issues a ZD ioread command (with a vector of addresses passed in raw
 * form as in_data) and blocks until the response is received
 * and filled into the response buffer.
 *
 *	uc		pointer to usb module state
 *	in_data		pointer to request data
 *	in_len		request data size (bytes)
 *	out_data	pointer to response buffer
 *	out_len		response buffer size (bytes)
 */
zyd_res
zyd_usb_ioread_req(
	struct zyd_usb *uc,
	const void *in_data,
	size_t in_len,
	void *out_data,
	size_t out_len)
{
	zyd_res res;
	int cnt;
	
	/* Initialise io_read structure */
	uc->io_read.done = B_FALSE;
	uc->io_read.buffer = out_data;
	uc->io_read.buf_len = out_len;

	uc->io_read.pending = B_TRUE;

	res = zyd_usb_cmd_send(uc, ZYD_CMD_IORD, in_data, in_len);
	
	if (res != ZYD_SUCCESS) {
		ZYD_WARN("Error writing command");
		return (ZYD_FAILURE);
	}

	cnt = 0;
	while(uc->io_read.done != B_TRUE && cnt<500) {
		delay(drv_usectohz(10*1000));
		++cnt;
	}

	if (uc->io_read.done != B_TRUE) {
		ZYD_DEBUG("timeout, stop request");
		return (ZYD_FAILURE);
	}

	if (uc->io_read.exc != B_FALSE) {
		ZYD_WARN("xfer result: exception");
		return (ZYD_FAILURE);
	}

	return (ZYD_SUCCESS);
}


/*
 * Called when data arrives from the cmd_in pipe.
 */
static void
cmd_in_cb(usb_pipe_handle_t pipe, usb_intr_req_t *req)
{
	struct zyd_usb *uc;
	struct zyd_ioread *rdp;
	mblk_t *mblk, *tmp_blk;
	unsigned char *data;
	size_t len;
	uint16_t code;

	uc = (struct zyd_usb *)req->intr_client_private;
	if (uc == NULL) {
		ZYD_WARN("cannot set result: uc = NULL");
		usb_free_intr_req(req);
		req = NULL;
		return;
	}
	rdp = &uc->io_read;

	mblk = req->intr_data;
	
	if (mblk->b_cont != NULL) {
		/* Fragmented message, concatenate */
		tmp_blk = msgpullup(mblk, -1);
		data = tmp_blk->b_rptr;
		len = MBLKL(tmp_blk);
	} else {
		/* Non-fragmented message, use directly */
		tmp_blk = NULL;
		data = mblk->b_rptr;
		len = MBLKL(mblk);
	}

	code = LE_16( *(uint16_t *)data );
	if (code != ZYD_RESPONSE_IOREAD) {
		/* Other response types not handled yet */
		usb_free_intr_req(req);
		req = NULL;
		return;
	}

	if (rdp->pending != B_TRUE) {
		ZYD_WARN("no ioread pending");
		usb_free_intr_req(req);
		req = NULL;
		return;
	}

	rdp->pending = B_FALSE;
	
	/* Now move on to the data part */
	data += sizeof (uint16_t);
	len  -= sizeof(uint16_t);

	if (rdp->buf_len > len) {
		ZYD_WARN("too few bytes received");
	}
	
	bcopy(data, rdp->buffer, rdp->buf_len);

	if (tmp_blk != NULL) freemsg(mblk);

	rdp->exc = B_FALSE;
	rdp->done = B_TRUE;

	usb_free_intr_req(req);
	req = NULL;
}

/*
 * Called when an exception occurs on the cmd_in pipe.
 */
static void
cmd_in_exc_cb(usb_pipe_handle_t pipe, usb_intr_req_t *req)
{
	struct zyd_usb *uc;
	struct zyd_ioread *rdp;

	ZYD_DEBUG("cmd_in_exc_cb");

	uc = (struct zyd_usb *)req->intr_client_private;
	if (uc == NULL) {
		ZYD_WARN("cannot set result: uc = NULL");
		usb_free_intr_req(req);
		req = NULL;
		return;
	}
	rdp = &uc->io_read;
	
	if (rdp->pending == B_TRUE) {
		rdp->exc = B_FALSE;	
		rdp->done = B_TRUE;
	}

	usb_free_intr_req(req);
	req = NULL;
}

/*
 * Start interrupt polling on the cmd_in pipe.
 */
static zyd_res
zyd_usb_cmd_in_start_polling(struct zyd_usb *uc)
{
	usb_intr_req_t *intr_req;
	int res;
	
	if (uc->cmd_in_running == B_TRUE)
		return (ZYD_SUCCESS);
	
	intr_req = usb_alloc_intr_req(uc->dip, 0, USB_FLAGS_SLEEP);
	intr_req->intr_attributes = USB_ATTRS_SHORT_XFER_OK;
	intr_req->intr_len = uc->ep_cmd_in.ep_descr.wMaxPacketSize;
	intr_req->intr_cb = cmd_in_cb;
	intr_req->intr_exc_cb = cmd_in_exc_cb;
	intr_req->intr_client_private = (usb_opaque_t)uc;
	
	res = usb_pipe_intr_xfer(uc->pipe_cmd_in, intr_req, USB_FLAGS_NOSLEEP);
	
	if (res != USB_SUCCESS) {
		ZYD_WARN("Error starting polling on cmd_in pipe");
		usb_free_intr_req(intr_req);
		intr_req = NULL;

		return (ZYD_FAILURE);
	}
	
	uc->cmd_in_running = B_TRUE;
	
	return (ZYD_SUCCESS);
}

/*
 * Stop interrupt polling on the cmd_in pipe.
 */
static void
zyd_usb_cmd_in_stop_polling(struct zyd_usb *uc)
{
	ZYD_DEBUG("stop cmd_in polling");
	if (uc->cmd_in_running == B_TRUE) {
		usb_pipe_stop_intr_polling(uc->pipe_cmd_in, USB_FLAGS_SLEEP);
		uc->cmd_in_running = B_FALSE;
	}
}

/*
 * Called when data arrives on the data_in pipe.
 */
static void
data_in_cb(usb_pipe_handle_t pipe, usb_bulk_req_t *req)
{
	struct zyd_usb *uc;
	mblk_t *mblk, *tmp_blk;
	unsigned char *signature, *data;
	size_t len;

	uc = (struct zyd_usb *)req->bulk_client_private;
	if (uc == NULL) {
		ZYD_WARN("uc = NULL");
		usb_free_bulk_req(req);
		req = NULL;
		return;
	}

	mblk = req->bulk_data;

	/* Fragmented STREAMS message? */
	if (mblk->b_cont != NULL) {
		/* Fragmented, concatenate it into a single block */
		tmp_blk = msgpullup(mblk, -1);
		data = tmp_blk->b_rptr;
		len = MBLKL(tmp_blk);
	} else {
		/* Not fragmented, use directly */
		tmp_blk = NULL;
		data = mblk->b_rptr;
		len = MBLKL(mblk);
	}
	
	if (len < 2) {
		ZYD_WARN("received usb transfer too short");
		goto error;
	}

	/*
	 * If this is a composite packet, the last two bytes contain
	 * two special signature bytes.
	 */	
	signature = data + len - 2;

	if (signature[0] == ZYD_RX_COMPOSITE_SIG_1 &&
	    signature[1] == ZYD_RX_COMPOSITE_SIG_2) {
		/* multi-frame transfer */
		ZYD_DEBUG("got composite packet (dropped)");
	} else {
		/* single-frame transfer */
		zyd_receive(ZYD_USB_TO_SOFTC(uc), data, MBLKL(mblk));
	}

error:	
	if (tmp_blk != NULL) freemsg(tmp_blk);

	usb_free_bulk_req(req);
	req = NULL;
	
	if (zyd_usb_data_in_start_request(uc) != ZYD_SUCCESS) {
		ZYD_WARN("error restarting data_in transfer");
		uc->data_in.running = B_FALSE;
	}
}

/*
 * Called when an exception occurs on the data_in pipe.
 */
static void
data_in_exc_cb(usb_pipe_handle_t pipe, usb_bulk_req_t *req)
{
	struct zyd_usb *uc;

	ZYD_DEBUG("data_in_exc_cb");

	uc = (struct zyd_usb *)req->bulk_client_private;
	if (uc == NULL) {
		ZYD_WARN("uc = NULL");
		usb_free_bulk_req(req);
		req = NULL;
		return;
	}

	usb_free_bulk_req(req);
	req = NULL;
}

/*
 * Start a receive request on the data_in pipe.
 */
static zyd_res
zyd_usb_data_in_start_request(struct zyd_usb *uc)
{
	usb_bulk_req_t *req;
	int res;
	size_t len;

	usb_pipe_get_max_bulk_transfer_size(uc->dip, &len);

	req = usb_alloc_bulk_req(uc->dip, len, USB_FLAGS_SLEEP);
	req->bulk_len = len;
	req->bulk_timeout = 0;
	req->bulk_client_private = (usb_opaque_t)uc;
	req->bulk_attributes = USB_ATTRS_SHORT_XFER_OK | USB_ATTRS_AUTOCLEARING;
	req->bulk_cb = data_in_cb;
	req->bulk_exc_cb = data_in_exc_cb;
	
	res = usb_pipe_bulk_xfer(uc->pipe_data_in, req, USB_FLAGS_NOSLEEP);
	if (res != USB_SUCCESS) {
		ZYD_WARN("error starting receive request on data_in pipe");
		return (ZYD_FAILURE);
	}
	return (ZYD_SUCCESS);
}


/*
 * Start receiving packets on the data_in pipe.
 */
zyd_res
zyd_usb_data_in_enable(struct zyd_usb *uc)
{
	uc->data_in.running = B_TRUE;
	if (zyd_usb_data_in_start_request(uc) != ZYD_SUCCESS) {
		uc->data_in.running = B_FALSE;
		return (ZYD_FAILURE);
	}
	return (ZYD_SUCCESS);
}

/*
 * Stop receiving packets on the data_in pipe.
 */
void
zyd_usb_data_in_disable(struct zyd_usb *uc)
{
	uc->data_in.running = B_FALSE;
	usb_pipe_reset(uc->dip, uc->pipe_data_in, USB_FLAGS_SLEEP,
			NULL, NULL);
}

/*
 * Send a packet to data_out.
 *
 * A packet consists of a zyd_tx_header + the IEEE802.11 frame.
 */
zyd_res
zyd_usb_send_packet(struct zyd_usb *uc, const void *data, size_t len)
{
	/* Simple synchronous send a.t.m. */
	return zyd_usb_bulk_pipe_send(uc, uc->pipe_data_out, data, len);
}

/*
 * Initialize USB device communication and USB module state.
 *
 *	uc	pointer to usb module state
 *	dip	pointer to device info structure
 */
zyd_res
zyd_usb_init(struct zyd_usb *uc, dev_info_t *dip)
{
	int ures, zres;
	
	uc->dip = dip;

	ures = usb_client_attach(uc->dip, USBDRV_VERSION, 0);
	if (ures != USB_SUCCESS)
	{
		ZYD_WARN("usb_client_attach failed, error code: %d", ures);
		return (ZYD_FAILURE);
	}
	
	/* 
	 * LVL_ALL is needed for later endpoint scanning,
	 * and the tree must not be freed before that.
	 */
	ures = usb_get_dev_data(uc->dip, &uc->cdata, USB_PARSE_LVL_ALL, 0);
	if (ures != USB_SUCCESS)
	{
		ZYD_WARN("usb_get_dev_data failed, error code: %d", ures);
		return (ZYD_FAILURE);
	}
	
	zres = zyd_usb_loadfirmware (uc, zd1211_firmware,
	                             zd1211_firmware_size);
	if (zres != ZYD_SUCCESS)
	{
		ZYD_WARN("firmware load failed");
		return (ZYD_FAILURE);
	}
	
	/* Set configuration 1 - required for later communication */
	ures = usb_set_cfg(uc->dip, 0, USB_FLAGS_SLEEP, NULL, NULL);
	if (ures != USB_SUCCESS) {
		ZYD_WARN("failed to set configuration 1, error %d", ures);
		return (ZYD_FAILURE);
	}
	
	if (zyd_usb_open_pipes(uc) != ZYD_SUCCESS) {
		ZYD_WARN("failed to open pipes");
		return (ZYD_FAILURE);
	}

	/* Now we can perhaps free the description tree */
	/* TODO: are endpoint descriptors safely(deeply) copied? */
	usb_free_descr_tree(uc->dip, uc->cdata);

	if (zyd_usb_cmd_in_start_polling(uc) != ZYD_SUCCESS) {
		return (ZYD_FAILURE);
	}
	
	return (ZYD_SUCCESS);
}

/*
 * Deinitialize USB device communication.
 */
void
zyd_usb_deinit(struct zyd_usb *uc)
{
	zyd_usb_cmd_in_stop_polling(uc);
	zyd_usb_close_pipes(uc);
	usb_client_detach(uc->dip, uc->cdata);
}
