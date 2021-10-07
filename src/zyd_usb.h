#ifndef _SYS_ZYD_USB_H
#define	_SYS_ZYD_USB_H

#pragma ident	"%Z%%M%	%I%	%E% SMI"

#ifdef __cplusplus
extern "C" {
#endif

#include "zyd_impl.h"

zyd_res	zyd_usb_init(struct zyd_usb *uc, dev_info_t *dip);

void	zyd_usb_deinit(struct zyd_usb *uc);
		    
zyd_res	zyd_usb_cmd_send(struct zyd_usb *uc, uint16_t code,
			const void *data, size_t len);

zyd_res	zyd_usb_ioread_req(struct zyd_usb *uc, const void *in_data,
			size_t in_len, void *out_data, size_t out_len);

zyd_res	zyd_usb_data_in_enable(struct zyd_usb *uc);
void	zyd_usb_data_in_disable(struct zyd_usb *uc);

zyd_res	zyd_usb_send_packet(struct zyd_usb *uc, const void *data,
			size_t len);


#ifdef __cplusplus
}
#endif

#endif /* _SYS_ZYD_USB_H */
