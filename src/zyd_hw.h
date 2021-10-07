#ifndef _SYS_ZYD_HW_H
#define	_SYS_ZYD_HW_H

#pragma ident	"%Z%%M%	%I%	%E% SMI"

#ifdef __cplusplus
extern "C" {
#endif

#include "zyd_impl.h"

void	zyd_hw_set_channel(struct zyd_softc *sc, uint8_t chan);
zyd_res zyd_hw_send_frame(struct zyd_softc *sc, mblk_t *mp);

zyd_res	zyd_hw_init(struct zyd_softc *sc);
void	zyd_hw_deinit(struct zyd_softc *sc);

zyd_res	zyd_hw_start(struct zyd_softc *sc);
void	zyd_hw_stop(struct zyd_softc *sc);

#ifdef __cplusplus
}
#endif

#endif /* _SYS_ZYD_HW_H */
