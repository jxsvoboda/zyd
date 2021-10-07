#ifndef _SYS_ZYD_UTIL_H
#define	_SYS_ZYD_UTIL_H

#pragma ident	"%Z%%M%	%I%	%E% SMI"

#ifdef __cplusplus
extern "C" {
#endif

#include "zyd_impl.h"

#define ZYD_SER_SIG	B_TRUE
#define ZYD_NO_SIG	B_FALSE


/*
 * Zydas's own USB-safe synchronization primitive. There are many USB API
 * functions which forbids that caller holds a mutex. So we're avoiding that
 * by using out own primitive (it consist of )  
 */ 
void	zyd_serial_init(struct zyd_softc *sc);
zyd_res	zyd_serial_enter(struct zyd_softc *sc, boolean_t wait_sig);
void	zyd_serial_exit(struct zyd_softc *sc);
void	zyd_serial_deinit(struct zyd_softc *sc);



/*
 * Simple lock for callback-waiting. This lock should be used in situations when
 * one needs to wait for a callback function. It sipmply encapsulates one mutex
 * and one conditional variable. 
 */ 
struct zyd_cb_lock {
	boolean_t done;
	kmutex_t mutex;
	kcondvar_t cv;	
};

void	zyd_cb_lock_init(struct zyd_cb_lock *lock);
void	zyd_cb_lock_destroy(struct zyd_cb_lock *lock);
zyd_res	zyd_cb_lock_wait(struct zyd_cb_lock *lock, clock_t timeout);
void	zyd_cb_lock_signal(struct zyd_cb_lock *lock);



/* Debuging functions */
#ifdef ZYD_DEBUG_ON

void	zyd_bhexdump(const void *p, size_t len);

#endif

#ifdef __cplusplus
}
#endif

#endif /* _SYS_ZYD_H */
