#pragma ident	"%Z%%M%	%I%	%E% SMI"

/*
 * USB-safe synchronization.
 * Debugging routines.
 *
 * Kmutexes should never be held when making calls to USBA
 * or when seeping. Thus, we implement our own "mutex" on top
 * of kmutexes and kcondvars.
 *
 * Usage: Any (possibly concurrent) access to the soft state or device must
 * be serialized with a pair of zyd_serial_enter()/zyd_serial_exit().
 */

#include <sys/ksynch.h>

#include "zyd_impl.h"
#include "zyd_util.h"

/*
 * Initialize the serialization object.
 */
void
zyd_serial_init(struct zyd_softc *sc)
{
	mutex_init(&sc->serial.lock, NULL, MUTEX_DRIVER,
		sc->usb.cdata->dev_iblock_cookie);
	cv_init(&sc->serial.wait, NULL, CV_DRIVER, NULL);

	sc->serial.held = B_FALSE;
	sc->serial.initialized = B_TRUE;
}

/*
 * Wait for the serialization object.
 *
 * If wait_sig is ZYD_SER_SIG, the function may return 
 * a signal is received. In this case, the serialization object
 * is not acquired (but the mutex is) and the return value is ZYD_FAILURE.
 *
 * In any other case the function returns ZYD_SUCCESS and the
 * serialization object is acquired.
 */
zyd_res
zyd_serial_enter(struct zyd_softc *sc, boolean_t wait_sig)
{
	zyd_res res;

	mutex_enter(&sc->serial.lock);
	
	res = ZYD_SUCCESS;

	while (sc->serial.held != B_FALSE) {
		if (wait_sig == ZYD_SER_SIG) {
			res = cv_wait_sig(&sc->serial.wait, &sc->serial.lock);
		} else {
			cv_wait(&sc->serial.wait, &sc->serial.lock);
		}
	}
	sc->serial.held = B_TRUE;

	mutex_exit(&sc->serial.lock);

	return res;
}

/*
 * Release the serialization object.
 */
void
zyd_serial_exit(struct zyd_softc *sc)
{
	mutex_enter(&sc->serial.lock);
	sc->serial.held = B_FALSE;
	cv_broadcast(&sc->serial.wait);
	mutex_exit(&sc->serial.lock);
}

/*
 * Destroy the serialization object.
 */
void
zyd_serial_deinit(struct zyd_softc *sc)
{
	cv_destroy(&sc->serial.wait);
	mutex_destroy(&sc->serial.lock);

	sc->serial.initialized = B_FALSE;
}




/*
 * zyd_cb_lock: a special signal structure that is used for notification
 * that a callback function has been called. 
 */


/* Initializes the zyd_cb_lock structure. */ 
void
zyd_cb_lock_init(struct zyd_cb_lock *lock)
{
	ASSERT(lock != NULL);
	mutex_init(&lock->mutex, NULL, MUTEX_DRIVER, NULL);
	cv_init(&lock->cv, NULL, CV_DRIVER, NULL);
	lock->done = B_FALSE;	
}


/* Deinitalizes the zyd_cb_lock structure. */ 
void
zyd_cb_lock_destroy(struct zyd_cb_lock *lock)
{
	ASSERT(lock != NULL);
	mutex_destroy(&lock->mutex);
	cv_destroy(&lock->cv);
}


/*
 * Wait on lock until someone calls the "signal" function or the timeout 
 * expires. Note: timeout is in microseconds.
 */ 
zyd_res
zyd_cb_lock_wait(struct zyd_cb_lock *lock, clock_t timeout)
{
	zyd_res res;
	clock_t etime;
	int cv_res;

	ASSERT(lock != NULL);

	mutex_enter(&lock->mutex);

	if (timeout < 0) {
		/* no timeout - wait as long as needed */
		while(lock->done == B_FALSE)
			cv_wait(&lock->cv, &lock->mutex);
	} else {
		/* wait with timeout (given in usec) */
		etime = ddi_get_lbolt() + drv_usectohz(timeout);
		while (lock->done == B_FALSE) {
			cv_res = cv_timedwait_sig(&lock->cv,
					&lock->mutex, etime);
			if (cv_res <= 0) break;
		}
	}

	res = (lock->done == B_TRUE) ? ZYD_SUCCESS : ZYD_FAILURE;

	mutex_exit(&lock->mutex);

	return (res);
}


/* Signal that the job (eg. callback) is done and unblock anyone who waits. */
void
zyd_cb_lock_signal(struct zyd_cb_lock *lock)
{
	ASSERT(lock != NULL);

	mutex_enter(&lock->mutex);

	lock->done = B_TRUE;
	cv_broadcast(&lock->cv);

	mutex_exit(&lock->mutex);
}



#ifdef ZYD_DEBUG_ON

/*
 * Hexa-dump an array of bytes.
 */
void zyd_bhexdump(const void *p, size_t len)
{
	const unsigned char *cp;
	
	cp = p;

	while (len >= 8) {
		ZYD_DEBUG("0x%02x, 0x%02x, 0x%02x, 0x%02x, "
		"0x%02x, 0x%02x, 0x%02x, 0x%02x,",
		cp[0], cp[1], cp[2], cp[3],
		cp[4], cp[5], cp[6], cp[7]);
		cp += 8;
		len -= 8;
	}
	
	while (len > 0) {
		ZYD_DEBUG("0x%02x", *cp);
		++cp;
		--len;
	}
}

#endif
