#include "watchdog.h"
#include "config.h"

#if WATCHDOG_ENABLE
#include <Watchdog_t4.h>

// WDT1 = WDOG1 on the i.MXRT1062. WDT_T4 wraps the register service sequence.
static WDT_T4<WDT1> _wdt;
#endif

void watchdog_begin() {
#if WATCHDOG_ENABLE
    WDT_timings_t cfg;
    cfg.trigger = WATCHDOG_TIMEOUT_S;   // (s) ISR warning point — keep == timeout
    cfg.timeout = WATCHDOG_TIMEOUT_S;   // (s) without feed() → hardware reset
    _wdt.begin(cfg);
#endif
}

void watchdog_feed() {
#if WATCHDOG_ENABLE
    _wdt.feed();
#endif
}
