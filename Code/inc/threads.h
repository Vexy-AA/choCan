

#include <stdint.h>
#include <atomic>
#include <stdlib.h>
#include <string.h>

#include "ch.hpp"
#include "hal.h"

#include "shell.h"
#include "chprintf.h"


#include "rt_test_root.h"
#include "oslib_test_root.h"
#include "usbcfg.h"
#include "ringBuffer.hpp"

namespace chibios_rt {
    template <int N>
    class CustomizedThread : public BaseThread {
    protected:
        THD_WORKING_AREA(wa, N);
    public:
        void sleep_ms(sysinterval_t interval){
        sleep(TIME_MS2I(interval));
        }
        ThreadReference start(tprio_t prio) override {
        void _thd_start(void *arg);
        return ThreadReference(chThdCreateStatic(wa, sizeof(wa), prio,
                                                _thd_start, this));
        }
    private:
    };
}

