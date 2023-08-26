
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
#define SLEEP           0
#define GOTO            1
#define STOP            2
#define BITCLEAR        3
#define BITSET          4
#define MESSAGE         5

typedef struct {
  uint8_t       action;
  union {
    msg_t       msg;
    uint32_t    value;
    ioline_t    line;
  };
} seqop_t;


// Flashing sequence for LED3.
static const seqop_t ledGreen1[] =
{
  {BITCLEAR,    LINE_LED_GREEN_1},
  {SLEEP,       0},
  {BITSET,      LINE_LED_GREEN_1},
  {SLEEP,       100},
  {BITCLEAR,    LINE_LED_GREEN_1},
  {SLEEP,       400},
  {GOTO,        0}
};

// Flashing sequence for LED4.
static const seqop_t ledRed1[] =
{
  {BITCLEAR,    LINE_LED_RED_1},
  {SLEEP,       50},
  {BITSET,      LINE_LED_RED_1},
  {SLEEP,       100},
  {BITCLEAR,    LINE_LED_RED_1},
  {SLEEP,       350},
  {GOTO,        0}
};

// Flashing sequence for LED5.
static const seqop_t ledBlue1[] =
{
  {BITCLEAR,    LINE_LED_BLUE_1},
  {SLEEP,       100},
  {BITSET,      LINE_LED_BLUE_1},
  {SLEEP,       100},
  {BITCLEAR,    LINE_LED_BLUE_1},
  {SLEEP,       300},
  {GOTO,        0}
};

// Flashing sequence for LED6.
static const seqop_t ledYellow1[] =
{
  {BITCLEAR,    LINE_LED_YELLOW_1},
  {SLEEP,       150},
  {BITSET,      LINE_LED_YELLOW_1},
  {SLEEP,       100},
  {BITCLEAR,    LINE_LED_YELLOW_1},
  {SLEEP,       250},
  {GOTO,        0}
};

static const seqop_t ledWhite1[] =
{
  {BITCLEAR,    LINE_LED_WHITE_1},
  {SLEEP,       200},
  {BITSET,      LINE_LED_WHITE_1},
  {SLEEP,       100},
  {BITCLEAR,    LINE_LED_WHITE_1},
  {SLEEP,       200},
  {GOTO,        0}
};

static const seqop_t ledGreen2[] =
{
  {BITCLEAR,    LINE_LED_GREEN_2},
  {SLEEP,       250},
  {BITSET,      LINE_LED_GREEN_2},
  {SLEEP,       100},
  {BITCLEAR,    LINE_LED_GREEN_2},
  {SLEEP,       150},
  {GOTO,        0}
};



class SequencerThread : public BaseStaticThread<256> {
private:
  const seqop_t *base, *curr;                   // Thread local variables.

protected:
  void main(void) override {
    setName("sequencer");
    systime_t tNow = 0;
    while (true) {
      switch(curr->action) {
      case SLEEP:
        //sleepUntil(TIME_MS2I(curr->value) + tNow);
        if (curr->value == 0) break;
        sleep(TIME_MS2I(curr->value));
        break;
      case GOTO:
        curr = &base[curr->value];
        if (curr->value == 0)
          tNow = System::getTime();
        continue;
      case STOP:
        return;
      case BITCLEAR:
        palClearLine(curr->line);
        break;
      case BITSET:
        palSetLine(curr->line);
        break;
      case MESSAGE:
      
        break;
      }
      curr++;
    }
  }

public:
  SequencerThread(const seqop_t *sequence) : BaseStaticThread<256>() {

    base = curr = sequence;
  }
};


/* 
static SequencerThread threadLightGreen1(ledGreen1);
static SequencerThread threadLightRed(ledRed1);
static SequencerThread threadLightBlue(ledBlue1);
static SequencerThread threadLightYellow(ledYellow1);
static SequencerThread threadLightWhite(ledWhite1);
static SequencerThread threadLightGreen2(ledGreen2);

 */


/* threadLightGreen1.start(NORMALPRIO + 20);
      threadLightRed.start(NORMALPRIO + 20);
      threadLightBlue.start(NORMALPRIO + 20);
      threadLightYellow.start(NORMALPRIO + 20);
      threadLightWhite.start(NORMALPRIO + 20);
      threadLightGreen2.start(NORMALPRIO + 20); */

}