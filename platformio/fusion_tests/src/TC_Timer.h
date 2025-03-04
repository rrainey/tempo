#ifndef TC_Timer_h
#define TC_Timer_h

#define GCLK1_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

static inline void TC3_wait_for_sync() {
#ifdef _SAMD51_
    while (TC3->COUNT16.SYNCBUSY.reg != 0) {
    }
#else
#ifdef _SAMD21_
    while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
#else
#error("Unknown chip")
#endif
#endif
}

class TC_Timer {
  public:
    TC_Timer() {
        callback = NULL;
    }
    void startTimer(unsigned long period, void (*f)());
    void stopTimer();
    void restartTimer(unsigned long period);
    int setPeriod(unsigned long period);

public:
    void (*callback)();
};

extern TC_Timer TC;

#endif