/*
 * Arduino SAMD51 Timer module
 * An interrupt timer based on the SAMD51 clock subsystem
 * from https://github.com/Dennis-van-Gils/SAMD51_InterruptTime
 * Dennis van Gils
 * MIT License
 */

#include <Arduino.h>

#include "TC_Timer.h"

TC_Timer TC;

#ifdef _SAMD51_

void TC_Timer::startTimer(unsigned long period, void (*f)()) {
  // Enable the TC bus clock, use clock generator 1
  GCLK->PCHCTRL[TC3_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val |
                                   (1 << GCLK_PCHCTRL_CHEN_Pos);
  while (GCLK->SYNCBUSY.reg > 0);

  TC3->COUNT16.CTRLA.bit.ENABLE = 0;
  
  // Use match mode so that the timer counter resets when the count matches the
  // compare register
  TC3->COUNT16.WAVE.bit.WAVEGEN = TC_WAVE_WAVEGEN_MFRQ;
  TC3_wait_for_sync();
  
   // Enable the compare interrupt
  TC3->COUNT16.INTENSET.reg = 0;
  TC3->COUNT16.INTENSET.bit.MC0 = 1;

  // Enable IRQ
  NVIC_EnableIRQ(TC3_IRQn);

  this->callback = f;

  setPeriod(period);
}

void TC_Timer::stopTimer() {
  TC3->COUNT16.CTRLA.bit.ENABLE = 0;
}

void TC_Timer::restartTimer(unsigned long period) {
  // Enable the TC bus clock, use clock generator 1
  GCLK->PCHCTRL[TC3_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val |
                                   (1 << GCLK_PCHCTRL_CHEN_Pos);
  while (GCLK->SYNCBUSY.reg > 0);

  TC3->COUNT16.CTRLA.bit.ENABLE = 0;
  
  // Use match mode so that the timer counter resets when the count matches the
  // compare register
  TC3->COUNT16.WAVE.bit.WAVEGEN = TC_WAVE_WAVEGEN_MFRQ;
  TC3_wait_for_sync();
  
   // Enable the compare interrupt
  TC3->COUNT16.INTENSET.reg = 0;
  TC3->COUNT16.INTENSET.bit.MC0 = 1;

  // Enable IRQ
  NVIC_EnableIRQ(TC3_IRQn);

  setPeriod(period);
}

int TC_Timer::setPeriod(unsigned long period) {
  int prescaler;
  uint32_t TC_CTRLA_PRESCALER_DIVN;

  TC3->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  TC3_wait_for_sync();
  TC3->COUNT16.CTRLA.reg &= ~TC_CTRLA_PRESCALER_DIV1024;
  TC3_wait_for_sync();
  TC3->COUNT16.CTRLA.reg &= ~TC_CTRLA_PRESCALER_DIV256;
  TC3_wait_for_sync();
  TC3->COUNT16.CTRLA.reg &= ~TC_CTRLA_PRESCALER_DIV64;
  TC3_wait_for_sync();
  TC3->COUNT16.CTRLA.reg &= ~TC_CTRLA_PRESCALER_DIV16;
  TC3_wait_for_sync();
  TC3->COUNT16.CTRLA.reg &= ~TC_CTRLA_PRESCALER_DIV4;
  TC3_wait_for_sync();
  TC3->COUNT16.CTRLA.reg &= ~TC_CTRLA_PRESCALER_DIV2;
  TC3_wait_for_sync();
  TC3->COUNT16.CTRLA.reg &= ~TC_CTRLA_PRESCALER_DIV1;
  TC3_wait_for_sync();

  if (period > 300000) {
    TC_CTRLA_PRESCALER_DIVN = TC_CTRLA_PRESCALER_DIV1024;
    prescaler = 1024;
  } else if (80000 < period && period <= 300000) {
    TC_CTRLA_PRESCALER_DIVN = TC_CTRLA_PRESCALER_DIV256;
    prescaler = 256;
  } else if (20000 < period && period <= 80000) {
    TC_CTRLA_PRESCALER_DIVN = TC_CTRLA_PRESCALER_DIV64;
    prescaler = 64;
  } else if (10000 < period && period <= 20000) {
    TC_CTRLA_PRESCALER_DIVN = TC_CTRLA_PRESCALER_DIV16;
    prescaler = 16;
  } else if (5000 < period && period <= 10000) {
    TC_CTRLA_PRESCALER_DIVN = TC_CTRLA_PRESCALER_DIV8;
    prescaler = 8;
  } else if (2500 < period && period <= 5000) {
    TC_CTRLA_PRESCALER_DIVN = TC_CTRLA_PRESCALER_DIV4;
    prescaler = 4;
  } else if (1000 < period && period <= 2500) {
    TC_CTRLA_PRESCALER_DIVN = TC_CTRLA_PRESCALER_DIV2;
    prescaler = 2;
  } else if (period <= 1000) {
    TC_CTRLA_PRESCALER_DIVN = TC_CTRLA_PRESCALER_DIV1;
    prescaler = 1;
  }
  TC3->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIVN;
  TC3_wait_for_sync();

  int compareValue = (int)(GCLK1_HZ / (prescaler/((float)period / 1000000))) - 1;

  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC3->COUNT16.COUNT.reg = map(TC3->COUNT16.COUNT.reg, 0,
                               TC3->COUNT16.CC[0].reg, 0, compareValue);
  TC3->COUNT16.CC[0].reg = compareValue;
  TC3_wait_for_sync();

  TC3->COUNT16.CTRLA.bit.ENABLE = 1;
  TC3_wait_for_sync();

  return 0;
}

void TC3_Handler() {
  // If this interrupt is due to the compare register matching the timer count
  if (TC3->COUNT16.INTFLAG.bit.MC0 == 1) {
    TC3->COUNT16.INTFLAG.bit.MC0 = 1;
    if (TC.callback != NULL) {
        (*(TC.callback))();
    }
  }
}

#endif

#ifdef _SAMD21_

/**
 * SAM21 version incorporates code from a University of Greifswald, Colloidal Plasma Group project:
 * https://github.com/ug-cp/fast_samd21_tc/blob/main/README.md
 */

static void fast_samd21_tc3_reset() {
  TC3->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
  while (TC3->COUNT16.CTRLA.bit.SWRST);
}

/*
  Starts the timer.
*/
void fast_samd21_tc3_start() {
  TC3->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
}

/*
  Pauses the timer.
*/
void fast_samd21_tc3_disable() {
  TC3->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
}

/*
  Stops the timer. You should start with fast_samd21_tc3_configure again.
*/
void fast_samd21_tc3_stop() {
  fast_samd21_tc3_disable();
  fast_samd21_tc3_reset();
}

static inline uint32_t
_fast_samd21_tc_calculate_compare_register(double us,
					   uint32_t prescaler) {
  return (uint32_t) round(us * (SystemCoreClock / prescaler) / 1e6) - 1;
}

uint8_t fast_samd21_tc_calculate_compare_register(double us,
						  uint16_t *prescaler,
						  uint16_t *compare_register) {
  // find prescaler and compare register value
  // try TC_CTRLA_PRESCALER_DIV1
  uint16_t _prescaler = 1;
  uint32_t _compare_register =
    _fast_samd21_tc_calculate_compare_register(us, _prescaler);
  if (_compare_register > UINT16_MAX) {
    // try TC_CTRLA_PRESCALER_DIV2
    _prescaler = 2;
    _compare_register =
      _fast_samd21_tc_calculate_compare_register(us, _prescaler);
    if (_compare_register > UINT16_MAX) {
      // try TC_CTRLA_PRESCALER_DIV4
      _prescaler = 4;
      _compare_register =
	_fast_samd21_tc_calculate_compare_register(us, _prescaler);
      if (_compare_register > UINT16_MAX) {
        // try TC_CTRLA_PRESCALER_DIV8
        _prescaler = 8;
        _compare_register =
	  _fast_samd21_tc_calculate_compare_register(us, _prescaler);
        if (_compare_register > UINT16_MAX) {
          // try TC_CTRLA_PRESCALER_DIV16
          _prescaler = 16;
          _compare_register =
	    _fast_samd21_tc_calculate_compare_register(us, _prescaler);
          if (_compare_register > UINT16_MAX) {
            // try TC_CTRLA_PRESCALER_DIV64
            _prescaler = 64;
            _compare_register =
	      _fast_samd21_tc_calculate_compare_register(us, _prescaler);
            if (_compare_register > UINT16_MAX) {
              // try TC_CTRLA_PRESCALER_DIV256
              _prescaler = 256;
              _compare_register =
		_fast_samd21_tc_calculate_compare_register(us, _prescaler);
              if (_compare_register > UINT16_MAX) {
                // try TC_CTRLA_PRESCALER_DIV1024
                _prescaler = 1024;
                _compare_register =
		  _fast_samd21_tc_calculate_compare_register(us, _prescaler);
		if (_compare_register > UINT16_MAX)
		  return 3;
              }
            }
          }
        }
      }
    }
  }
  *prescaler = _prescaler;
  *compare_register = (uint16_t) _compare_register;
  return 0;
}


void TC_Timer::startTimer(unsigned long period, void (*f)()) {

#ifdef notdef
  // Enable the TC bus clock, use clock generator 1
  GCLK->PCHCTRL[TC3_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val |
                                   (1 << GCLK_PCHCTRL_CHEN_Pos);
  while (GCLK->SYNCBUSY.reg > 0);

  TC3->COUNT16.CTRLA.bit.ENABLE = 0;
  
  // Use match mode so that the timer counter resets when the count matches the
  // compare register
  TC3->COUNT16.WAVE.bit.WAVEGEN = TC_WAVE_WAVEGEN_MFRQ;
  TC3_wait_for_sync();
  
   // Enable the compare interrupt
  TC3->COUNT16.INTENSET.reg = 0;
  TC3->COUNT16.INTENSET.bit.MC0 = 1;
#endif

  // Enable IRQ
  NVIC_EnableIRQ(TC3_IRQn);

  this->callback = f;

  setPeriod(period);
}

void TC_Timer::stopTimer() {
  TC3->COUNT16.CTRLA.bit.ENABLE = 0;
}

void TC_Timer::restartTimer(unsigned long period) {

#ifdef notdef
  // Enable the TC bus clock, use clock generator 1
  GCLK->PCHCTRL[TC3_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val |
                                   (1 << GCLK_PCHCTRL_CHEN_Pos);
  while (GCLK->SYNCBUSY.reg > 0);

  TC3->COUNT16.CTRLA.bit.ENABLE = 0;
  
  // Use match mode so that the timer counter resets when the count matches the
  // compare register
  TC3->COUNT16.WAVE.bit.WAVEGEN = TC_WAVE_WAVEGEN_MFRQ;
  TC3_wait_for_sync();
  
   // Enable the compare interrupt
  TC3->COUNT16.INTENSET.reg = 0;
  TC3->COUNT16.INTENSET.bit.MC0 = 1;

  // Enable IRQ
  NVIC_EnableIRQ(TC3_IRQn);
#endif

  setPeriod(period);
}

int TC_Timer::setPeriod(unsigned long us) {
   if (((uint32_t) us) == 0)
    return 1;
  if (us > 1398111)
    return 2;
  if (us < 0)
    return 4;
  // find prescaler and compare register value
  // try TC_CTRLA_PRESCALER_DIV1
  uint16_t prescaler;
  uint16_t compare_register;
  if (fast_samd21_tc_calculate_compare_register(us,
						&prescaler,
						&compare_register) == 3) {
    return 3;
  }

  // select clock generator TC3
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN |
                                  GCLK_CLKCTRL_GEN_GCLK0 |
                                  GCLK_CLKCTRL_ID(GCM_TCC2_TC3)) ;
  while (GCLK->STATUS.bit.SYNCBUSY);

  fast_samd21_tc3_reset();

  // set 16-bit mode and set waveform 'match frequency'
  TC3->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_MFRQ;

  // prescaler
  switch (prescaler) {
    case 1:
      TC3->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;
      break;
    case 2:
      TC3->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV2 | TC_CTRLA_ENABLE;
      break;
    case 4:
      TC3->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV4 | TC_CTRLA_ENABLE;
      break;
    case 8:
      TC3->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV8 | TC_CTRLA_ENABLE;
      break;
    case 16:
      TC3->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16 | TC_CTRLA_ENABLE;
      break;
    case 64:
      TC3->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV64 | TC_CTRLA_ENABLE;
      break;
    case 256:
      TC3->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV256 | TC_CTRLA_ENABLE;
      break;
    case 1024:
      TC3->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_ENABLE;
      break;
  }

  // set compare register value
  TC3->COUNT16.CC[0].reg = compare_register;

  while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);

  NVIC_DisableIRQ(TC3_IRQn);
  NVIC_ClearPendingIRQ(TC3_IRQn);
  NVIC_SetPriority(TC3_IRQn, 0);
  NVIC_EnableIRQ(TC3_IRQn);

  // enable interrupt
  TC3->COUNT16.INTENSET.bit.MC0 = 1;
  while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);

  return 0;
}

void TC3_Handler() {
  // If this interrupt is due to the compare register matching the timer count
  if (TC3->COUNT16.INTFLAG.bit.MC0 == 1) {
    TC3->COUNT16.INTFLAG.bit.MC0 = 1;
    if (TC.callback != NULL) {
        (*(TC.callback))();
    }
  }
}

#endif
