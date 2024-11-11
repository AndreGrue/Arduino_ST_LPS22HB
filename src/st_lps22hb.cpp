/*****************************************************************************/
/**
 * @file       st_lps22hb.cpp
 * @brief      st lps22hb library for arduino
 * @author     André Grüttner
 * @copyright  Please see the accompanying LICENSE.txt file.
 * @date       2024-11-03
 */
/*****************************************************************************/
#include "st_lps22hb.h"

#include <Arduino.h>
#include <Wire.h>
#include <pinDefinitions.h>

#ifdef __MBED__
#  include "drivers/InterruptIn.h"
#  include "mbed_events.h"
#  include "mbed_shared_queues.h"

static events::EventQueue queue(10 * EVENTS_EVENT_SIZE);
#endif

/*****************************************************************************/
namespace andrgrue::sensor {

/*****************************************************************************/

//
#define LPS22HB_WHO_AM_I 0X0F   // Who am I
#define LPS22HB_RES_CONF 0X1A   // Normal (0) or Low current mode (1)
#define LPS22HB_CTRL_REG1 0X10  // Output rate and filter settings
#define LPS22HB_CTRL_REG2                                                      \
  0X11  // BOOT FIFO_EN STOP_ON_FTH IF_ADD_INC I2C_DIS SWRESET One_Shot
#define LPS22HB_CTRL_REG3 0X12    // interrupt
#define LPS22HB_STATUS_REG 0X27   // Temp or Press data available bits
#define LPS22HB_PRES_OUT_XL 0X28  // XLSB
#define LPS22HB_PRES_OUT_L 0X29   // LSB
#define LPS22HB_PRES_OUT_H 0X2A   // MSB
#define LPS22HB_TEMP_OUT_L 0X2B   // LSB
#define LPS22HB_TEMP_OUT_H 0X2C   // MSB
#define LPS22HB_WHO_AM_I_VALUE                                                 \
  0xB1  // Expected return value of WHO_AM_I register

// I2C Device Addresses -
#define LPS22HB_DEVICE_ADDRESS                                                 \
  0x5C                        //  Address of barometer (Nano 33 BLE SENSE)
#define DEFAULT_ADDRESS 0x5D  //  Sensor I2C Default Address

/*****************************************************************************/

st_lps22hb::st_lps22hb(TwoWire&    wire,
                       const float pressure_variance,
                       const float temperature_variance)
    : wire_(wire)
    , address_(LPS22HB_DEVICE_ADDRESS)
    , pressure_variance_(pressure_variance)
    , temperature_variance_(temperature_variance) {};

bool st_lps22hb::initialize(const Rate    rate,
                            const PinName irqPin
#ifdef __MBED__
                            ,
                            mbed::Callback<void(void)> cb
#endif
) {
  rate_   = rate;
  irqPin_ = irqPin;

#ifdef __MBED__
  if (interruptCallback(cb))
#endif
  {

    //  Check that chip boot up is complete
    while ((read(LPS22HB_CTRL_REG2) & 0x87) != 0) {
      yield();
    }
    reset();

    //  power-down mode, LPF off, and BDU on
    write(LPS22HB_CTRL_REG1, 0x02);

    // check connection
    if (connected()) {
      enableInterrupt();
      if (Rate::RATE_ONE_SHOT != rate_) {
        setRate(rate_);
      }

      return true;
    }
  }

  return false;
}

void st_lps22hb::terminate() {
  disableInterrupt();

  //  power-down mode, LPF off, and BDU on
  write(LPS22HB_CTRL_REG1, 0x02);
}

bool st_lps22hb::connected() { return (whoAmI() == LPS22HB_WHO_AM_I_VALUE); }

void st_lps22hb::reset() {
  //  0x04 = 0100b, bit 2 = SWRESET, set to reset chip
  write(LPS22HB_CTRL_REG2, 0x04);

  // software reset. Bit self clears when reset complete.
  while ((read(LPS22HB_CTRL_REG2) & 0x04) != 0) {
    yield();
  }
}

void st_lps22hb::setRate(const Rate rate) {
  rate_ = rate;
  //  Set ODR bits 4, 5 & 6 (_rate & 0x07) << 4 and BDU 0x02
  write(LPS22HB_CTRL_REG1, ((static_cast<uint8_t>(rate_) & 0x07) << 4) | 0x02);
}

void st_lps22hb::enableInterrupt() {
  // configured as an input
  if (NC != irqPin_) {
    pinMode(irqPin_, INPUT);
  }

  // enable data ready interrupt, active high, push pull
  constexpr uint8_t regval = 0x04;
  write(LPS22HB_CTRL_REG3, regval);
}

void st_lps22hb::disableInterrupt() {
  // disable data ready interrupt, active high, push pull
  constexpr uint8_t regval = 0x00;
  write(LPS22HB_CTRL_REG3, regval);
}

#ifdef __MBED__

bool st_lps22hb::interruptCallback(mbed::Callback<void(void)> cb) {
  if (NC != irqPin_ && nullptr != cb) {
    static mbed::InterruptIn irq(irqPin_, PullDown);
    static rtos::Thread      event_t(osPriorityHigh, 768, nullptr, "events");
    cb_ = cb;
    osStatus ret =
        event_t.start(callback(&queue, &events::EventQueue::dispatch_forever));
    if (osOK == ret) {
      irq.rise(mbed::callback(this, &st_lps22hb::interruptHandler));
      return true;
    }
    return false;
  }
  return true;
}

void st_lps22hb::interruptHandler() {
  if (cb_) {
    timestamp_ns_ = micros() * 1000ULL;
    queue.call(cb_);
  }
}
#endif

bool st_lps22hb::dataAvailable() {
  if (NC == irqPin_) {
    uint8_t status = read(LPS22HB_STATUS_REG);
    // check for pressure and temperature data ready
    return ((status & 0x03) == 0x03);
  } else {
    return (HIGH == digitalRead(irqPin_));
  }
}

void st_lps22hb::triggerOneShot() {
  write(LPS22HB_CTRL_REG2, 0x01);

  // wait for ONE_SHOT bit to clear
  while ((read(LPS22HB_CTRL_REG2) & 0x01) != 0) {
    yield();
  }
}

float st_lps22hb::pressure() {
  if (rate_ == Rate::RATE_ONE_SHOT) {
    triggerOneShot();
  }

  //  To guarantee the correct behavior of the BDU feature,
  //  PRESS_OUT_H (0x2A) must be the last address read.
  uint8_t  pressOutXL = read(LPS22HB_PRES_OUT_XL);
  uint8_t  pressOutL  = read(LPS22HB_PRES_OUT_L);
  uint8_t  pressOutH  = read(LPS22HB_PRES_OUT_H);
  uint32_t val    = (((uint32_t)pressOutH << 16) | ((uint32_t)pressOutL << 8)
                  | (uint32_t)pressOutXL);
  float    result = (double)val / 409600.0f;
  return result;
}

void st_lps22hb::pressure(Data& data) {
  data.value     = pressure();
  data.variance  = pressure_variance_;
  data.timestamp = timestamp_ns_;
  data.flags     = 0;
}

int16_t st_lps22hb::twosCompToInteger(const uint16_t two_compliment_val) {
  // [0x0000; 0x7FFF] corresponds to [0; 32,767]
  // [0x8000; 0xFFFF] corresponds to [-32,768; -1]
  // int16_t has the range [-32,768; 32,767]

  uint16_t sign_mask = 0x8000;

  if ((two_compliment_val & sign_mask) == 0) {
    // positive number - do nothing
    return two_compliment_val;
  } else {
    //  if negative invert all bits, add one, and add sign
    return -(~two_compliment_val + 1);
  }
}

float st_lps22hb::temperature() {
  if (rate_ == Rate::RATE_ONE_SHOT) {
    triggerOneShot();
  }

  uint8_t  tempOutL = read(LPS22HB_TEMP_OUT_L);
  uint8_t  tempOutH = read(LPS22HB_TEMP_OUT_H);
  uint16_t count    = (tempOutH << 8) | (tempOutL & 0xff);
  int16_t  val      = twosCompToInteger(count);
  float    result   = ((float)val) / 100.0f;  // In Celsius
  return result;
}

void st_lps22hb::temperature(Data& data) {
  data.value     = temperature();
  data.variance  = temperature_variance_;
  data.timestamp = timestamp_ns_;
  data.flags     = 0;
}

uint8_t st_lps22hb::read(const uint8_t reg) {
  wire_.beginTransmission(address_);
  wire_.write(reg);
  wire_.endTransmission();
  wire_.requestFrom(address_, 1);
  return wire_.read();
}

void st_lps22hb::write(const uint8_t reg, const uint8_t data) {
  wire_.beginTransmission(address_);
  wire_.write(reg);
  wire_.write(data);
  wire_.endTransmission();
}

uint8_t st_lps22hb::whoAmI() {
  wire_.beginTransmission(address_);
  wire_.write(LPS22HB_WHO_AM_I);
  wire_.endTransmission();
  wire_.requestFrom(address_, 1);
  return wire_.read();
}

/*****************************************************************************/
}  // namespace andrgrue::sensor
