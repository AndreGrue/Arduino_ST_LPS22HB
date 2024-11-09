/*****************************************************************************/
/**
 * @file       st_lps22hb.h
 * @brief      st lps22hb library for arduino
 * @author     André Grüttner
 * @copyright  Please see the accompanying LICENSE file.
 * @date       2024-11-03
 */
/*****************************************************************************/
#pragma once

/*****************************************************************************/
#include <Arduino.h>
#include <Wire.h>

#include <cstdint>

/*****************************************************************************/
namespace andrgrue::sensor {

/*****************************************************************************/

/**
 * @brief st lps22hb pressure sensor driver for Arduino Nano uController
 */
class st_lps22hb {
  // data types
public:
  /// data rate
  enum class Rate : uint8_t {
    RATE_ONE_SHOT = 0,
    RATE_1HZ,
    RATE_10HZ,
    RATE_25HZ,
    RATE_50HZ,
    RATE_75HZ,
  };

  /**
   * @brief sensor data
   */
  struct Data {
    float    value;
    uint64_t timestamp;
    uint8_t  flags;
  };

  // construction
public:
  /**
   * @brief constructor
   * @param wire I2C bus
   */
  st_lps22hb(TwoWire& wire);
  virtual ~st_lps22hb() = default;

  // operations
public:
  /**
   * @brief initialize sensor
   * @param rate data rate
   * @param irqPin interrupt pin name
   * @param cb interrupt callback
   */
  bool initialize(const Rate    rate   = Rate::RATE_ONE_SHOT,
                  const PinName irqPin = NC,
#ifdef __MBED__
                  mbed::Callback<void(void)> cb = nullptr
#endif
  );

  /**
   * @brief terminate sensor
   */
  void terminate();

  /**
   * @brief set data rate
   * @param rate data rate
   */
  void setRate(const Rate rate);

  void reset();
  bool connected();

  /**
   * @brief set interrupt pin
   * @oaram irqPin interrupt pin name
   */
  void setIrqPin(const PinName irqPin) { irqPin_ = irqPin; }

  /**
   * @brief enable interrupt generation
   * interrupt pin must be configured
   */
  void enableInterrupt();

  /**
   * @brief disable interrupt generation
   */
  void disableInterrupt();

#if __MBED__
  /**
   * @brief link interrupt callback
   * @param cb interrupt callback
   */
  void interruptCallback(mbed::Callback<void()> cb);
#endif

  /**
   * @brief check if new data is available
   *        if interrupt pin is configured, it checks the state of that pin
   *        if not, it checks the data ready bits in the status register
   * @return true if new data is available
   */
  bool dataAvailable();

  /**
   * @brief read temperature in [°Celsius]
   */
  float temperature();
  void  temperature(Data& data);

  /*+
   * @brief read pressure in [Pa]
   */
  float pressure();
  void  pressure(Data& data);

protected:
private:
  uint8_t read(const uint8_t reg);
  void    write(const uint8_t reg, const uint8_t data);
  uint8_t whoAmI();
  void    triggerOneShot();
  int16_t twosCompToInteger(const uint16_t two_compliment_val);
#if __MBED__
  void interruptHandler();
#endif

  // data
public:
protected:
private:
  TwoWire&      wire_;
  const uint8_t address_;
  Rate          rate_ {Rate::RATE_ONE_SHOT};
  PinName       irqPin_ {NC};
#ifdef __MBED__
  mbed::Callback<void(void)> cb_;
#endif
  volatile uint64_t timestamp_ns_ {0};
};

/*****************************************************************************/
}  // namespace andrgrue::sensor
