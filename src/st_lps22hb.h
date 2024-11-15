/*****************************************************************************/
/**
 * @file       st_lps22hb.h
 * @brief      st lps22hb library for arduino
 * @author     André Grüttner
 * @copyright  Please see the accompanying LICENSE.txt file.
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

// I2C Device Addresses

/// I2C Device Address of barometer - Nano 33 BLE SENSE
#define LPS22HB_DEVICE_ADDRESS_NANO33BLE 0x5C
/// I2C Device Address of barometer - Default
#define LPS22HB_DEVICE_ADDRESS_DEFAULT 0x5D

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

  /// low pass filter
  enum class LowPassFilter : uint8_t {
    LPF_2 = 0,
    LPF_9,
    LPF_20,
  };

  /**
   * @brief sensor data
   */
  struct Data {
    float    value;
    float    variance;
    uint64_t timestamp;
    uint8_t  flags;
  };

  // construction
public:
  /**
   * @brief constructor
   * @param wire I2C bus
   * @param address I2C device address of sensor
   * @param pressure_variance variance of the pressure sensor in [Pa^2]
   * @param temperature_variance variance of the temperature sensor in [°C^2]
   */
  st_lps22hb(TwoWire&      wire,
             const uint8_t address           = LPS22HB_DEVICE_ADDRESS_NANO33BLE,
             const float   pressure_variance = 25.0f,
             const float   temperature_variance = 0.5625f);
  virtual ~st_lps22hb() = default;

  // operations
public:
  /**
   * @brief initialize sensor
   * @param rate data rate
   * @param irqPin interrupt pin name
   * @param cb interrupt callback
   */
  bool initialize(const Rate          rate   = Rate::RATE_ONE_SHOT,
                  const LowPassFilter lpf    = LowPassFilter::LPF_2,
                  const PinName       irqPin = NC
#ifdef __MBED__
                  ,
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
   * @return true if successful
   */
  bool interruptCallback(mbed::Callback<void()> cb);
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
  LowPassFilter lpf_ {LowPassFilter::LPF_2};
  PinName       irqPin_ {NC};
#ifdef __MBED__
  mbed::Callback<void(void)> cb_;
#endif
  volatile uint64_t timestamp_ns_ {0};
  /**
   * @brief variance of the pressure sensor [Pa^2]
   * assuming half of the absolute accuraccy 0.1hPa as a conservative estimate
   * for the standard deviation resulting in a variance of 25.0Pa^2
   */
  const float pressure_variance_ {25.0f};
  /**
   * @brief variance of the temperature sensor [°C^2]
   * assuming half of the absolute accuraccy 0.75°C as a conservative estimate
   * for the standard deviation resulting in a variance of 0.5625°C^2
   */
  const float temperature_variance_ {0.5625f};
};

/*****************************************************************************/
}  // namespace andrgrue::sensor
