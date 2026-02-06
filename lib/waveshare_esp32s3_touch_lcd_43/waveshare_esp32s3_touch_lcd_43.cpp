// 2025-02-06 21:30:00 v1.2.0 - I2C HAL abstraction + lazy touch init
// Waveshare ESP32-S3-Touch-LCD-4.3 display device implementation.

#include "roo_display/hal/config.h"

#include "waveshare_esp32s3_touch_lcd_43.h"

#include "roo_logging.h"

namespace roo_display {
namespace products {
namespace waveshare {

namespace {

// CH422G I/O Expander I2C addresses.
constexpr uint8_t kCh422gAddrWrSet = 0x24;  // Write set register address.
constexpr uint8_t kCh422gAddrWrIo = 0x38;   // Write I/O register address.

// CH422G register addresses (for reference; not directly used with HAL).
constexpr uint8_t kCh422gRegOut = 0x38;  // Output register.
constexpr uint8_t kCh422gRegIn = 0x26;   // Input register.

// CH422G pin assignments for I/O expander outputs.
constexpr uint8_t kExioTpRst = 1;   // GT911 reset pin.
constexpr uint8_t kExioLcdBl = 2;   // LCD backlight control.
constexpr uint8_t kExioLcdRst = 3;  // LCD reset pin.

// I2C bus configuration.
constexpr int kI2cSda = 8;
constexpr int kI2cScl = 9;
constexpr uint32_t kI2cFreq = 400000;

// GT911 touch controller configuration.
constexpr int8_t kGt911IntPin = 4;
constexpr uint8_t kGt911I2cAddr = 0x5D;
constexpr int kGt911AsyncInitDelayMs = 300;

// Display timing configuration for 800x480 RGB565 parallel interface.
constexpr esp32s3_dma::Config kWaveshareConfig = {
    .width = 800,
    .height = 480,
    .de = 5,
    .hsync = 46,
    .vsync = 3,
    .pclk = 7,
    .hsync_pulse_width = 4,
    .hsync_back_porch = 8,
    .hsync_front_porch = 8,
    .hsync_polarity = 0,
    .vsync_pulse_width = 4,
    .vsync_back_porch = 16,
    .vsync_front_porch = 16,
    .vsync_polarity = 0,
    .pclk_active_neg = 1,
    .prefer_speed = 16000000,
    .r0 = 1,
    .r1 = 2,
    .r2 = 42,
    .r3 = 41,
    .r4 = 40,
    .g0 = 39,
    .g1 = 0,
    .g2 = 45,
    .g3 = 48,
    .g4 = 47,
    .g5 = 21,
    .b0 = 14,
    .b1 = 38,
    .b2 = 18,
    .b3 = 17,
    .b4 = 10,
    .bswap = false};

}  // namespace

WaveshareEsp32s3TouchLcd43::WaveshareEsp32s3TouchLcd43(
    Orientation orientation, I2cMasterBusHandle i2c)
    : i2c_(i2c),
      ch422g_wr_set_(i2c_, kCh422gAddrWrSet),
      ch422g_wr_io_(i2c_, kCh422gAddrWrIo),
      display_(kWaveshareConfig),
      touch_(i2c_, -1, kGt911IntPin, kGt911AsyncInitDelayMs),
      exio_shadow_(0b00001110) {  // Initial state: LCD_RST=1, LCD_BL=1, TP_RST=1.
  display_.setOrientation(orientation);
}

bool WaveshareEsp32s3TouchLcd43::initTransport() {
  if (!psramFound()) {
    LOG(ERROR) << "PSRAM not found - required for frame buffer";
    return false;
  }

  // Initialize I2C bus using HAL.
  i2c_.init(kI2cSda, kI2cScl, kI2cFreq);

  // Initialize both CH422G I2C slave device addresses.
  ch422g_wr_set_.init();
  ch422g_wr_io_.init();

  delay(10);  // Allow bus to stabilize.

  // Perform GT911 reset via CH422G immediately (like original).
  // This ensures GT911 is ready when TouchGt911 auto-initializes.
  initTouchHardware();

  return true;
}

DisplayDevice& WaveshareEsp32s3TouchLcd43::display() {
  return display_;
}

TouchDevice* WaveshareEsp32s3TouchLcd43::touch() {
  return &touch_;
}

TouchCalibration WaveshareEsp32s3TouchLcd43::touch_calibration() {
  return TouchCalibration(0, 0, 800, 480);
}

void WaveshareEsp32s3TouchLcd43::setBacklight(bool on) {
  writeEXIO(kExioLcdBl, on);
}

void WaveshareEsp32s3TouchLcd43::initTouchHardware() {
  // The GT911 samples the INT pin state during reset to determine its I2C
  // address. INT must be LOW during reset to select address 0x5D.

  // Step 1: Pull INT pin LOW to select I2C address 0x5D.
  pinMode(kGt911IntPin, OUTPUT);
  digitalWrite(kGt911IntPin, LOW);
  delay(10);

  // Step 2: Assert GT911 reset via CH422G (active LOW).
  // CRITICAL: Reset shadow register to ensure only LCD pins are high.
  exio_shadow_ = (1 << kExioLcdRst) | (1 << kExioLcdBl);
  writeEXIO(kExioTpRst, false);
  delay(100);

  // Step 3: Release reset. GT911 boots asynchronously (~300ms).
  writeEXIO(kExioTpRst, true);

  // Step 4: Configure INT pin as input for interrupt handling.
  pinMode(kGt911IntPin, INPUT);

  // Note: The TouchGt911 driver includes a 300ms async initialization delay,
  // ensuring the GT911 has completed boot before communication begins.
}

void WaveshareEsp32s3TouchLcd43::writeEXIO(uint8_t pin, bool state) {
  // Update shadow register to track output state.
  if (state) {
    exio_shadow_ |= (1 << pin);
  } else {
    exio_shadow_ &= ~(1 << pin);
  }

  // CH422G protocol: Send write enable to address 0x24, then data to 0x38.
  // Step 1: Enable write mode at address 0x24.
  roo::byte enable_cmd[] = {roo::byte{0x01}};
  if (!ch422g_wr_set_.transmit(enable_cmd, 1)) {
    LOG(ERROR) << "CH422G write enable failed";
    return;
  }

  // Step 2: Write output data to address 0x38.
  roo::byte output_data[] = {roo::byte{exio_shadow_}};
  if (!ch422g_wr_io_.transmit(output_data, 1)) {
    LOG(ERROR) << "CH422G output write failed";
  }
}

}  // namespace waveshare
}  // namespace products
}  // namespace roo_display