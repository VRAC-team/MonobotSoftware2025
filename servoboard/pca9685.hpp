#ifndef VRAC_PCA9685_HPP
#define VRAC_PCA9685_HPP

#include <modm/architecture/interface/i2c_device.hpp>

struct pca9685
{
    const float OSCILLATOR_FREQ = 27000000;

	enum Register
	{
		REG_MODE1         = 0x00,
		REG_MODE2         = 0x01,
		REG_SUBADR1       = 0x02,
		REG_SUBADR2       = 0x03,
		REG_SUBADR3       = 0x04,
		REG_ALLCALLADR    = 0x05,
		REG_LED0_ON_L     = 0x06,
		REG_LED0_ON_H     = 0x07,
		REG_LED0_OFF_L    = 0x08,
		REG_LED0_OFF_H    = 0x09,
		REG_LED1_ON_L     = 0x0a,
		REG_LED1_ON_H     = 0x0b,
		REG_LED1_OFF_L    = 0x0c,
		REG_LED1_OFF_H    = 0x0d,
		REG_LED2_ON_L     = 0x0e,
		REG_LED2_ON_H     = 0x0f,
		REG_LED2_OFF_L    = 0x10,
		REG_LED2_OFF_H    = 0x11,
		REG_LED3_ON_L     = 0x12,
		REG_LED3_ON_H     = 0x13,
		REG_LED3_OFF_L    = 0x14,
		REG_LED3_OFF_H    = 0x15,
		REG_LED4_ON_L     = 0x16,
		REG_LED4_ON_H     = 0x17,
		REG_LED4_OFF_L    = 0x18,
		REG_LED4_OFF_H    = 0x19,
		REG_LED5_ON_L     = 0x1a,
		REG_LED5_ON_H     = 0x1b,
		REG_LED5_OFF_L    = 0x1c,
		REG_LED5_OFF_H    = 0x1d,
		REG_LED6_ON_L     = 0x1e,
		REG_LED6_ON_H     = 0x1f,
		REG_LED6_OFF_L    = 0x20,
		REG_LED6_OFF_H    = 0x21,
		REG_LED7_ON_L     = 0x22,
		REG_LED7_ON_H     = 0x23,
		REG_LED7_OFF_L    = 0x24,
		REG_LED7_OFF_H    = 0x25,
		REG_LED8_ON_L     = 0x26,
		REG_LED8_ON_H     = 0x27,
		REG_LED8_OFF_L    = 0x28,
		REG_LED8_OFF_H    = 0x29,
		REG_LED9_ON_L     = 0x2a,
		REG_LED9_ON_H     = 0x2b,
		REG_LED9_OFF_L    = 0x2c,
		REG_LED9_OFF_H    = 0x2d,
		REG_LED10_ON_L    = 0x2e,
		REG_LED10_ON_H    = 0x2f,
		REG_LED10_OFF_L   = 0x30,
		REG_LED10_OFF_H   = 0x31,
		REG_LED11_ON_L    = 0x32,
		REG_LED11_ON_H    = 0x33,
		REG_LED11_OFF_L   = 0x34,
		REG_LED11_OFF_H   = 0x35,
		REG_LED12_ON_L    = 0x36,
		REG_LED12_ON_H    = 0x37,
		REG_LED12_OFF_L   = 0x38,
		REG_LED12_OFF_H   = 0x39,
		REG_LED13_ON_L    = 0x3a,
		REG_LED13_ON_H    = 0x3b,
		REG_LED13_OFF_L   = 0x3c,
		REG_LED13_OFF_H   = 0x3d,
		REG_LED14_ON_L    = 0x3e,
		REG_LED14_ON_H    = 0x3f,
		REG_LED14_OFF_L   = 0x40,
		REG_LED14_OFF_H   = 0x41,
		REG_LED15_ON_L    = 0x42,
		REG_LED15_ON_H    = 0x43,
		REG_LED15_OFF_L   = 0x44,
		REG_LED15_OFF_H   = 0x45,
		/*
		 * 0x46 - 0xf9 reserved for future use
		 */
		REG_ALL_LED_ON_L  = 0xfa,
		REG_ALL_LED_ON_H  = 0xfb,
		REG_ALL_LED_OFF_L = 0xfc,
		REG_ALL_LED_OFF_H = 0xfd,
		REG_PRE_SCALE     = 0xfe,
		REG_TestMode      = 0xfe,
	};

	enum Mode1
	{
		MODE1_RESTART = 0x80,
		MODE1_EXTCLK  = 0x40,
		MODE1_AI      = 0x20,
		MODE1_SLEEP   = 0x10,
		MODE1_SUB1    = 0x08,
		MODE1_SUB2    = 0x04,
		MODE1_SUB3    = 0x02,
		MODE1_ALLCALL = 0x01,
	};

	enum Mode2
	{
		MODE2_INVRT   = 0x10,
		MODE2_OCH     = 0x08,
		MODE2_OUTDRV  = 0x04,
		MODE2_OUTNE1  = 0x02,
		MODE2_OUTNE0  = 0x01,
	};
};	// struct pca9685


template<typename I2cMaster>
class PCA9685 : public pca9685, public modm::I2cDevice<I2cMaster, 1, modm::I2cWriteReadTransaction>
{
public:
	PCA9685(uint8_t address = 0x40);

	bool initialize();
    uint8_t read_prescaler();
    void set_pwm_freq(float freq_hz);
	bool write_us(uint8_t channel, uint16_t value);

private:
    uint8_t m_prescale;
	float m_freq;
};


template<typename I2cMaster>
PCA9685<I2cMaster>::PCA9685(uint8_t address) :
	modm::I2cDevice<I2cMaster, 1, modm::I2cWriteReadTransaction>(address)
{}

template<typename I2cMaster>
bool PCA9685<I2cMaster>::initialize()
{
    uint8_t buffer[2];
	buffer[0] = REG_MODE1;
	buffer[1] = MODE1_RESTART;
	this->transaction.configureWrite(buffer, 2);
    if (!this->runTransaction()) {
        return false;
    }

    return true;
}

template<typename I2cMaster>
uint8_t PCA9685<I2cMaster>::read_prescaler() {
    uint8_t buffer[1];
	uint8_t data[1];
	buffer[0] = REG_PRE_SCALE;
	this->transaction.configureWriteRead(buffer, 1, data, 1);
    this->runTransaction();

	return data[0];
}

template<typename I2cMaster>
void PCA9685<I2cMaster>::set_pwm_freq(float freq_hz) {
    if (freq_hz < 1)
        freq_hz = 1;
    else if (freq_hz > 3500)
        freq_hz = 3500;
    
	m_freq = freq_hz;
	
    float prescaler = ((OSCILLATOR_FREQ / (freq_hz * 4096.0f)) + 0.5f) - 1.0f;
    if (prescaler < 3)
        prescaler = 3;
    else if (prescaler > 255)
        prescaler = 255;
	
	uint8_t prescale = (uint8_t)prescaler;

    uint8_t buffer[2];
	uint8_t data[1];
    buffer[0] = REG_MODE1;
    this->transaction.configureWriteRead(buffer, 1, data, 1);
    this->runTransaction();
    uint8_t old_mode = data[0];

    buffer[0] = REG_MODE1;
    buffer[1] = (old_mode & ~MODE1_RESTART) | MODE1_SLEEP;
    this->transaction.configureWrite(buffer, 2);
    this->runTransaction();

    buffer[0] = REG_PRE_SCALE;
    buffer[1] = prescale;
    this->transaction.configureWrite(buffer, 2);
    this->runTransaction();

    buffer[0] = REG_MODE1;
    buffer[1] = old_mode;
    this->transaction.configureWrite(buffer, 2);
    this->runTransaction();

    // It takes 500us max for the oscillator to be up and running once SLEEP bit has been set to logic 0.
    modm::delay_us(500);

    buffer[0] = REG_MODE1;
    buffer[1] = old_mode | MODE1_RESTART | MODE1_AI;
    this->transaction.configureWrite(buffer, 2);
    this->runTransaction();
}

template<typename I2cMaster>
bool PCA9685<I2cMaster>::write_us(uint8_t channel, uint16_t value)
{
	if (channel >= 16)
		return false;

	uint16_t prescale = read_prescaler();
	prescale += 1;

    float pulselength = 1000000; // 1,000,000 us per second
	pulselength *= prescale;
	pulselength /= OSCILLATOR_FREQ;

    if (pulselength == 0)
        return false;
    
    float pulse = (float)value/pulselength;

    uint16_t off_time = (uint16_t)pulse;

    uint8_t buffer[5];
	buffer[0] = REG_LED0_ON_L + 4 * channel;
	buffer[1] = 0;
	buffer[2] = 0;
	buffer[3] = off_time;
	buffer[4] = off_time >> 8;
	this->transaction.configureWrite(buffer, 5);
	return this->runTransaction();
}


#endif