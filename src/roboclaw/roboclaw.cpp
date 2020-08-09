/*
 * roboclaw.cpp - implementation of the roboclaw C++ library using ROS serial
 *
 * This code is a modified version of the Ion Motion Control Arduino library.
 * To view the code in its original form, download it at
 *     http://downloads.ionmc.com/code/arduino.zip
 */

#include <roboclaw/roboclaw.hpp>

 /*
  * Macros taken directly from Arduino Library
  */
#define MAXRETRY 2
#define SetDWORDval(arg) (uint8_t)(((uint32_t)arg) >> 24), (uint8_t)(((uint32_t)arg) >> 16), (uint8_t)(((uint32_t)arg) >> 8), (uint8_t)arg
#define SetWORDval(arg) (uint8_t)(((uint16_t)arg) >> 8), (uint8_t)arg

  /*
   * Constructor opens port at desired baudrate
   */
Roboclaw::Roboclaw(std::string &port, uint32_t baudrate)
{
    /* initialize pointer to a new Serial port object */
    port_ = new serial::Serial(port, baudrate, serial::Timeout::simpleTimeout(100));
    port_->close();
    port_->open();
}

/*
 * Destructor closes serial port and frees the associated memory
 */
Roboclaw::~Roboclaw()
{
    port_->close();
    delete port_;
}

/*
 * writes a single byte to the serial port
 */
void Roboclaw::write(uint8_t byte)
{
    port_->write(&byte, 1);
}

/*
 * reads and returns a single byte from the serial port or -1 in error or timeout
 * returns an int16_t to be consistent with the Arduino library and allow for
 *       returning -1 error
 */
int16_t Roboclaw::read()
{
    uint8_t buff[1];
    if (port_->read(buff, 1) == 1)
    {
        return buff[0];
    }
    else
    {
        return -1;
    }
}

/*
 * flushes the serial port's input and output buffers
 */
void Roboclaw::flush()
{
    port_->flush();
}

/*
 * resets the crc calculation
 */
void Roboclaw::crc_clear()
{
    crc_ = 0;
}

/*
 * updates the crc calculation with based on the specified byte
 * see the Roboclaw sheet and user manual for more on this
 */
void Roboclaw::crc_update(uint8_t data)
{
    int i;
    crc_ = crc_ ^ ((uint16_t)data << 8);
    for (i = 0; i < 8; i++)
    {
        if (crc_ & 0x8000)
            crc_ = (crc_ << 1) ^ 0x1021;
        else
            crc_ <<= 1;
    }
}

/*
 * returns the current value of the crc
 * this is not necessary as private methods can directly access the crc_ attribute,
 *      but it is being kept for now to keep the original Arduino code intact
 */
uint16_t Roboclaw::crc_get()
{
    return crc_;
}

/*
 * writes n bytes to the serial port as specified by function arguments
 */
bool Roboclaw::write_n(uint8_t cnt, ...)
{
    uint8_t trys = MAXRETRY;
    do
    {
        crc_clear();

        va_list marker;
        va_start(marker, cnt); /* Initialize variable arguments */
        /* read each argument after cnt, update crc, and send the data */
        for (uint8_t index = 0; index < cnt; index++)
        {
            uint8_t data = va_arg(marker, int);
            crc_update(data);
            write(data);
        }
        va_end(marker); /* Reset variable arguments */
        /* send the crc to the Roboclaw and check for return value */
        uint8_t b1 = (crc_ >> 8) & 0xFF; // TODO: Make these 4 lines pretty.
        uint8_t b2 = (crc_)&0xFF;
        port_->write(&b1, 1);
        port_->write(&b2, 1);

        if (read() == 0xFF)
            return true;
    } while (trys--);
    return false;
}

/*
 * reads the output of a command into n uint32_t variables specified by pointer arguments
 */
bool Roboclaw::read_n(uint8_t cnt, uint8_t address, uint8_t cmd, ...)
{
    uint32_t value = 0;
    uint8_t trys = MAXRETRY;
    int16_t data;
    do
    {
        flush();

        data = 0;
        crc_clear();
        write(address);
        crc_update(address);
        write(cmd);
        crc_update(cmd);

        /* read each four byte output of the command into a uint32_t */
        va_list marker;
        va_start(marker, cmd); /* Initialize variable arguments. */
        for (uint8_t index = 0; index < cnt; index++)
        {
            /* retrieve the pointer to the next uint32_t to be updated */
            uint32_t *ptr = va_arg(marker, uint32_t *);

            if (data != -1)
            {
                data = read();
                crc_update(data);
                value = (uint32_t)data << 24;
            }
            else
            {
                break;
            }

            if (data != -1)
            {
                data = read();
                crc_update(data);
                value |= (uint32_t)data << 16;
            }
            else
            {
                break;
            }

            if (data != -1)
            {
                data = read();
                crc_update(data);
                value |= (uint32_t)data << 8;
            }
            else
            {
                break;
            }

            if (data != -1)
            {
                data = read();
                crc_update(data);
                value |= (uint32_t)data;
            }
            else
            {
                break;
            }

            *ptr = value;
        }
        va_end(marker); /* Reset variable arguments.      */

        /* read crc from the roboclaw and double check with our calculation */
        if (data != -1)
        {
            uint16_t ccrc;
            data = read();
            if (data != -1)
            {
                ccrc = data << 8;
                data = read();
                if (data != -1)
                {
                    ccrc |= data;
                    return crc_get() == ccrc;
                }
            }
        }
    } while (trys--);

    return false;
}

/*
 * reads a one byte register returning its contents or false in error
 */
uint8_t Roboclaw::read1(uint8_t address, uint8_t cmd, bool *valid)
{
    if (valid)
        *valid = false;

    uint8_t value = 0;
    uint8_t trys = MAXRETRY;
    /* data is a signed int to allow for -1 assignment in error */
    int16_t data;
    do
    {
        flush();

        crc_clear();
        write(address);
        crc_update(address);
        write(cmd);
        crc_update(cmd);

        data = read();
        crc_update(data);
        value = data;

        if (data != -1)
        {
            uint16_t ccrc;
            data = read();
            if (data != -1)
            {
                ccrc = data << 8;
                data = read();
                if (data != -1)
                {
                    ccrc |= data;
                    if (crc_get() == ccrc)
                    {
                        *valid = true;
                        return value;
                    }
                }
            }
        }
    } while (trys--);

    return false;
}

/*
 * reads a two byte register, returning its contents or false in error
 */
uint16_t Roboclaw::read2(uint8_t address, uint8_t cmd, bool *valid)
{
    if (valid)
        *valid = false;

    uint16_t value = 0;
    uint8_t trys = MAXRETRY;
    int16_t data;
    do
    {
        flush();

        crc_clear();
        write(address);
        crc_update(address);
        write(cmd);
        crc_update(cmd);

        data = read();
        crc_update(data);
        value = (uint16_t)data << 8;

        if (data != -1)
        {
            data = read();
            crc_update(data);
            value |= (uint16_t)data;
        }

        if (data != -1)
        {
            uint16_t ccrc;
            data = read();
            if (data != -1)
            {
                ccrc = data << 8;
                data = read();
                if (data != -1)
                {
                    ccrc |= data;
                    if (crc_get() == ccrc)
                    {
                        *valid = true;
                        return value;
                    }
                }
            }
        }
    } while (trys--);

    return false;
}

/*
 * reads a four byte register
 *      returns register contents or false in error
 */
uint32_t Roboclaw::read4(uint8_t address, uint8_t cmd, bool *valid)
{
    if (valid)
        *valid = false;

    uint16_t value = 0;
    uint8_t trys = MAXRETRY;
    int16_t data;
    do
    {
        flush();

        crc_clear();
        write(address);
        crc_update(address);
        write(cmd);
        crc_update(cmd);

        data = read();
        crc_update(data);
        value = (uint16_t)data << 8;

        if (data != -1)
        {
            data = read();
            crc_update(data);
            value |= (uint16_t)data;
        }

        if (data != -1)
        {
            uint16_t ccrc;
            data = read();
            if (data != -1)
            {
                ccrc = data << 8;
                data = read();
                if (data != -1)
                {
                    ccrc |= data;
                    if (crc_get() == ccrc)
                    {
                        *valid = true;
                        return value;
                    }
                }
            }
        }
    } while (trys--);

    return false;
}

/*
 * Reads a four byte register along with the roboclaw's status
 * returns the value of the register or false in the event of a timeout or error
 * updates value of status pointer argument
 * indicates success/failure through the valid argument
 */
uint32_t Roboclaw::read4_1(uint8_t address, uint8_t cmd, uint8_t *status, bool *valid)
{
    if (valid)
        *valid = false;

    uint16_t value = 0;
    uint8_t trys = MAXRETRY;
    int16_t data;
    do
    {
        flush();

        crc_clear();
        write(address);
        crc_update(address);
        write(cmd);
        crc_update(cmd);

        data = read();
        crc_update(data);
        value = (uint16_t)data << 8;

        if (data != -1)
        {
            data = read();
            crc_update(data);
            value |= (uint16_t)data;
        }

        if (data != -1)
        {
            uint16_t ccrc;
            data = read();
            if (data != -1)
            {
                ccrc = data << 8;
                data = read();
                if (data != -1)
                {
                    ccrc |= data;
                    if (crc_get() == ccrc)
                    {
                        *valid = true;
                        return value;
                    }
                }
            }
        }
    } while (trys--);

    return false;
}

/************************************************************
 * original methods unchanged from original Arduino library *
 ************************************************************/

 /**
  * Drive motor 1 forward. Valid data range is 0 - 127. A value of 127 = full speed forward, 64 =
  * about half speed forward and 0 = full stop.
  *
  * @param address Controller address value from 0x80 to 0x87.
  * @param speed 0 - 127 speed where 127 full speed, 0 is stopped.
  * @return true if success.
  */
bool Roboclaw::ForwardM1(uint8_t address, uint8_t speed)
{
    return write_n(3, address, M1FORWARD, speed);
}

/**
 * Drive motor 1 backwards. Valid data range is 0 - 127. A value of 127 full speed backwards, 64 =
 * about half speed backward and 0 = full stop.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param speed 0 - 127 speed where 127 full speed, 0 is stopped.
 * @return true if success.
 */
bool Roboclaw::BackwardM1(uint8_t address, uint8_t speed)
{
    return write_n(3, address, M1BACKWARD, speed);
}

/**
 * attention: SetMainVoltages is the preferred method to be used. Sets main battery (B- / B+) minimum voltage level. If the battery voltages drops below the set
 * voltage level RoboClaw will stop driving the motors. The voltage is set in .2 volt increments. A
 * value of 0 sets the minimum value allowed which is 6V. The valid data range is 0 - 140 (6V -
 * 34V). The formula for calculating the voltage is: (Desired Volts - 6) x 5 = Value. Examples of
 * valid values are 6V = 0, 8V = 10 and 11V = 25.
 *
 * @param address Controller address value from 0x80 to 0x87.
 * @param voltage 0 - 140, which corresponds to 6V - 34 V.
 * @return true if success.
 */
bool Roboclaw::SetMinVoltageMainBattery(uint8_t address, uint8_t voltage)
{
    return write_n(3, address, SETMINMB, voltage);
}

/**
 * attention: SetMainVoltages is the preferred method to be used. Sets main battery (B- / B+) maximum voltage level.
 * The valid data range is 30 - 175 (6V - 34V). During regenerative breaking a back voltage is
 * applied to charge the battery. When using a power supply, by setting the maximum voltage level,
 * RoboClaw will, before exceeding it, go into hard braking mode until the voltage drops below the
 * maximum value set. This will prevent overvoltage conditions when using power supplies.
 * The formula for calculating the voltage is: Desired Volts x 5.12 = Value.
 * Examples of valid values are 12V = 62, 16V = 82 and 24V = 123.
 *
 * @param address Controller address value from 0x80 to 0x87.
 * @param voltage 0 - 140, which corresponds to 6V - 34 V.
 * @return true if success.
 */
bool Roboclaw::SetMaxVoltageMainBattery(uint8_t address, uint8_t voltage)
{
    return write_n(3, address, SETMAXMB, voltage);
}

/**
 * Drive motor 2 forward. Valid data range is 0 - 127. A value of 127 full speed forward, 64 = about
 * half speed forward and 0 = full stop.
 *
 * @param address Controller address value from 0x80 to 0x87.
 * @param speed 0 - 127 speed where 127 full speed, 0 is stopped.
 * @return true if success.
 */
bool Roboclaw::ForwardM2(uint8_t address, uint8_t speed)
{
    return write_n(3, address, M2FORWARD, speed);
}

/**
 * Drive motor 2 backwards. Valid data range is 0 - 127. A value of 127 full speed backwards, 64 =
 * about half speed backward and 0 = full stop.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param speed 0 - 127 speed where 127 full speed, 0 is stopped.
 * @return true if success.
 */
bool Roboclaw::BackwardM2(uint8_t address, uint8_t speed)
{
    return write_n(3, address, M2BACKWARD, speed);
}

/**
 * Drive motor 1 forward or reverse. Valid data range is 0 - 127. A value of 0 = full speed reverse,
 * 64 = stop and 127 = full speed forward.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param speed 0 - 127 speed where 127 full speed forward, 64 is stopped, 0 is full speed reverse.
 * @return true if success.
 */
bool Roboclaw::ForwardBackwardM1(uint8_t address, uint8_t speed)
{
    return write_n(3, address, M17BIT, speed);
}

/**
 * Drive motor 1 forward or reverse. Valid data range is 0 - 127. A value of 0 = full speed reverse,
 * 64 = stop and 127 = full speed forward.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param speed 0 - 127 speed where 127 full speed forward, 64 is stopped, 0 is full speed reverse.
 * @return true if success.
 */
bool Roboclaw::ForwardBackwardM2(uint8_t address, uint8_t speed)
{
    return write_n(3, address, M27BIT, speed);
}

/**
 * Drive forward in mix mode (differential drive mode). Valid data range is 0 - 127. A value of 0 = full stop and 127 = full
 * forward.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param speed 0 - 127 speed where 127 full speed, 0 is stopped.
 * @return true if success.
 */
bool Roboclaw::ForwardMixed(uint8_t address, uint8_t speed)
{
    return write_n(3, address, MIXEDFORWARD, speed);
}

/**
 * Drive backward in mix mode (differential drive mode). Valid data range is 0 - 127. A value of 0 = full stop and 127 = full
 * reverse.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param speed 0 - 127 speed where 127 full reverse, 0 is stopped.
 * @return true if success.
 */
bool Roboclaw::BackwardMixed(uint8_t address, uint8_t speed)
{
    return write_n(3, address, MIXEDBACKWARD, speed);
}

/**
 * Turn right in mix mode (differential drive mode). Valid data range is 0 - 127. A value of 0 = stop turn and 127 = full
 * speed turn.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param speed 0 - 127 speed where 0 stop turn, 127 full speed turn.
 * @return true if success.
 */
bool Roboclaw::TurnRightMixed(uint8_t address, uint8_t speed)
{
    return write_n(3, address, MIXEDRIGHT, speed);
}

/**
 * Turn left in mix mode (differential drive mode). Valid data range is 0 - 127. A value of 0 = stop turn and 127 = full
 * speed turn.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param speed 0 - 127 speed where 0 stop turn, 127 full speed turn.
 * @return true if success.
 */
bool Roboclaw::TurnLeftMixed(uint8_t address, uint8_t speed)
{
    return write_n(3, address, MIXEDLEFT, speed);
}

/**
 * Drive forward or backwards. Valid data range is 0 - 127. A value of 0 = full backward, 64 = stop
 * and 127 = full forward
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param speed 0 - 127 value of 0 = full backward, 64 = stop and 127 = full forward
 * @return true if success.
 */
bool Roboclaw::ForwardBackwardMixed(uint8_t address, uint8_t speed)
{
    return write_n(3, address, MIXEDFB, speed);
}

/**
 * Turn left or right. Valid data range is 0 - 127. A value of 0 = full left, 0 = stop turn and 127 = full
 * right.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param speed 0 - 127, value of 0 = full left, 0 = stop turn and 127 = full right.
 * @return true if success.
 */
bool Roboclaw::LeftRightMixed(uint8_t address, uint8_t speed)
{
    return write_n(3, address, MIXEDLR, speed);
}

/**
 * Read M1 encoder count/position.
 * Quadrature encoders have a range of 0 to 4,294,967,295. Absolute encoder values are converted
 * from an analog voltage into a value from 0 to 2047 for the full 2v range.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param status The Status byte tracks counter underflow, direction and overflow. The byte value represents:
 * Bit0 - Counter Underflow (1= Underflow Occurred, Cleared After Reading)
 * Bit1 - Direction (0 = Forward, 1 = Backwards)
 * Bit2 - Counter Overflow (1= Underflow Occurred, Cleared After Reading)
 * @param valid true if success
 * @return value of encoder.
 */
uint32_t Roboclaw::ReadEncM1(uint8_t address, uint8_t *status, bool *valid)
{
    return read4_1(address, GETM1ENC, status, valid);
}

/**
 * Read M2 encoder count/position.
 * Quadrature encoders have a range of 0 to 4,294,967,295. Absolute encoder values are converted
 * from an analog voltage into a value from 0 to 2047 for the full 2v range.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param status The Status byte tracks counter underflow, direction and overflow. The byte value represents:
 * Bit0 - Counter Underflow (1= Underflow Occurred, Cleared After Reading)
 * Bit1 - Direction (0 = Forward, 1 = Backwards)
 * Bit2 - Counter Overflow (1= Underflow Occurred, Cleared After Reading)
 * @param valid true if success
 * @return value of encoder.
 */
uint32_t Roboclaw::ReadEncM2(uint8_t address, uint8_t *status, bool *valid)
{
    return read4_1(address, GETM2ENC, status, valid);
}

/**
 * Read M1 counter speed. Returned value is in pulses per second. RoboClaw keeps track of how
 * many pulses received per second for both encoder channels.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param status Status indicates the direction (0 – forward, 1 - backward).
 * @param valid true if success
 * @return value in pulses per second.
 */
uint32_t Roboclaw::ReadSpeedM1(uint8_t address, uint8_t *status, bool *valid)
{
    return read4_1(address, GETM1SPEED, status, valid);
}

/**
 * Read M2 counter speed. Returned value is in pulses per second. RoboClaw keeps track of how
 * many pulses received per second for both encoder channels. 
*
 * @param address Controller address value from 0x80 to 0x87
 * @param status Status indicates the direction (0 – forward, 1 - backward).
 * @param valid true if success
 * @return value in pulses per second.
 */
uint32_t Roboclaw::ReadSpeedM2(uint8_t address, uint8_t *status, bool *valid)
{
    return read4_1(address, GETM2SPEED, status, valid);
}

/**
 * Will reset both quadrature decoder counters to zero. This command applies to quadrature
 * encoders only. 
 *
 * @param address Controller address value from 0x80 to 0x87
 * @return true if success.
 */
bool Roboclaw::ResetEncoders(uint8_t address)
{
    return write_n(2, address, RESETENC);
}

/**
 * Read RoboClaw firmware version. Returns up to 48 bytes(depending on the Roboclaw model)
 * and is terminated by a line feed character and a null character.
 * The command will return up to 48 bytes. The return string includes the product name and
 * firmware version. The return string is terminated with a line feed (10) and null (0) character.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param version char pointer to the version string
 * @return true if success.
 */

bool Roboclaw::ReadVersion(uint8_t address, char *version)
{
    uint8_t data;
    uint8_t trys = MAXRETRY;
    do
    {
        flush();

        data = 0;

        crc_clear();
        write(address);
        crc_update(address);
        write(GETVERSION);
        crc_update(GETVERSION);

        uint8_t i;
        for (i = 0; i < 48; i++)
        {
            if (data != -1)
            {
                data = read();
                version[i] = data;
                crc_update(version[i]);
                if (version[i] == 0)
                {
                    uint16_t ccrc;
                    data = read();
                    if (data != -1)
                    {
                        ccrc = data << 8;
                        data = read();
                        if (data != -1)
                        {
                            ccrc |= data;
                            return crc_get() == ccrc;
                        }
                    }
                    break;
                }
            }
            else
            {
                break;
            }
        }
    } while (trys--);

    return false;
}

/**
 * Set the value of the Encoder 1 register. Useful when homing motor 1. This command applies to
 * quadrature encoders only.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param val value to set.
 * @return true if success.
 */
bool Roboclaw::SetEncM1(uint8_t address, int val)
{
    return write_n(6, address, SETM1ENCCOUNT, SetDWORDval(val));
}

/**
 * Set the value of the Encoder 2 register. Useful when homing motor 1. This command applies to
 * quadrature encoders only.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param val value to set.
 * @return true if success.
 */
bool Roboclaw::SetEncM2(uint8_t address, int val)
{
    return write_n(6, address, SETM2ENCCOUNT, SetDWORDval(val));
}

/**
 * Read the main battery voltage level connected to B+ and B- terminals. The voltage is returned in
 * 10ths of a volt(eg 300 = 30v).
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param valid true or false.
 * @return voltage value.
 */
uint16_t Roboclaw::ReadMainBatteryVoltage(uint8_t address, bool *valid)
{
    return read2(address, GETMBATT, valid);
}

/**
 * Read a logic battery voltage level connected to LB+ and LB- terminals. The voltage is returned in
 * 10ths of a volt(eg 50 = 5v).
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param valid true or false.
 * @return voltage value.
 */
uint16_t Roboclaw::ReadLogicBatteryVoltage(uint8_t address, bool *valid)
{
    return read2(address, GETLBATT, valid);
}

/**
 * Sets logic input (LB- / LB+) minimum voltage level. RoboClaw will shut down with an error if
* the voltage is below this level. The voltage is set in .2 volt increments. A value of 0 sets the
* minimum value allowed which is 6V. The valid data range is 0 - 140 (6V - 34V). The formula for
* calculating the voltage is: (Desired Volts - 6) x 5 = Value. Examples of valid values are 6V = 0,
* 8V = 10 and 11V = 25. 
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param voltage 0 - 140 (6V - 34V) 
 * @return true if success.
 */
bool Roboclaw::SetMinVoltageLogicBattery(uint8_t address, uint8_t voltage)
{
    return write_n(3, address, SETMINLB, voltage);
}

/**
 * Sets logic input (LB- / LB+) maximum voltage level. The valid data range is 30 - 175 (6V -
 * 34V). RoboClaw will shutdown with an error if the voltage is above this level. The formula for
 * calculating the voltage is: Desired Volts x 5.12 = Value. Examples of valid values are 12V = 62,
 * 16V = 82 and 24V = 123.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param voltage 30 - 175 (6V - 34V) 
 * @return true if success.
 */
bool Roboclaw::SetMaxVoltageLogicBattery(uint8_t address, uint8_t voltage)
{
    return write_n(3, address, SETMAXLB, voltage);
}

/**
 * M1 Several motor and quadrature combinations can be used with RoboClaw. In some cases the
 * default PID values will need to be tuned for the systems being driven. This gives greater
 * flexibility in what motor and encoder combinations can be used. The RoboClaw PID system
 * consist of four constants starting with QPPS, P = Proportional, I= Integral and D= Derivative.
 * The defaults values are:
 * QPPS = 44000
 * P = 0x00010000
 * I = 0x00008000
 * D = 0x00004000
 * QPPS is the speed of the encoder when the motor is at 100% power. P, I, D are the default
 * values used after a reset.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param kp_fp proportional value in PID controller
 * @param ki_fp integral value in PID controller
 * @param kd_fp derivative value in PID controller
 * @param qpps the speed of the encoder when the motor is at 100% power.
 * @return true if success.
 */
bool Roboclaw::SetM1VelocityPID(uint8_t address, float kp_fp, float ki_fp, float kd_fp, uint32_t qpps)
{
    uint32_t kp = kp_fp * 65536;
    uint32_t ki = ki_fp * 65536;
    uint32_t kd = kd_fp * 65536;
    return write_n(18, address, SETM1PID, SetDWORDval(kd), SetDWORDval(kp), SetDWORDval(ki), SetDWORDval(qpps));
}

/**
 * M2 Several motor and quadrature combinations can be used with RoboClaw. In some cases the
 * default PID values will need to be tuned for the systems being driven. This gives greater
 * flexibility in what motor and encoder combinations can be used. The RoboClaw PID system
 * consist of four constants starting with QPPS, P = Proportional, I= Integral and D= Derivative.
 * The defaults values are:
 * QPPS = 44000
 * P = 0x00010000
 * I = 0x00008000
 * D = 0x00004000
 * QPPS is the speed of the encoder when the motor is at 100% power. P, I, D are the default
 * values used after a reset.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param kp_fp proportional value in PID controller
 * @param ki_fp integral value in PID controller
 * @param kd_fp derivative value in PID controller
 * @param qpps the speed of the encoder when the motor is at 100% power.
 * @return true if success.
 */
bool Roboclaw::SetM2VelocityPID(uint8_t address, float kp_fp, float ki_fp, float kd_fp, uint32_t qpps)
{
    uint32_t kp = kp_fp * 65536;
    uint32_t ki = ki_fp * 65536;
    uint32_t kd = kd_fp * 65536;
    return write_n(18, address, SETM2PID, SetDWORDval(kd), SetDWORDval(kp), SetDWORDval(ki), SetDWORDval(qpps));
}

/**
 * M1 Read the pulses counted in that last 300th of a second. This is an unfiltered version of command
 * 18. Command 30 can be used to make a independent PID routine. Value returned is in encoder
 * counts per second. 
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param status Status indicates the direction (0 – forward, 1 - backward).
 * @param valid true if success
 * @return speed value.
 */
uint32_t Roboclaw::ReadISpeedM1(uint8_t address, uint8_t *status, bool *valid)
{
    return read4_1(address, GETM1ISPEED, status, valid);
}

/**
 * M1 Read the pulses counted in that last 300th of a second. This is an unfiltered version of command
 * 18. Command 30 can be used to make a independent PID routine. Value returned is in encoder
 * counts per second. 
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param status Status indicates the direction (0 – forward, 1 - backward).
 * @param valid true if success
 * @return speed value.
 */
uint32_t Roboclaw::ReadISpeedM2(uint8_t address, uint8_t *status, bool *valid)
{
    return read4_1(address, GETM2ISPEED, status, valid);
}

/**
 * Drive M1 using a duty cycle value. The duty cycle is used to control the speed of the motor
 * without a quadrature encoder.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param duty The duty value is signed and the range is -32767 to +32767 (eg. +-100% duty). 
 * @return true if success.
 */
bool Roboclaw::DutyM1(uint8_t address, uint16_t duty)
{
    return write_n(4, address, M1DUTY, SetWORDval(duty));
}

/**
 * Drive M2 using a duty cycle value. The duty cycle is used to control the speed of the motor
 * without a quadrature encoder.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param duty The duty value is signed and the range is -32767 to +32767 (eg. +-100% duty). 
 * @return true if success.
 */
bool Roboclaw::DutyM2(uint8_t address, uint16_t duty)
{
    return write_n(4, address, M2DUTY, SetWORDval(duty));
}

/**
 * Drive both M1 and M2 using a duty cycle value. The duty cycle is used to control the speed of
 * the motor without a quadrature encoder. 
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param duty The duty value is signed and the range is -32767 to +32767 (eg. +-100% duty). 
 * @return true if success.
 */
bool Roboclaw::DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2)
{
    return write_n(6, address, MIXEDDUTY, SetWORDval(duty1), SetWORDval(duty2));
}

/**
 * Drive M1 using a speed value. The sign indicates which direction the motor will turn. This
 * command is used to drive the motor by quad pulses per second. Different quadrature encoders
 * will have different rates at which they generate the incoming pulses. The values used will differ
 * from one encoder to another. Once a value is sent the motor will begin to accelerate as fast as
 * possible until the defined rate is reached.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param speed speed of motor 
 * @return true if success.
 */
bool Roboclaw::SpeedM1(uint8_t address, uint32_t speed)
{
    return write_n(6, address, M1SPEED, SetDWORDval(speed));
}

/**
 * Drive M2 using a speed value. The sign indicates which direction the motor will turn. This
 * command is used to drive the motor by quad pulses per second. Different quadrature encoders
 * will have different rates at which they generate the incoming pulses. The values used will differ
 * from one encoder to another. Once a value is sent the motor will begin to accelerate as fast as
 * possible until the defined rate is reached.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param speed speed of motor 
 * @return true if success.
 */
bool Roboclaw::SpeedM2(uint8_t address, uint32_t speed)
{
    return write_n(6, address, M2SPEED, SetDWORDval(speed));
}

/**
 * Drive M1 and M2 in the same command using a signed speed value. The sign indicates which
 * direction the motor will turn. This command is used to drive both motors by quad pulses per
 * second. Different quadrature encoders will have different rates at which they generate the
 * incoming pulses. The values used will differ from one encoder to another. Once a value is sent
 * the motor will begin to accelerate as fast as possible until the rate defined is reached.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param speed1 speed of motor m1
 * @param speed2 speed of motor m2
 * @return true if success.
 */
bool Roboclaw::SpeedM1M2(uint8_t address, uint32_t speed1, uint32_t speed2)
{
    return write_n(10, address, MIXEDSPEED, SetDWORDval(speed1), SetDWORDval(speed2));
}

/**
 * Drive M1 with a signed speed and acceleration value. The sign indicates which direction the
 * motor will run. The acceleration values are not signed. This command is used to drive the motor
 * by quad pulses per second and using an acceleration value for ramping. Different quadrature
 * encoders will have different rates at which they generate the incoming pulses. The values used
 * will differ from one encoder to another. Once a value is sent the motor will begin to accelerate
 * incrementally until the rate defined is reached.
 * The acceleration is measured in speed increase per second. An acceleration value of 12,000
 * QPPS with a speed of 12,000 QPPS would accelerate a motor from 0 to 12,000 QPPS in 1
 * second. Another example would be an acceleration value of 24,000 QPPS and a speed value of
 * 12,000 QPPS would accelerate the motor to 12,000 QPPS in 0.5 seconds
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param accel acceleration
 * @param speed speed of motor
 * @return true if success.
 */
bool Roboclaw::SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed)
{
    return write_n(10, address, M1SPEEDACCEL, SetDWORDval(accel), SetDWORDval(speed));
}

/**
 * Drive M2 with a signed speed and acceleration value. The sign indicates which direction the
 * motor will run. The acceleration values are not signed. This command is used to drive the motor
 * by quad pulses per second and using an acceleration value for ramping. Different quadrature
 * encoders will have different rates at which they generate the incoming pulses. The values used
 * will differ from one encoder to another. Once a value is sent the motor will begin to accelerate
 * incrementally until the rate defined is reached.
 * The acceleration is measured in speed increase per second. An acceleration value of 12,000
 * QPPS with a speed of 12,000 QPPS would accelerate a motor from 0 to 12,000 QPPS in 1
 * second. Another example would be an acceleration value of 24,000 QPPS and a speed value of
 * 12,000 QPPS would accelerate the motor to 12,000 QPPS in 0.5 seconds
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param accel acceleration
 * @param speed speed of motor
 * @return true if success.
 */
bool Roboclaw::SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed)
{
    return write_n(10, address, M2SPEEDACCEL, SetDWORDval(accel), SetDWORDval(speed));
}

/**
 * Drive M1 and M2 in the same command using one value for acceleration and two signed speed
 * values for each motor. The sign indicates which direction the motor will run. The acceleration
 * value is not signed. The motors are sync during acceleration. This command is used to drive
 * the motor by quad pulses per second and using an acceleration value for ramping. Different
 * quadrature encoders will have different rates at which they generate the incoming pulses. The
 * values used will differ from one encoder to another. Once a value is sent the motor will begin to
 * accelerate incrementally until the rate defined is reached.
 * The acceleration is measured in speed increase per second. An acceleration value of 12,000
 * QPPS with a speed of 12,000 QPPS would accelerate a motor from 0 to 12,000 QPPS in 1
 * second. Another example would be an acceleration value of 24,000 QPPS and a speed value of
 * 12,000 QPPS would accelerate the motor to 12,000 QPPS in 0.5 seconds
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param accel acceleration
 * @param speed speed of motor
 * @return true if success.
 */
bool Roboclaw::SpeedAccelM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t speed2)
{
    return write_n(14, address, MIXEDSPEEDACCEL, SetDWORDval(accel), SetDWORDval(speed1), SetDWORDval(speed2));
}

/**
 * Drive M1 with a signed speed and distance value. The sign indicates which direction the motor
 * will run. The distance value is not signed. This command is buffered. This command is used to
 * control the top speed and total distance traveled by the motor. Each motor channel M1 and M2
 * have separate buffers. This command will execute immediately if no other command for that
 * channel is executing, otherwise the command will be buffered in the order it was sent. Any
 * buffered or executing command can be stopped when a new command is issued by setting the
 * Flag argument. All values used are in quad pulses per second.
 * The flag argument can be set to a 1 or 0. If a value of 0 is used the command will be buffered
 * and executed in the order sent. If a value of 1 is used the current running command is stopped,
 * any other commands in the buffer are deleted and the new command is executed.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param speed speed of motor
 * @param distance distance to drice
 * @param flag  set to a 1 or 0. If a value of 0 is used the command will be buffered
 * and executed in the order sent. If a value of 1 is used the current running command is stopped,
 * any other commands in the buffer are deleted and the new command is executed
 * @return true if success.
 */
bool Roboclaw::SpeedDistanceM1(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag)
{
    return write_n(11, address, M1SPEEDDIST, SetDWORDval(speed), SetDWORDval(distance), flag);
}

/**
 * Drive M2 with a signed speed and distance value. The sign indicates which direction the motor
 * will run. The distance value is not signed. This command is buffered. This command is used to
 * control the top speed and total distance traveled by the motor. Each motor channel M1 and M2
 * have separate buffers. This command will execute immediately if no other command for that
 * channel is executing, otherwise the command will be buffered in the order it was sent. Any
 * buffered or executing command can be stopped when a new command is issued by setting the
 * Flag argument. All values used are in quad pulses per second.
 * The flag argument can be set to a 1 or 0. If a value of 0 is used the command will be buffered
 * and executed in the order sent. If a value of 1 is used the current running command is stopped,
 * any other commands in the buffer are deleted and the new command is executed.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param speed speed of motor
 * @param distance distance to drice
 * @param flag  set to a 1 or 0. If a value of 0 is used the command will be buffered
 * and executed in the order sent. If a value of 1 is used the current running command is stopped,
 * any other commands in the buffer are deleted and the new command is executed
 * @return true if success.
 */
bool Roboclaw::SpeedDistanceM2(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag)
{
    return write_n(11, address, M2SPEEDDIST, SetDWORDval(speed), SetDWORDval(distance), flag);
}

/**
 * Drive M1 and M2 with a speed and distance value. The sign indicates which direction the motor
 * will run. The distance value is not signed. This command is buffered. Each motor channel M1
 * and M2 have separate buffers. This command will execute immediately if no other command for
 * that channel is executing, otherwise the command will be buffered in the order it was sent. Any
 * buffered or executing command can be stopped when a new command is issued by setting the
 * Buffer argument. All values used are in quad pulses per second.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param speed1 speed of motor 1
 * @param distance1 distance for motor 1
 * @param speed1 speed of motor 2
 * @param distance1 distance for motor 2
 * @param flag  set to a 1 or 0. If a value of 0 is used the command will be buffered
 * and executed in the order sent. If a value of 1 is used the current running command is stopped,
 * any other commands in the buffer are deleted and the new command is executed
 * @return true if success.
 */
bool Roboclaw::SpeedDistanceM1M2(uint8_t address, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag)
{
    return write_n(19, address, MIXEDSPEEDDIST, SetDWORDval(speed1), SetDWORDval(distance1), SetDWORDval(speed2), SetDWORDval(distance2), flag);
}

/**
 * Drive M1 with a speed, acceleration and distance value. The sign indicates which direction the
 * motor will run. The acceleration and distance values are not signed. This command is used to
 * control the motors top speed, total distanced traveled and at what incremental acceleration value
 * to use until the top speed is reached. Each motor channel M1 and M2 have separate buffers. This
 * command will execute immediately if no other command for that channel is executing, otherwise
 * the command will be buffered in the order it was sent. Any buffered or executing command can
 * be stopped when a new command is issued by setting the Buffer argument. All values used are
 * in quad pulses per second.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param accel acceleration of motor M1
 * @param speed speed of motor M1
 * @param distance distance for motor M1
 * @param flag  set to a 1 or 0. If a value of 0 is used the command will be buffered
 * and executed in the order sent. If a value of 1 is used the current running command is stopped,
 * any other commands in the buffer are deleted and the new command is executed
 * @return true if success.
 */
bool Roboclaw::SpeedAccelDistanceM1(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag)
{
    return write_n(15, address, M1SPEEDACCELDIST, SetDWORDval(accel), SetDWORDval(speed), SetDWORDval(distance), flag);
}

/**
 * Drive M2 with a speed, acceleration and distance value. The sign indicates which direction the
 * motor will run. The acceleration and distance values are not signed. This command is used to
 * control the motors top speed, total distanced traveled and at what incremental acceleration value
 * to use until the top speed is reached. Each motor channel M1 and M2 have separate buffers. This
 * command will execute immediately if no other command for that channel is executing, otherwise
 * the command will be buffered in the order it was sent. Any buffered or executing command can
 * be stopped when a new command is issued by setting the Buffer argument. All values used are
 * in quad pulses per second.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param accel acceleration of motor M2
 * @param speed speed of motor M2
 * @param distance distance for motor M2
 * @param flag  set to a 1 or 0. If a value of 0 is used the command will be buffered
 * and executed in the order sent. If a value of 1 is used the current running command is stopped,
 * any other commands in the buffer are deleted and the new command is executed
 * @return true if success.
 */
bool Roboclaw::SpeedAccelDistanceM2(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag)
{
    return write_n(15, address, M2SPEEDACCELDIST, SetDWORDval(accel), SetDWORDval(speed), SetDWORDval(distance), flag);
}

/**
 * Drive M1 and M2 with a speed, acceleration and distance value. The sign indicates which
 * direction the motor will run. The acceleration and distance values are not signed. This command
 * is used to control both motors top speed, total distanced traveled and at what incremental
 * acceleration value to use until the top speed is reached. Each motor channel M1 and M2 have
 * separate buffers. This command will execute immediately if no other command for that channel
 * is executing, otherwise the command will be buffered in the order it was sent. Any buffered
 * or executing command can be stopped when a new command is issued by setting the Buffer
 * argument. All values used are in quad pulses per second.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param accel acceleration of motors
 * @param speed1 speed of motor 1
 * @param distance1 distance for motor 1
 * @param speed2 speed of motor 2
 * @param distance2 distance for motor 2
 * @param flag  set to a 1 or 0. If a value of 0 is used the command will be buffered
 * and executed in the order sent. If a value of 1 is used the current running command is stopped,
 * any other commands in the buffer are deleted and the new command is executed
 * @return true if success.
 */
bool Roboclaw::SpeedAccelDistanceM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag)
{
    return write_n(23, address, MIXEDSPEEDACCELDIST, SetDWORDval(accel), SetDWORDval(speed1), SetDWORDval(distance1), SetDWORDval(speed2), SetDWORDval(distance2), flag);
}

/**
 * Read both motor M1 and M2 buffer lengths. This command can be used to determine how many
 * commands are waiting to execute.
 * The return values represent how many commands per buffer are waiting to be executed. The
 * maximum buffer size per motor is 64 commands(0x3F). A return value of 0x80(128) indicates
 * the buffer is empty. A return value of 0 indiciates the last command sent is executing. A value of
 * 0x80 indicates the last command buffered has finished.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param depth1 depth of buffer 1
 * @param depth2 depth of buffer 2
 * 
 * @reurn represent how many commands per buffer are waiting to be executed.
 */
bool Roboclaw::ReadBuffers(uint8_t address, uint8_t &depth1, uint8_t &depth2)
{
    bool valid;
    uint16_t value = read2(address, GETBUFFERS, &valid);
    if (valid)
    {
        depth1 = value >> 8;
        depth2 = value;
    }
    return valid;
}

/**
 * Read the current PWM output values for the motor channels. The values returned are +/-32767.
 * The duty cycle percent is calculated by dividing the Value by 327.67.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param pwm1 pwm 1
 * @param pwm2 pwm 2
 * 
 * @reurn pwm values +/-32767
 */
bool Roboclaw::ReadPWMs(uint8_t address, int16_t &pwm1, int16_t &pwm2)
{
    bool valid;
    uint32_t value = read4(address, GETPWMS, &valid);
    if (valid)
    {
        pwm1 = value >> 16;
        pwm2 = value & 0xFFFF;
    }
    return valid;
}

/**
 * Read the current draw from each motor in 10ma increments. The amps value is calculated by
 * dividing the value by 100. 
 *
 * param address Controller address value from 0x80 to 0x87
 * @param current1 current 1
 * @param current2 current 2
 * 
 * @reurn pwm values +/-32767
 */
bool Roboclaw::ReadCurrents(uint8_t address, int16_t &current1, int16_t &current2)
{
    bool valid;
    uint32_t value = read4(address, GETCURRENTS, &valid);
    if (valid)
    {
        current1 = value >> 16;
        current2 = value & 0xFFFF;
    }
    return valid;
}

/**
 * Drive M1 and M2 in the same command using one value for acceleration and two signed speed
 * values for each motor. The sign indicates which direction the motor will run. The acceleration
 * value is not signed. The motors are sync during acceleration. This command is used to drive
 * the motor by quad pulses per second and using an acceleration value for ramping. Different
 * quadrature encoders will have different rates at which they generate the incoming pulses. The
 * values used will differ from one encoder to another. Once a value is sent the motor will begin to
 * accelerate incrementally until the rate defined is reached.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param accel1 acceleration of motor 1
 * @param speed1 speed of motor 1
 * @param accel2 acceleration of motor 2
 * @param speed2 speed of motor 2
 * @param flag  set to a 1 or 0. If a value of 0 is used the command will be buffered
 * and executed in the order sent. If a value of 1 is used the current running command is stopped,
 * any other commands in the buffer are deleted and the new command is executed
 * @return true if success.
 */
bool Roboclaw::SpeedAccelM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2)
{
    return write_n(18, address, MIXEDSPEED2ACCEL, SetDWORDval(accel1), SetDWORDval(speed1), SetDWORDval(accel2), SetDWORDval(speed2));
}

/**
 * Drive M1 and M2 in the same command using one value for acceleration and two signed speed
 * values for each motor. The sign indicates which direction the motor will run. The acceleration
 * value is not signed. The motors are sync during acceleration. This command is used to drive
 * the motor by quad pulses per second and using an acceleration value for ramping. Different
 * quadrature encoders will have different rates at which they generate the incoming pulses. The
 * values used will differ from one encoder to another. Once a value is sent the motor will begin to
 * accelerate incrementally until the rate defined is reached.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param accel1 acceleration of motor 1
 * @param speed1 speed of motor 1
 * @param distance1 distance for motor 1
 * @param accel2 acceleration of motor 2
 * @param speed2 speed of motor 2
 * @param distance2 distance for motor 2
 * @param flag  set to a 1 or 0. If a value of 0 is used the command will be buffered
 * and executed in the order sent. If a value of 1 is used the current running command is stopped,
 * any other commands in the buffer are deleted and the new command is executed
 * @return true if success.
 */
bool Roboclaw::SpeedAccelDistanceM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag)
{
    return write_n(27, address, MIXEDSPEED2ACCELDIST, SetDWORDval(accel1), SetDWORDval(speed1), SetDWORDval(distance1), SetDWORDval(accel2), SetDWORDval(speed2), SetDWORDval(distance2), flag);
}


/**
 * Drive M1 with a signed duty and acceleration value. The sign indicates which direction the motor
 * will run. The acceleration values are not signed. This command is used to drive the motor by
 * PWM and using an acceleration value for ramping. Accel is the rate per second at which the duty
 * changes from the current duty to the specified duty.
 * The duty value is signed and the range is -32768 to +32767(eg. +-100% duty). The accel value
 * range is 0 to 655359(eg maximum acceleration rate is -100% to 100% in 100ms).
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param duty duty value is signed and the range is -32768 to +32767(eg. +-100% duty)
 * @param accel  The accel value range is 0 to 655359 g maximum acceleration rate is -100% to 100% in 100ms
 * @return true if success.
 */
bool Roboclaw::DutyAccelM1(uint8_t address, uint16_t duty, uint32_t accel)
{
    return write_n(8, address, M1DUTYACCEL, SetWORDval(duty), SetDWORDval(accel));
}

/**
 * Drive M1 with a signed duty and acceleration value. The sign indicates which direction the motor
 * will run. The acceleration values are not signed. This command is used to drive the motor by
 * PWM and using an acceleration value for ramping. Accel is the rate per second at which the duty
 * changes from the current duty to the specified duty.
 * The duty value is signed and the range is -32768 to +32767(eg. +-100% duty). The accel value
 * range is 0 to 655359(eg maximum acceleration rate is -100% to 100% in 100ms).
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param duty duty value is signed and the range is -32768 to +32767(eg. +-100% duty)
 * @param accel  The accel value range is 0 to 655359 g maximum acceleration rate is -100% to 100% in 100ms
 * @return true if success.
 */
bool Roboclaw::DutyAccelM2(uint8_t address, uint16_t duty, uint32_t accel)
{
    return write_n(8, address, M2DUTYACCEL, SetWORDval(duty), SetDWORDval(accel));
}

/**
 * Drive M1 and M2 in the same command using acceleration and duty values for each motor.
 * The sign indicates which direction the motor will run. The acceleration value is not signed. This
 * command is used to drive the motor by PWM using an acceleration value for ramping
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param duty duty value is signed and the range is -32768 to +32767(eg. +-100% duty)
 * @param accel  The accel value range is 0 to 655359 g maximum acceleration rate is -100% to 100% in 100ms
 * @return true if success.
 */
bool Roboclaw::DutyAccelM1M2(uint8_t address, uint16_t duty1, uint32_t accel1, uint16_t duty2, uint32_t accel2)
{
    return write_n(14, address, MIXEDDUTYACCEL, SetWORDval(duty1), SetDWORDval(accel1), SetWORDval(duty2), SetDWORDval(accel2));
}

/**
 * Read M1 PID and QPPS Settings.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param Kp_fp p value
 * @param Ki_fp i value
 * @param Kd_fp d value
 * @param qpps QPPS
 * @return true if success.
 */
bool Roboclaw::ReadM1VelocityPID(uint8_t address, float &Kp_fp, float &Ki_fp, float &Kd_fp, uint32_t &qpps)
{
    uint32_t Kp, Ki, Kd;
    bool valid = read_n(4, address, READM1PID, &Kp, &Ki, &Kd, &qpps);
    Kp_fp = ((float)Kp) / 65536;
    Ki_fp = ((float)Ki) / 65536;
    Kd_fp = ((float)Kd) / 65536;
    return valid;
}

/**
 * Read M2 PID and QPPS Settings.
 *
 * @param address Controller address value from 0x80 to 0x87
 * @param Kp_fp p value
 * @param Ki_fp i value
 * @param Kd_fp d value
 * @param qpps QPPS
 * @return true if success.
 */
bool Roboclaw::ReadM2VelocityPID(uint8_t address, float &Kp_fp, float &Ki_fp, float &Kd_fp, uint32_t &qpps)
{
    uint32_t Kp, Ki, Kd;
    bool valid = read_n(4, address, READM2PID, &Kp, &Ki, &Kd, &qpps);
    Kp_fp = ((float)Kp) / 65536;
    Ki_fp = ((float)Ki) / 65536;
    Kd_fp = ((float)Kd) / 65536;
    return valid;
}

/**
 * Set the Main Battery Voltage cutoffs, Min and Max. Min and Max voltages are in 10th of a volt
 * increments. Multiply the voltage to set by 10.
 * @param address Controller address value from 0x80 to 0x87
 * @param min min voltage cutoff
 * @param max max voltage cutoff
 * @return true if success.
 */
bool Roboclaw::SetMainVoltages(uint8_t address, uint16_t min, uint16_t max)
{
    return write_n(6, address, SETMAINVOLTAGES, SetWORDval(min), SetWORDval(max));
}

/**
 * Set the Logic Battery Voltage cutoffs, Min and Max. Min and Max voltages are in 10th of a volt
 * increments. Multiply the voltage to set by 10.
 * @param address Controller address value from 0x80 to 0x87
 * @param min min voltage cutoff
 * @param max max voltage cutoff
 * @return true if success.
 */
bool Roboclaw::SetLogicVoltages(uint8_t address, uint16_t min, uint16_t max)
{
    return write_n(6, address, SETLOGICVOLTAGES, SetWORDval(min), SetWORDval(max));
}

/**
 * Read the Main Battery Voltage Settings. The voltage is calculated by dividing the value by 10 
 * @paam address Controller address value from 0x80 to 0x87
 * @param min min voltage cutoff
 * @param max max voltage cutoff
 * @return true if success.
 */
bool Roboclaw::ReadMinMaxMainVoltages(uint8_t address, uint16_t &min, uint16_t &max)
{
    bool valid;
    uint32_t value = read4(address, GETMINMAXMAINVOLTAGES, &valid);
    if (valid)
    {
        min = value >> 16;
        max = value & 0xFFFF;
    }
    return valid;
}

/**
 * Read the Logic Battery Voltage Settings. The voltage is calculated by dividing the value by 10 
 * @paam address Controller address value from 0x80 to 0x87
 * @param min min voltage cutoff
 * @param max max voltage cutoff
 * @return true if success.
 */
bool Roboclaw::ReadMinMaxLogicVoltages(uint8_t address, uint16_t &min, uint16_t &max)
{
    bool valid;
    uint32_t value = read4(address, GETMINMAXLOGICVOLTAGES, &valid);
    if (valid)
    {
        min = value >> 16;
        max = value & 0xFFFF;
    }
    return valid;
}

/**
 * Set M1 position PID constants. The RoboClaw Position PID system consist of seven constants starting with P = Proportional, I=
 * Integral and D= Derivative, MaxI = Maximum Integral windup, Deadzone in encoder counts,
 * MinPos = Minimum Position and MaxPos = Maximum Position. The defaults values are all zero. 
 * Postion constants are used only with the Position commands, 65,66 and 67 or when encoders
 * are enabled in RC/Analog modes.
 * @param address Controller address value from 0x80 to 0x87
 * @param kp_fp min voltage cutoff
 * @param ki_fp max voltage cutoff
 * @param kd_fp max voltage cutoff
 * @param kiMax min voltage cutoff
 * @param deadzone max voltage cutoff
 * @param min max voltage cutoff
 * @param max max voltage cutoff
 * @return true if success.
 */
bool Roboclaw::SetM1PositionPID(uint8_t address, float kp_fp, float ki_fp, float kd_fp, uint32_t kiMax, uint32_t deadzone, uint32_t min, uint32_t max)
{
    uint32_t kp = kp_fp * 1024;
    uint32_t ki = ki_fp * 1024;
    uint32_t kd = kd_fp * 1024;
    return write_n(30, address, SETM1POSPID, SetDWORDval(kd), SetDWORDval(kp), SetDWORDval(ki), SetDWORDval(kiMax), SetDWORDval(deadzone), SetDWORDval(min), SetDWORDval(max));
}

/**
 * Set M2 position PID constants. The RoboClaw Position PID system consist of seven constants starting with P = Proportional, I=
 * Integral and D= Derivative, MaxI = Maximum Integral windup, Deadzone in encoder counts,
 * MinPos = Minimum Position and MaxPos = Maximum Position. The defaults values are all zero.
 * Position constants are used only with the Position commands, 65,66 and 67 or when encoders
 * are enabled in RC/Analog modes.
 * @param address Controller address value from 0x80 to 0x87
 * @param kp_fp p value
 * @param ki_fp i value
 * @param kd_fp d value
 * @param kiMax maximum integral windup
 * @param deadzone deadzone in encoder counts
 * @param min minimum position
 * @param max maximum position
 * @return true if success.
 */
bool Roboclaw::SetM2PositionPID(uint8_t address, float kp_fp, float ki_fp, float kd_fp, uint32_t kiMax, uint32_t deadzone, uint32_t min, uint32_t max)
{
    uint32_t kp = kp_fp * 1024;
    uint32_t ki = ki_fp * 1024;
    uint32_t kd = kd_fp * 1024;
    return write_n(30, address, SETM2POSPID, SetDWORDval(kd), SetDWORDval(kp), SetDWORDval(ki), SetDWORDval(kiMax), SetDWORDval(deadzone), SetDWORDval(min), SetDWORDval(max));
}

/**
 * Read M1 Position PID Constants - Read the Position PID Settings.
 * @param address Controller address value from 0x80 to 0x87
 * @param Kp_fp p value
 * @param Ki_fp i value
 * @param Kd_fp d value
 * @param KiMax maximum integral windup
 * @param DeadZone deadzone in encoder counts
 * @param Min minimum position
 * @param Max maximum position
 * @return true if success.
 */
bool Roboclaw::ReadM1PositionPID(uint8_t address, float &Kp_fp, float &Ki_fp, float &Kd_fp, uint32_t &KiMax, uint32_t &DeadZone, uint32_t &Min, uint32_t &Max)
{
    uint32_t Kp, Ki, Kd;
    bool valid = read_n(7, address, READM1POSPID, &Kp, &Ki, &Kd, &KiMax, &DeadZone, &Min, &Max);
    Kp_fp = ((float)Kp) / 1024;
    Ki_fp = ((float)Ki) / 1024;
    Kd_fp = ((float)Kd) / 1024;
    return valid;
}

/**
 * Read M2 Position PID Constants - Read the Position PID Settings.
 * @param address Controller address value from 0x80 to 0x87
 * @param Kp_fp p value
 * @param Ki_fp i value
 * @param Kd_fp d value
 * @param KiMax maximum integral windup
 * @param DeadZone deadzone in encoder counts
 * @param Min minimum position
 * @param Max maximum position
 * @return true if success.
 */
bool Roboclaw::ReadM2PositionPID(uint8_t address, float &Kp_fp, float &Ki_fp, float &Kd_fp, uint32_t &KiMax, uint32_t &DeadZone, uint32_t &Min, uint32_t &Max)
{
    uint32_t Kp, Ki, Kd;
    bool valid = read_n(7, address, READM2POSPID, &Kp, &Ki, &Kd, &KiMax, &DeadZone, &Min, &Max);
    Kp_fp = ((float)Kp) / 1024;
    Ki_fp = ((float)Ki) / 1024;
    Kd_fp = ((float)Kd) / 1024;
    return valid;
}

/**
 * Move M1 position from the current position to the specified new position and hold the new
 * position. Accel sets the acceleration value and deccel the decceleration value. QSpeed sets the
 * speed in quadrature pulses the motor will run at after acceleration and before decceleration.
 * @param address Controller address value from 0x80 to 0x87
 * @param accel acceleration value
 * @param speed speed value
 * @param deccel decceleration value
 * @param position position value
 * @return true if success.
 */
bool Roboclaw::SpeedAccelDeccelPositionM1(uint8_t address, uint32_t accel, uint32_t speed, uint32_t deccel, uint32_t position, uint8_t flag)
{
    return write_n(19, address, M1SPEEDACCELDECCELPOS, SetDWORDval(accel), SetDWORDval(speed), SetDWORDval(deccel), SetDWORDval(position), flag);
}

/**
 * Move M2 position from the current position to the specified new position and hold the new
 * position. Accel sets the acceleration value and deccel the decceleration value. QSpeed sets the
 * speed in quadrature pulses the motor will run at after acceleration and before decceleration.
 * @param address Controller address value from 0x80 to 0x87
 * @param accel acceleration value
 * @param speed speed value
 * @param deccel decceleration value
 * @param position position value
 * @return true if success.
 */
bool Roboclaw::SpeedAccelDeccelPositionM2(uint8_t address, uint32_t accel, uint32_t speed, uint32_t deccel, uint32_t position, uint8_t flag)
{
    return write_n(19, address, M2SPEEDACCELDECCELPOS, SetDWORDval(accel), SetDWORDval(speed), SetDWORDval(deccel), SetDWORDval(position), flag);
}

/**
 * Move M1 & M2 positions from their current positions to the specified new positions and hold the
 * new positions. Accel sets the acceleration value and deccel the decceleration value. QSpeed sets
 * the speed in quadrature pulses the motor will run at after acceleration and before decceleration. 
 * @paam address Controller address value from 0x80 to 0x87
 * @param accel1 acceleration value
 * @param speed1 speed value
 * @param deccel1 decceleration value
 * @param position1 position value
 * @param accel2 acceleration value
 * @param speed2 speed value
 * @param deccel2 decceleration value
 * @param position2 position value
 * @param position2 position value
 * @return true if success.
 */
bool Roboclaw::SpeedAccelDeccelPositionM1M2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t deccel1, uint32_t position1, uint32_t accel2, uint32_t speed2, uint32_t deccel2, uint32_t position2, uint8_t flag)
{
    return write_n(35, address, MIXEDSPEEDACCELDECCELPOS, SetDWORDval(accel1), SetDWORDval(speed1), SetDWORDval(deccel1), SetDWORDval(position1), SetDWORDval(accel2), SetDWORDval(speed2), SetDWORDval(deccel2), SetDWORDval(position2), flag);
}

/** 
 * SetM1 Default Duty Acceleration
 * Set the default acceleration for M1 when using duty cycle commands(Cmds 32,33 and 34) or
 * when using Standard Serial, RC and Analog PWM modes.
 * @param address Controller address value from 0x80 to 0x87
 * @param accel acceleration value
 * @return true if success.
 */
bool Roboclaw::SetM1DefaultAccel(uint8_t address, uint32_t accel)
{
    return write_n(6, address, SETM1DEFAULTACCEL, SetDWORDval(accel));
}

/** 
 * SetM2 Default Duty Acceleration
 * Set the default acceleration for M2 when using duty cycle commands(Cmds 32,33 and 34) or
 * when using Standard Serial, RC and Analog PWM modes.
 * @param address Controller address value from 0x80 to 0x87
 * @param accel acceleration value
 * @return true if success.
 */
bool Roboclaw::SetM2DefaultAccel(uint8_t address, uint32_t accel)
{
    return write_n(6, address, SETM2DEFAULTACCEL, SetDWORDval(accel));
}

/** 
 * Setmodes for S3,S4 and S5. 
 * @paam address Controller address value from 0x80 to 0x87
 * @param S3mode S3mode pin
 * @param S4mode S4mode pin
 * @param S5mode S5mode pin
 * @return true if success.
 */
bool Roboclaw::SetPinFunctions(uint8_t address, uint8_t S3mode, uint8_t S4mode, uint8_t S5mode)
{
    return write_n(5, address, SETPINFUNCTIONS, S3mode, S4mode, S5mode);
}

/** 
 * Rea mode settings for S3,S4 and S5. See command 74 for mode descriptions
 * @param address Controller address value from 0x80 to 0x87
 * @param S3mode S3mode pin
 * @param S4mode S4mode pin
 * @param S5mode S5mode pin
 * @return true if success.
 */
bool Roboclaw::GetPinFunctions(uint8_t address, uint8_t &S3mode, uint8_t &S4mode, uint8_t &S5mode)
{
    uint8_t val1, val2, val3;
    uint8_t trys = MAXRETRY;
    int16_t data;
    do
    {
        flush();

        crc_clear();
        write(address);
        crc_update(address);
        write(GETPINFUNCTIONS);
        crc_update(GETPINFUNCTIONS);

        data = read();
        crc_update(data);
        val1 = data;

        if (data != -1)
        {
            data = read();
            crc_update(data);
            val2 = data;
        }

        if (data != -1)
        {
            data = read();
            crc_update(data);
            val3 = data;
        }

        if (data != -1)
        {
            uint16_t ccrc;
            data = read();
            if (data != -1)
            {
                ccrc = data << 8;
                data = read();
                if (data != -1)
                {
                    ccrc |= data;
                    if (crc_get() == ccrc)
                    {
                        S3mode = val1;
                        S4mode = val2;
                        S5mode = val3;
                        return true;
                    }
                }
            }
        }
    } while (trys--);

    return false;
}

/** 
 * SetRC/Analog mode control deadband percentage in 10ths of a percent. Default value is
 * 25(2.5%). Minimum value is 0(no DeadBand), Maximum value is 250(25%).
 * @param address Controller address value from 0x80 to 0x87
 * @param Min S3mode pin
 * @param Max S4mode pin
 * @return true if success.
 */
bool Roboclaw::SetDeadBand(uint8_t address, uint8_t Min, uint8_t Max)
{
    return write_n(4, address, SETDEADBAND, Min, Max);
}

/** 
 * Rea DeadBand for RC/Analog controls
 * Read DeadBand settings in 10ths of a percent.
 * @param address Controller address value from 0x80 to 0x87
 * @param Min S3mode pin
 * @param Max S4mode pin
 * @return true if success.
 */
bool Roboclaw::GetDeadBand(uint8_t address, uint8_t &Min, uint8_t &Max)
{
    bool valid;
    uint16_t value = read2(address, GETDEADBAND, &valid);
    if (valid)
    {
        Min = value >> 8;
        Max = value;
    }
    return valid;
}

/** 
 * Rea M1 and M2 encoder counters. Quadrature encoders have a range of 0 to 4,294,967,295.
 * Absolute encoder values are converted from an analog voltage into a value from 0 to 2047 for
 * the full 2V analog range.
 * @param address Controller address value from 0x80 to 0x87
 * @param Min S3mode pin
 * @param Max S4mode pin
 * @return true if success.
 */
bool Roboclaw::ReadEncoders(uint8_t address, uint32_t &enc1, uint32_t &enc2)
{
    bool valid = read_n(2, address, GETENCODERS, &enc1, &enc2);
    return valid;
}

/** 
 * Rea ISpeeds Counters
 * Read M1 and M2 instantaneous speeds. Returns the speed in encoder counts per second for the
 * last 300th of a second for both encoder channels.
 * @param address Controller address value from 0x80 to 0x87
 * @param ispeed1 M1 instantaneous speed 
 * @paam ispeed2 M2 instantaneous speed 
 * @reurn true if success.
 */
bool Roboclaw::ReadISpeeds(uint8_t address, uint32_t &ispeed1, uint32_t &ispeed2)
{
    bool valid = read_n(2, address, GETISPEEDS, &ispeed1, &ispeed2);
    return valid;
}

/** 
 * Rest Settings to factory defaults.
 * @param address Controller address value from 0x80 to 0x87
 * @return true if success.
 */
bool Roboclaw::RestoreDefaults(uint8_t address)
{
    return write_n(2, address, RESTOREDEFAULTS);
}

/** 
 * Rea the board temperature. Value returned is in 10ths of degrees.
 * @param address Controller address value from 0x80 to 0x87
 * @param temp board temperature
 * @return true if success.
 */
bool Roboclaw::ReadTemp(uint8_t address, uint16_t &temp)
{
    bool valid;
    temp = read2(address, GETTEMP, &valid);
    return valid;
}

/** 
 * Rea the second board temperature(only on supported units). Value returned is in 10ths of
 * degrees.
 * @param address Controller address value from 0x80 to 0x87
 * @param temp board temperature
 * @return true if success.
 */
bool Roboclaw::ReadTemp2(uint8_t address, uint16_t &temp)
{
    bool valid;
    temp = read2(address, GETTEMP2, &valid);
    return valid;
}

/** 
 * Rea the current unit status.
 * @param address Controller address value from 0x80 to 0x87
 * @param valid status
 * @return true if success.
 */
uint16_t Roboclaw::ReadError(uint8_t address, bool *valid)
{
    return read2(address, GETERROR, valid);
}

/** 
 * Rea the encoder mode for both motors.
 * Encoder Mode bits
 * Bit 7 Enable/Disable RC/Analog Encoder support
 * Bit 6 Reverse Encoder Relative Direction
 * Bit 5 Reverse Motor Relative Direction
 * Bit 4-1 N/A
 * Bit 0 Quadrature(0)/Absolute(1)
 * @param address Controller address value from 0x80 to 0x87
 * @param M1mode M1 mode
 * @param M2mode M2 mode
 * @return true if success.
 */
bool Roboclaw::ReadEncoderModes(uint8_t address, uint8_t &M1mode, uint8_t &M2mode)
{
    bool valid;
    uint16_t value = read2(address, GETENCODERMODE, &valid);
    if (valid)
    {
        M1mode = value >> 8;
        M2mode = value;
    }
    return valid;
}

/** 
 * Setthe Encoder Mode for motor 1.
 * Encoder Mode bits
 * Bit 7 Enable/Disable RC/Analog Encoder support
 * Bit 6 Reverse Encoder Relative Direction
 * Bit 5 Reverse Motor Relative Direction
 * Bit 4-1 N/A
 * Bit 0 Quadrature(0)/Absolute(1)
 * @param address Controller address value from 0x80 to 0x87
 * @param mode M1 mode
 * @return true if success.
 */
bool Roboclaw::SetM1EncoderMode(uint8_t address, uint8_t mode)
{
    return write_n(3, address, SETM1ENCODERMODE, mode);
}

/** 
 * Setthe Encoder Mode for motor 2.
 * Encoder Mode bits
 * Bit 7 Enable/Disable RC/Analog Encoder support
 * Bit 6 Reverse Encoder Relative Direction
 * Bit 5 Reverse Motor Relative Direction
 * Bit 4-1 N/A
 * Bit 0 Quadrature(0)/Absolute(1)
 * @param address Controller address value from 0x80 to 0x87
 * @param mode M2 mode
 * @return true if success.
 */
bool Roboclaw::SetM2EncoderMode(uint8_t address, uint8_t mode)
{
    return write_n(3, address, SETM2ENCODERMODE, mode);
}

/** 
 * Wries all settings to non-volatile memory. Values will be loaded after each power up.
 * @param address Controller address value from 0x80 to 0x87
 * @return true if success.
 */
bool Roboclaw::WriteNVM(uint8_t address)
{
    return write_n(6, address, WRITENVM, SetDWORDval(0xE22EAB7A));
}

/** 
 * Rea all settings from non-volatile memory.
 * @param address Controller address value from 0x80 to 0x87
 * @return true if success.
 */
bool Roboclaw::ReadNVM(uint8_t address)
{
    return write_n(2, address, READNVM);
}

/** 
 * Setconfig bits for standard settings
 * @param address Controller address value from 0x80 to 0x87
 * @param config configuration bit (see http://downloads.basicmicro.com/docs/roboclaw_user_manual.pdf page 74 for available settings)
 * @return true if success.
 */
bool Roboclaw::SetConfig(uint8_t address, uint16_t config)
{
    return write_n(4, address, SETCONFIG, SetWORDval(config));
}

/** 
 * Rea config bits for standard settings 
 * @paam address Controller address value from 0x80 to 0x87
 * @param valid valid bit (see http://downloads.basicmicro.com/docs/roboclaw_user_manual.pdf page 74 for available settings)
 * @return true if success.
 */
bool Roboclaw::GetConfig(uint8_t address, uint16_t &config)
{
    bool valid;
    uint16_t value = read2(address, GETCONFIG, &valid);
    if (valid)
    {
        config = value;
    }
    return valid;
}

/** 
 * SetM1 Maximum Current Limit. Current value is in 10ma units. To calculate multiply current
 * limit by 100.
 * @param address Controller address value from 0x80 to 0x87
 * @param max max current limit
 * @return true if success.
 */
bool Roboclaw::SetM1MaxCurrent(uint8_t address, uint32_t max)
{
    return write_n(10, address, SETM1MAXCURRENT, SetDWORDval(max), SetDWORDval(0));
}

/** 
 * SetM2 Maximum Current Limit. Current value is in 10ma units. To calculate multiply current
 * limit by 100.
 * @param address Controller address value from 0x80 to 0x87
 * @param max max current limit
 * @return true if success.
 */
bool Roboclaw::SetM2MaxCurrent(uint8_t address, uint32_t max)
{
    return write_n(10, address, SETM2MAXCURRENT, SetDWORDval(max), SetDWORDval(0));
}

/** 
 * Rea M1 Maximum Current Limit. Current value is in 10ma units. To calculate divide value
 * by 100. MinCurrent is always 0.
 * @param address Controller address value from 0x80 to 0x87
 * @param max max current limit
 * @return true if success.
 */
bool Roboclaw::ReadM1MaxCurrent(uint8_t address, uint32_t &max)
{
    uint32_t tmax, dummy;
    bool valid = read_n(2, address, GETM1MAXCURRENT, &tmax, &dummy);
    if (valid)
        max = tmax;
    return valid;
}

/** 
 * Rea M2 Maximum Current Limit. Current value is in 10ma units. To calculate divide value
 * by 100. MinCurrent is always 0.
 * @param address Controller address value from 0x80 to 0x87
 * @param max max current limit
 * @return true if success.
 */
bool Roboclaw::ReadM2MaxCurrent(uint8_t address, uint32_t &max)
{
    uint32_t tmax, dummy;
    bool valid = read_n(2, address, GETM2MAXCURRENT, &tmax, &dummy);
    if (valid)
        max = tmax;
    return valid;
}

/** 
 * SetPWM Drive mode. Locked Antiphase(0) or Sign Magnitude(1).
 * @param address Controller address value from 0x80 to 0x87
 * @param mode Locked Antiphase(0) or Sign Magnitude(1)
 * @return true if success.
 */
bool Roboclaw::SetPWMMode(uint8_t address, uint8_t mode)
{
    return write_n(3, address, SETPWMMODE, mode);
}

/** 
 * Rea PWM Drive mode. Locked Antiphase(0) or Sign Magnitude(1).
 * @param address Controller address value from 0x80 to 0x87
 * @param mode Locked Antiphase(0) or Sign Magnitude(1)
 * @return true if success.
 */
bool Roboclaw::GetPWMMode(uint8_t address, uint8_t &mode)
{
    bool valid;
    uint8_t value = read1(address, GETPWMMODE, &valid);
    if (valid)
    {
        mode = value;
    }
    return valid;
}