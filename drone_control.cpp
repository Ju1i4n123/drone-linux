// drone_control.cpp - COMPLETE READY-TO-RUN DRONE FLIGHT CONTROL SYSTEM
// Compile: g++ -std=c++17 -O2 -o drone_control drone_control.cpp -lpthread -lrt
// Run: sudo ./drone_control

#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <vector>
#include <atomic>
#include <mutex>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cstring>
#include <signal.h>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <algorithm>

// Constants
constexpr float PI = 3.14159265359f;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI;
constexpr int PWM_MIN = 1000;
constexpr int PWM_MAX = 2000;
constexpr int PWM_ARM = 1100;
constexpr float OBSTACLE_THRESHOLD = 1.0f; // 1 meter

// Global shutdown flag
std::atomic<bool> g_shutdown(false);

// Sensor Data Structures
struct IMUData {
    float ax, ay, az;  // Accelerometer (m/s²)
    float gx, gy, gz;  // Gyroscope (rad/s)
    float mx, my, mz;  // Magnetometer (uT)
    float temperature; // Temperature (°C)
    float pressure;    // Pressure (Pa)
    float altitude;    // Altitude (m)
};

struct GPSData {
    double latitude = 0.0;
    double longitude = 0.0;
    float altitude = 0.0;
    float speed = 0.0;
    float course = 0.0;
    int satellites = 0;
    bool fix = false;
    uint64_t timestamp = 0;
};

struct LidarData {
    float distance = 10.0f;    // Distance in meters
    uint16_t strength = 0; // Signal strength
    bool valid = false;
};

struct DroneState {
    float roll = 0, pitch = 0, yaw = 0;     // Euler angles (rad)
    float vx = 0, vy = 0, vz = 0;          // Velocities (m/s)
    float x = 0, y = 0, z = 0;             // Position (m)
    float battery_voltage = 11.1f;
    bool armed = false;
    bool in_flight = false;
};

// PID Controller
class PIDController {
private:
    float kp, ki, kd;
    float integral = 0;
    float prev_error = 0;
    float max_integral;
    float max_output;
    
public:
    PIDController(float p, float i, float d, float max_int = 100.0f, float max_out = 1.0f)
        : kp(p), ki(i), kd(d), max_integral(max_int), max_output(max_out) {}
    
    float update(float error, float dt) {
        integral += error * dt;
        integral = std::max(-max_integral, std::min(max_integral, integral));
        
        float derivative = (error - prev_error) / dt;
        prev_error = error;
        
        float output = kp * error + ki * integral + kd * derivative;
        return std::max(-max_output, std::min(max_output, output));
    }
    
    void reset() {
        integral = 0;
        prev_error = 0;
    }
};

// I2C Communication Base Class
class I2CDevice {
protected:
    int fd;
    int address;
    bool connected = false;
    
public:
    I2CDevice(const char* device, int addr) : address(addr) {
        fd = open(device, O_RDWR);
        if (fd < 0) {
            std::cerr << "Warning: Failed to open I2C device " << device << std::endl;
            return;
        }
        if (ioctl(fd, I2C_SLAVE, address) < 0) {
            close(fd);
            fd = -1;
            std::cerr << "Warning: Failed to set I2C address 0x" << std::hex << address << std::dec << std::endl;
            return;
        }
        connected = true;
    }
    
    virtual ~I2CDevice() {
        if (fd >= 0) close(fd);
    }
    
    bool isConnected() const { return connected; }
    
    void writeReg(uint8_t reg, uint8_t value) {
        if (!connected) return;
        uint8_t buffer[2] = {reg, value};
        if (write(fd, buffer, 2) != 2) {
            std::cerr << "I2C write failed" << std::endl;
        }
    }
    
    uint8_t readReg(uint8_t reg) {
        if (!connected) return 0;
        if (write(fd, &reg, 1) != 1) {
            std::cerr << "I2C write failed" << std::endl;
            return 0;
        }
        uint8_t value = 0;
        if (read(fd, &value, 1) != 1) {
            std::cerr << "I2C read failed" << std::endl;
            return 0;
        }
        return value;
    }
    
    void readRegs(uint8_t reg, uint8_t* buffer, size_t len) {
        if (!connected) return;
        if (write(fd, &reg, 1) != 1) {
            std::cerr << "I2C write failed" << std::endl;
            return;
        }
        if (read(fd, buffer, len) != (ssize_t)len) {
            std::cerr << "I2C read failed" << std::endl;
        }
    }
};

// MPU6050 6-axis IMU Driver
class MPU6050 : public I2CDevice {
private:
    static constexpr int MPU6050_ADDR = 0x68;
    
    // Register addresses
    static constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
    static constexpr uint8_t REG_CONFIG = 0x1A;
    static constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
    static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t REG_INT_PIN_CFG = 0x37;
    static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t REG_TEMP_OUT_H = 0x41;
    static constexpr uint8_t REG_GYRO_XOUT_H = 0x43;
    static constexpr uint8_t REG_WHO_AM_I = 0x75;
    
    // Conversion factors
    float accel_scale = 4.0f * 9.81f / 32768.0f;
    float gyro_scale = 500.0f / 32768.0f * DEG_TO_RAD;
    
    // Calibration offsets
    float ax_offset = 0, ay_offset = 0, az_offset = 0;
    float gx_offset = 0, gy_offset = 0, gz_offset = 0;
    
public:
    MPU6050(const char* device) : I2CDevice(device, MPU6050_ADDR) {
        if (!connected) return;
        
        // Check device ID
        uint8_t who_am_i = readReg(REG_WHO_AM_I);
        if (who_am_i != 0x68 && who_am_i != 0x72) {
            std::cerr << "Warning: MPU6050 not found (ID: 0x" << std::hex << (int)who_am_i << ")" << std::dec << std::endl;
            connected = false;
            return;
        }
        
        // Wake up device
        writeReg(REG_PWR_MGMT_1, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Configure gyroscope (±500°/s)
        writeReg(REG_GYRO_CONFIG, 0x08);
        
        // Configure accelerometer (±4g)
        writeReg(REG_ACCEL_CONFIG, 0x08);
        
        // Configure DLPF (Digital Low Pass Filter) - 44Hz
        writeReg(REG_CONFIG, 0x03);
        
        // Enable I2C bypass for magnetometer access
        writeReg(REG_INT_PIN_CFG, 0x02);
        
        // Calibrate
        calibrate();
    }
    
    void calibrate() {
        if (!connected) return;
        
        std::cout << "Calibrating IMU... Keep drone still!" << std::endl;
        
        const int samples = 500;
        float ax_sum = 0, ay_sum = 0, az_sum = 0;
        float gx_sum = 0, gy_sum = 0, gz_sum = 0;
        
        for (int i = 0; i < samples; i++) {
            float ax, ay, az, gx, gy, gz, temp;
            readRaw(ax, ay, az, gx, gy, gz, temp);
            
            ax_sum += ax;
            ay_sum += ay;
            az_sum += az;
            gx_sum += gx;
            gy_sum += gy;
            gz_sum += gz;
            
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        
        ax_offset = ax_sum / samples;
        ay_offset = ay_sum / samples;
        az_offset = (az_sum / samples) - 9.81f;
        gx_offset = gx_sum / samples;
        gy_offset = gy_sum / samples;
        gz_offset = gz_sum / samples;
        
        std::cout << "IMU calibration complete!" << std::endl;
    }
    
    void readRaw(float& ax, float& ay, float& az, 
                 float& gx, float& gy, float& gz, float& temp) {
        if (!connected) {
            ax = ay = az = gx = gy = gz = 0;
            temp = 25.0f;
            return;
        }
        
        uint8_t buffer[14];
        readRegs(REG_ACCEL_XOUT_H, buffer, 14);
        
        int16_t ax_raw = (buffer[0] << 8) | buffer[1];
        int16_t ay_raw = (buffer[2] << 8) | buffer[3];
        int16_t az_raw = (buffer[4] << 8) | buffer[5];
        int16_t temp_raw = (buffer[6] << 8) | buffer[7];
        int16_t gx_raw = (buffer[8] << 8) | buffer[9];
        int16_t gy_raw = (buffer[10] << 8) | buffer[11];
        int16_t gz_raw = (buffer[12] << 8) | buffer[13];
        
        ax = ax_raw * accel_scale;
        ay = ay_raw * accel_scale;
        az = az_raw * accel_scale;
        gx = gx_raw * gyro_scale;
        gy = gy_raw * gyro_scale;
        gz = gz_raw * gyro_scale;
        temp = (temp_raw / 340.0f) + 36.53f;
    }
    
    void read(float& ax, float& ay, float& az, 
              float& gx, float& gy, float& gz, float& temp) {
        readRaw(ax, ay, az, gx, gy, gz, temp);
        ax -= ax_offset;
        ay -= ay_offset;
        az -= az_offset;
        gx -= gx_offset;
        gy -= gy_offset;
        gz -= gz_offset;
    }
};

// QMC5883L Magnetometer Driver
class QMC5883L : public I2CDevice {
private:
    static constexpr int QMC5883L_ADDR = 0x0D;
    static constexpr uint8_t REG_X_LSB = 0x00;
    static constexpr uint8_t REG_CONTROL = 0x09;
    static constexpr uint8_t REG_SET_RESET = 0x0B;
    
public:
    QMC5883L(const char* device) : I2CDevice(device, QMC5883L_ADDR) {
        if (!connected) return;
        
        // Configure: Continuous mode, 200Hz, 8G range, 512 OSR
        writeReg(REG_CONTROL, 0x1D);
        writeReg(REG_SET_RESET, 0x01);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    void read(float& mx, float& my, float& mz) {
        if (!connected) {
            mx = my = mz = 0;
            return;
        }
        
        uint8_t buffer[6];
        readRegs(REG_X_LSB, buffer, 6);
        
        int16_t raw_x = (buffer[1] << 8) | buffer[0];
        int16_t raw_y = (buffer[3] << 8) | buffer[2];
        int16_t raw_z = (buffer[5] << 8) | buffer[4];
        
        mx = raw_x * 0.244f;  // to uT
        my = raw_y * 0.244f;
        mz = raw_z * 0.244f;
    }
};

// DPS310 Barometer Driver
class DPS310 : public I2CDevice {
private:
    static constexpr int DPS310_ADDR = 0x77;
    static constexpr uint8_t REG_PRS_CFG = 0x06;
    static constexpr uint8_t REG_TMP_CFG = 0x07;
    static constexpr uint8_t REG_MEAS_CFG = 0x08;
    static constexpr uint8_t REG_CFG = 0x09;
    static constexpr uint8_t REG_PSR_B2 = 0x00;
    static constexpr uint8_t REG_TMP_B2 = 0x03;
    
    struct CalibCoeffs {
        int16_t c0 = 0, c1 = 0;
        int32_t c00 = 0, c10 = 0, c01 = 0, c11 = 0, c20 = 0, c21 = 0, c30 = 0;
    } coeffs;
    
    float kT = 524288.0f;
    float kP = 253952.0f;
    
    void readCalibrationCoeffs() {
        if (!connected) return;
        
        uint8_t buffer[18];
        readRegs(0x10, buffer, 18);
        
        coeffs.c0 = ((uint16_t)buffer[0] << 4) | ((buffer[1] >> 4) & 0x0F);
        if (coeffs.c0 & 0x800) coeffs.c0 |= 0xF000;
        
        coeffs.c1 = ((uint16_t)(buffer[1] & 0x0F) << 8) | buffer[2];
        if (coeffs.c1 & 0x800) coeffs.c1 |= 0xF000;
        
        coeffs.c00 = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) | (buffer[5] >> 4);
        if (coeffs.c00 & 0x80000) coeffs.c00 |= 0xFFF00000;
        
        coeffs.c10 = ((uint32_t)(buffer[5] & 0x0F) << 16) | ((uint32_t)buffer[6] << 8) | buffer[7];
        if (coeffs.c10 & 0x80000) coeffs.c10 |= 0xFFF00000;
        
        coeffs.c01 = ((uint16_t)buffer[8] << 8) | buffer[9];
        if (coeffs.c01 & 0x8000) coeffs.c01 |= 0xFFFF0000;
        
        coeffs.c11 = ((uint16_t)buffer[10] << 8) | buffer[11];
        if (coeffs.c11 & 0x8000) coeffs.c11 |= 0xFFFF0000;
        
        coeffs.c20 = ((uint16_t)buffer[12] << 8) | buffer[13];
        if (coeffs.c20 & 0x8000) coeffs.c20 |= 0xFFFF0000;
        
        coeffs.c21 = ((uint16_t)buffer[14] << 8) | buffer[15];
        if (coeffs.c21 & 0x8000) coeffs.c21 |= 0xFFFF0000;
        
        coeffs.c30 = ((uint16_t)buffer[16] << 8) | buffer[17];
        if (coeffs.c30 & 0x8000) coeffs.c30 |= 0xFFFF0000;
    }
    
public:
    DPS310(const char* device) : I2CDevice(device, DPS310_ADDR) {
        if (!connected) return;
        
        readCalibrationCoeffs();
        
        writeReg(REG_PRS_CFG, 0x34);  // 32Hz, 16x oversampling
        writeReg(REG_TMP_CFG, 0xB4);  // 32Hz, 16x oversampling
        writeReg(REG_CFG, 0x00);      // No interrupts
        writeReg(REG_MEAS_CFG, 0x07); // Continuous pressure and temperature
    }
    
    void read(float& pressure, float& temperature, float& altitude) {
        if (!connected) {
            pressure = 101325.0f;
            temperature = 25.0f;
            altitude = 0.0f;
            return;
        }
        
        uint8_t prs_buffer[3];
        readRegs(REG_PSR_B2, prs_buffer, 3);
        int32_t raw_prs = ((uint32_t)prs_buffer[0] << 16) | ((uint32_t)prs_buffer[1] << 8) | prs_buffer[2];
        if (raw_prs & 0x800000) raw_prs |= 0xFF000000;
        
        uint8_t tmp_buffer[3];
        readRegs(REG_TMP_B2, tmp_buffer, 3);
        int32_t raw_tmp = ((uint32_t)tmp_buffer[0] << 16) | ((uint32_t)tmp_buffer[1] << 8) | tmp_buffer[2];
        if (raw_tmp & 0x800000) raw_tmp |= 0xFF000000;
        
        float traw_sc = raw_tmp / kT;
        float praw_sc = raw_prs / kP;
        
        temperature = coeffs.c0 * 0.5f + coeffs.c1 * traw_sc;
        
        pressure = coeffs.c00 + praw_sc * (coeffs.c10 + praw_sc * (coeffs.c20 + praw_sc * coeffs.c30)) +
                   traw_sc * coeffs.c01 + traw_sc * praw_sc * (coeffs.c11 + praw_sc * coeffs.c21);
        
        float sea_level_pressure = 101325.0f;
        altitude = 44330.0f * (1.0f - std::pow(pressure / sea_level_pressure, 0.1903f));
    }
};

// TFmini Plus Lidar Driver
class TFminiPlus {
private:
    int fd = -1;
    static constexpr uint8_t FRAME_HEADER = 0x59;
    
public:
    TFminiPlus(const char* device) {
        fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0) {
            std::cerr << "Warning: Failed to open LIDAR UART " << device << std::endl;
            return;
        }
        
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        if (tcgetattr(fd, &tty) != 0) {
            close(fd);
            fd = -1;
            std::cerr << "Warning: Failed to get LIDAR UART attributes" << std::endl;
            return;
        }
        
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 5;
        
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        
        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            close(fd);
            fd = -1;
            std::cerr << "Warning: Failed to set LIDAR UART attributes" << std::endl;
        }
    }
    
    ~TFminiPlus() {
        if (fd >= 0) close(fd);
    }
    
    bool isConnected() const { return fd >= 0; }
    
    bool read(LidarData& data) {
        if (fd < 0) {
            data.distance = 10.0f;  // Safe default
            data.valid = false;
            return false;
        }
        
        uint8_t buffer[9];
        int n = ::read(fd, buffer, 9);
        
        if (n == 9 && buffer[0] == FRAME_HEADER && buffer[1] == FRAME_HEADER) {
            uint16_t distance = buffer[2] | (buffer[3] << 8);
            uint16_t strength = buffer[4] | (buffer[5] << 8);
            
            uint8_t checksum = 0;
            for (int i = 0; i < 8; i++) {
                checksum += buffer[i];
            }
            
            if (checksum == buffer[8]) {
                data.distance = distance / 100.0f;  // Convert to meters
                data.strength = strength;
                data.valid = true;
                return true;
            }
        }
        
        data.valid = false;
        return false;
    }
};

// GPS NMEA Parser
class GPSParser {
private:
    int fd = -1;
    std::string buffer;
    
    double parseLatLon(const std::string& value, const std::string& dir) {
        if (value.empty()) return 0.0;
        
        double deg = std::stod(value.substr(0, value.find('.') - 2));
        double min = std::stod(value.substr(value.find('.') - 2));
        double decimal = deg + min / 60.0;
        
        if (dir == "S" || dir == "W") decimal = -decimal;
        return decimal;
    }
    
    bool parseGGA(const std::string& sentence, GPSData& data) {
        std::vector<std::string> fields;
        size_t start = 0;
        size_t end = sentence.find(',');
        
        while (end != std::string::npos) {
            fields.push_back(sentence.substr(start, end - start));
            start = end + 1;
            end = sentence.find(',', start);
        }
        fields.push_back(sentence.substr(start));
        
        if (fields.size() < 15) return false;
        
        try {
            data.fix = std::stoi(fields[6]) > 0;
            if (data.fix) {
                data.latitude = parseLatLon(fields[2], fields[3]);
                data.longitude = parseLatLon(fields[4], fields[5]);
                data.satellites = std::stoi(fields[7]);
                if (!fields[9].empty()) {
                    data.altitude = std::stof(fields[9]);
                }
            }
            return true;
        } catch (...) {
            return false;
        }
    }
    
public:
    GPSParser(const char* device) {
        fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0) {
            std::cerr << "Warning: Failed to open GPS UART " << device << std::endl;
            return;
        }
        
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        tcgetattr(fd, &tty);
        
        cfsetospeed(&tty, B9600);
        cfsetispeed(&tty, B9600);
        
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;
        
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        
        tcsetattr(fd, TCSANOW, &tty);
    }
    
    ~GPSParser() {
        if (fd >= 0) close(fd);
    }
    
    bool isConnected() const { return fd >= 0; }
    
    bool update(GPSData& data) {
        if (fd < 0) return false;
        
        char c;
        while (::read(fd, &c, 1) == 1) {
            if (c == '\n') {
                if (buffer.find("$GPGGA") == 0) {
                    parseGGA(buffer, data);
                }
                buffer.clear();
            } else if (c != '\r') {
                buffer += c;
            }
        }
        return data.fix;
    }
};

// PWM Output for motor control
class PWMOutput {
private:
    std::string pwm_path = "/sys/class/pwm/pwmchip0/";
    int period_ns = 20000000;  // 20ms = 50Hz for servo/ESC
    bool initialized = false;
    
    void exportPWM(int channel) {
        std::ofstream export_file(pwm_path + "export");
        if (export_file.is_open()) {
            export_file << channel;
            export_file.close();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    void setPeriod(int channel) {
        std::ofstream period_file(pwm_path + "pwm" + std::to_string(channel) + "/period");
        if (period_file.is_open()) {
            period_file << period_ns;
            period_file.close();
        }
    }
    
    void enable(int channel) {
        std::ofstream enable_file(pwm_path + "pwm" + std::to_string(channel) + "/enable");
        if (enable_file.is_open()) {
            enable_file << "1";
            enable_file.close();
        }
    }
    
    void writePWM(int motor, int microseconds) {
        if (motor < 0 || motor > 3 || !initialized) return;
        
        int duty_ns = microseconds * 1000;  // Convert to nanoseconds
        duty_ns = std::max(1000000, std::min(2000000, duty_ns));  // Constrain
        
        std::ofstream duty_file(pwm_path + "pwm" + std::to_string(motor) + "/duty_cycle");
        if (duty_file.is_open()) {
            duty_file << duty_ns;
            duty_file.close();
        }
    }
    
public:
    PWMOutput() {
        // Try to initialize PWM - if it fails, we'll run in simulation mode
        try {
            for (int i = 0; i < 4; i++) {
                exportPWM(i);
                setPeriod(i);
                enable(i);
            }
            initialized = true;
            std::cout << "PWM outputs initialized" << std::endl;
        } catch (...) {
            std::cerr << "Warning: PWM initialization failed - running in simulation mode" << std::endl;
            initialized = false;
        }
    }
    
    void setMotors(int m1, int m2, int m3, int m4) {
        if (initialized) {
            writePWM(0, m1);
            writePWM(1, m2);
            writePWM(2, m3);
            writePWM(3, m4);
        } else {
            // Simulation mode - just print values
            static int counter = 0;
            if (++counter % 20 == 0) {  // Print every 20th update
                std::cout << "Motors: " << m1 << " " << m2 << " " << m3 << " " << m4 << std::endl;
            }
        }
    }
    
    void arm() {
        std::cout << "Arming motors..." << std::endl;
        for (int i = 0; i < 4; i++) {
            setMotors(PWM_ARM, PWM_ARM, PWM_ARM, PWM_ARM);
        }
    }
    
    void disarm() {
        std::cout << "Disarming motors..." << std::endl;
        setMotors(PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN);
    }
};

// Mahony AHRS for sensor fusion
class MahonyAHRS {
private:
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
    static constexpr float twoKp = 2.0f * 0.5f;
    static constexpr float twoKi = 2.0f * 0.0f;
    
public:
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
        float recipNorm;
        float halfvx, halfvy, halfvz;
        float halfex, halfey, halfez;
        float qa, qb, qc;
        
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
            recipNorm = 1.0f / std::sqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;
            
            halfvx = q1 * q3 - q0 * q2;
            halfvy = q0 * q1 + q2 * q3;
            halfvz = q0 * q0 - 0.5f + q3 * q3;
            
            halfex = (ay * halfvz - az * halfvy);
            halfey = (az * halfvx - ax * halfvz);
            halfez = (ax * halfvy - ay * halfvx);
            
            if(twoKi > 0.0f) {
                integralFBx += twoKi * halfex * dt;
                integralFBy += twoKi * halfey * dt;
                integralFBz += twoKi * halfez * dt;
                gx += integralFBx;
                gy += integralFBy;
                gz += integralFBz;
            } else {
                integralFBx = 0.0f;
                integralFBy = 0.0f;
                integralFBz = 0.0f;
            }
            
            gx += twoKp * halfex;
            gy += twoKp * halfey;
            gz += twoKp * halfez;
        }
        
        gx *= (0.5f * dt);
        gy *= (0.5f * dt);
        gz *= (0.5f * dt);
        qa = q0;
        qb = q1;
        qc = q2;
        q0 += (-qb * gx - qc * gy - q3 * gz);
        q1 += (qa * gx + qc * gz - q3 * gy);
        q2 += (qa * gy - qb * gz + q3 * gx);
        q3 += (qa * gz + qb * gy - qc * gx);
        
        recipNorm = 1.0f / std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }
    
    void getEulerAngles(float& roll, float& pitch, float& yaw) {
        roll = std::atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
        pitch = -std::asin(2.0f * (q1 * q3 - q0 * q2));
        yaw = std::atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
    }
};

// Main Flight Controller Class
class FlightController {
private:
    // Sensors
    std::unique_ptr<MPU6050> mpu6050;
    std::unique_ptr<QMC5883L> magnetometer;
    std::unique_ptr<DPS310> barometer;
    std::unique_ptr<TFminiPlus> lidar;
    std::unique_ptr<GPSParser> gps;
    std::unique_ptr<PWMOutput> motors;
    std::unique_ptr<MahonyAHRS> ahrs;
    
    // State
    DroneState state;
    IMUData imu_data;
    GPSData gps_data;
    LidarData lidar_data;
    
    // Control
    PIDController roll_pid{4.5f, 1.0f, 0.035f, 50.0f, 500.0f};
    PIDController pitch_pid{4.5f, 1.0f, 0.035f, 50.0f, 500.0f};
    PIDController yaw_pid{4.5f, 1.0f, 0.0f, 50.0f, 500.0f};
    PIDController altitude_pid{5.0f, 1.0f, 2.0f, 100.0f, 500.0f};
    
    // Enhanced obstacle avoidance with scanning
    float obstacle_distance = 10.0f;
    bool obstacle_detected = false;
    float obstacle_map[360] = {10.0f};  // 360 degree obstacle map
    int scan_angle = 0;
    bool scanning_enabled = false;
    float last_scan_time = 0;
    float min_obstacle_distance = 10.0f;
    int min_obstacle_angle = 0;
    
    // Threading
    std::thread sensor_thread;
    std::thread control_thread;
    std::mutex state_mutex;
    
    // Command interface
    float target_roll = 0.0f;
    float target_pitch = 0.0f;
    float target_yaw_rate = 0.0f;
    float target_altitude = 0.0f;
    float target_throttle = 0.0f;
    
    // Sensor availability flags
    bool has_imu = false;
    bool has_mag = false;
    bool has_baro = false;
    bool has_lidar = false;
    bool has_gps = false;
    
    void updateIMU() {
        if (mpu6050 && mpu6050->isConnected()) {
            float temp;
            mpu6050->read(imu_data.ax, imu_data.ay, imu_data.az,
                         imu_data.gx, imu_data.gy, imu_data.gz, temp);
            imu_data.temperature = temp;
        } else {
            // Simulate stable drone for testing
            imu_data.ax = 0.1f * std::sin(std::chrono::steady_clock::now().time_since_epoch().count() / 1e9);
            imu_data.ay = 0.1f * std::cos(std::chrono::steady_clock::now().time_since_epoch().count() / 1e9);
            imu_data.az = 9.81f;
            imu_data.gx = 0;
            imu_data.gy = 0;
            imu_data.gz = 0;
        }
    }
    
    void updateAttitude(float dt) {
        if (has_imu) {
            ahrs->updateIMU(imu_data.gx, imu_data.gy, imu_data.gz,
                           imu_data.ax, imu_data.ay, imu_data.az, dt);
            ahrs->getEulerAngles(state.roll, state.pitch, state.yaw);
        } else {
            // Simple integration for testing
            state.roll += imu_data.gx * dt;
            state.pitch += imu_data.gy * dt;
            state.yaw += imu_data.gz * dt;
            
            // Limit angles
            state.roll = std::max(-PI/2, std::min(PI/2, state.roll));
            state.pitch = std::max(-PI/2, std::min(PI/2, state.pitch));
        }
    }
    
    void sensorLoop() {
        auto last_time = std::chrono::steady_clock::now();
        
        while (!g_shutdown) {
            auto current_time = std::chrono::steady_clock::now();
            float dt = std::chrono::duration<float>(current_time - last_time).count();
            last_time = current_time;
            
            try {
                // IMU update
                updateIMU();
                
                // Magnetometer
                if (magnetometer && magnetometer->isConnected()) {
                    magnetometer->read(imu_data.mx, imu_data.my, imu_data.mz);
                }
                
                // Barometer
                if (barometer && barometer->isConnected()) {
                    barometer->read(imu_data.pressure, imu_data.temperature, imu_data.altitude);
                }
                
                // Lidar with scanning
                if (lidar && lidar->isConnected()) {
                    lidar->read(lidar_data);
                    if (lidar_data.valid) {
                        // Update obstacle map at current yaw angle
                        int map_index = ((int)(state.yaw * RAD_TO_DEG + 360)) % 360;
                        obstacle_map[map_index] = lidar_data.distance;
                        
                        // If scanning mode is enabled, rotate slowly
                        if (scanning_enabled && state.armed && !state.in_flight) {
                            auto current_time = std::chrono::steady_clock::now().time_since_epoch().count() / 1e9;
                            if (current_time - last_scan_time > 0.1f) {  // Scan every 100ms
                                target_yaw_rate = 30.0f * DEG_TO_RAD;  // 30 deg/s scan rate
                                last_scan_time = current_time;
                            }
                        }
                        
                        // Find minimum obstacle distance in front 90 degree arc
                        min_obstacle_distance = 10.0f;
                        for (int i = -45; i <= 45; i++) {
                            int idx = (map_index + i + 360) % 360;
                            if (obstacle_map[idx] < min_obstacle_distance) {
                                min_obstacle_distance = obstacle_map[idx];
                                min_obstacle_angle = i;
                            }
                        }
                        
                        obstacle_distance = min_obstacle_distance;
                        obstacle_detected = (obstacle_distance < OBSTACLE_THRESHOLD);
                        
                        // Decay old measurements
                        for (int i = 0; i < 360; i++) {
                            if (i != map_index && obstacle_map[i] < 10.0f) {
                                obstacle_map[i] += 0.01f;  // Slowly forget old measurements
                                if (obstacle_map[i] > 10.0f) obstacle_map[i] = 10.0f;
                            }
                        }
                    }
                } else {
                    // Safe default
                    obstacle_distance = 10.0f;
                    obstacle_detected = false;
                }
                
                // GPS
                if (gps && gps->isConnected()) {
                    gps->update(gps_data);
                }
                
                // Update attitude
                std::lock_guard<std::mutex> lock(state_mutex);
                updateAttitude(dt);
                
                if (has_baro) {
                    state.z = imu_data.altitude;
                }
                
            } catch (const std::exception& e) {
                std::cerr << "Sensor error: " << e.what() << std::endl;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    void controlLoop() {
        auto last_time = std::chrono::steady_clock::now();
        
        while (!g_shutdown) {
            auto current_time = std::chrono::steady_clock::now();
            float dt = std::chrono::duration<float>(current_time - last_time).count();
            last_time = current_time;
            
            std::lock_guard<std::mutex> lock(state_mutex);
            
            if (state.armed) {
                // ENHANCED OBSTACLE AVOIDANCE WITH DIRECTIONAL RESPONSE
                float pitch_override = 0.0f;
                float roll_override = 0.0f;
                
                if (obstacle_detected) {
                    // Emergency response based on obstacle direction
                    if (min_obstacle_angle < -20) {
                        // Obstacle to the left - move right
                        roll_override = 10.0f * DEG_TO_RAD;
                        std::cout << "\n*** OBSTACLE LEFT! Distance: " << obstacle_distance 
                                 << "m - Moving RIGHT! ***\n" << std::endl;
                    } else if (min_obstacle_angle > 20) {
                        // Obstacle to the right - move left
                        roll_override = -10.0f * DEG_TO_RAD;
                        std::cout << "\n*** OBSTACLE RIGHT! Distance: " << obstacle_distance 
                                 << "m - Moving LEFT! ***\n" << std::endl;
                    } else {
                        // Obstacle straight ahead - move back
                        pitch_override = -15.0f * DEG_TO_RAD;
                        std::cout << "\n*** OBSTACLE AHEAD! Distance: " << obstacle_distance 
                                 << "m - Moving BACK! ***\n" << std::endl;
                    }
                }
                
                // Check if we should avoid based on movement direction
                bool should_avoid = false;
                if (target_pitch > 0 && min_obstacle_angle >= -45 && min_obstacle_angle <= 45) {
                    should_avoid = true;  // Moving forward into obstacle
                } else if (target_roll > 0 && min_obstacle_angle > 0) {
                    should_avoid = true;  // Moving right into obstacle
                } else if (target_roll < 0 && min_obstacle_angle < 0) {
                    should_avoid = true;  // Moving left into obstacle
                }
                
                // Apply avoidance if needed
                if (obstacle_detected && should_avoid) {
                    target_pitch += pitch_override;
                    target_roll += roll_override;
                }
                
                // Calculate control errors
                float roll_error = target_roll - state.roll;
                float pitch_error = target_pitch - state.pitch;
                float yaw_rate_error = target_yaw_rate - imu_data.gz;
                float altitude_error = target_altitude - state.z;
                
                // PID control
                float roll_output = roll_pid.update(roll_error, dt);
                float pitch_output = pitch_pid.update(pitch_error, dt);
                float yaw_output = yaw_pid.update(yaw_rate_error, dt);
                float altitude_output = altitude_pid.update(altitude_error, dt);
                
                // Mix outputs for quadcopter X configuration
                float base_throttle = target_throttle + altitude_output;
                
                // OBSTACLE AVOIDANCE THROTTLE REDUCTION
                if (obstacle_detected) {
                    base_throttle *= 0.7f;  // Reduce throttle when avoiding
                }
                
                int m1 = base_throttle + roll_output - pitch_output + yaw_output;
                int m2 = base_throttle - roll_output - pitch_output - yaw_output;
                int m3 = base_throttle - roll_output + pitch_output + yaw_output;
                int m4 = base_throttle + roll_output + pitch_output - yaw_output;
                
                // Constrain motor outputs
                m1 = std::max(PWM_MIN, std::min(PWM_MAX, m1));
                m2 = std::max(PWM_MIN, std::min(PWM_MAX, m2));
                m3 = std::max(PWM_MIN, std::min(PWM_MAX, m3));
                m4 = std::max(PWM_MIN, std::min(PWM_MAX, m4));
                
                // Set motor outputs
                motors->setMotors(m1, m2, m3, m4);
                
                state.in_flight = (base_throttle > 1200);
            } else {
                motors->disarm();
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); // 200Hz control loop
        }
    }
    
public:
    FlightController() : state{} {
        std::cout << "Initializing Flight Controller..." << std::endl;
        
        try {
            // Initialize sensors (continue even if some fail)
            try {
                mpu6050 = std::make_unique<MPU6050>("/dev/i2c-1");
                has_imu = mpu6050->isConnected();
                if (has_imu) std::cout << "✓ MPU6050 IMU initialized" << std::endl;
                else std::cout << "✗ MPU6050 IMU not found" << std::endl;
            } catch (...) {
                std::cout << "✗ MPU6050 IMU initialization failed" << std::endl;
            }
            
            try {
                magnetometer = std::make_unique<QMC5883L>("/dev/i2c-1");
                has_mag = magnetometer->isConnected();
                if (has_mag) std::cout << "✓ QMC5883L Magnetometer initialized" << std::endl;
                else std::cout << "✗ QMC5883L Magnetometer not found" << std::endl;
            } catch (...) {
                std::cout << "✗ QMC5883L Magnetometer initialization failed" << std::endl;
            }
            
            try {
                barometer = std::make_unique<DPS310>("/dev/i2c-1");
                has_baro = barometer->isConnected();
                if (has_baro) std::cout << "✓ DPS310 Barometer initialized" << std::endl;
                else std::cout << "✗ DPS310 Barometer not found" << std::endl;
            } catch (...) {
                std::cout << "✗ DPS310 Barometer initialization failed" << std::endl;
            }
            
            try {
                lidar = std::make_unique<TFminiPlus>("/dev/ttyUSB0");
                has_lidar = lidar->isConnected();
                if (has_lidar) std::cout << "✓ TFmini Plus Lidar initialized" << std::endl;
                else std::cout << "✗ TFmini Plus Lidar not found on /dev/ttyUSB0" << std::endl;
            } catch (...) {
                std::cout << "✗ TFmini Plus Lidar initialization failed" << std::endl;
            }
            
            try {
                gps = std::make_unique<GPSParser>("/dev/ttyUSB1");
                has_gps = gps->isConnected();
                if (has_gps) std::cout << "✓ GPS initialized" << std::endl;
                else std::cout << "✗ GPS not found on /dev/ttyUSB1" << std::endl;
            } catch (...) {
                std::cout << "✗ GPS initialization failed" << std::endl;
            }
            
            // Always initialize motors and AHRS
            motors = std::make_unique<PWMOutput>();
            ahrs = std::make_unique<MahonyAHRS>();
            
            std::cout << "\nStarting sensor and control threads..." << std::endl;
            
            // Start threads
            sensor_thread = std::thread(&FlightController::sensorLoop, this);
            control_thread = std::thread(&FlightController::controlLoop, this);
            
            std::cout << "Flight Controller Ready!" << std::endl;
            
            if (!has_imu) {
                std::cout << "\nWARNING: No IMU detected - flying will be unstable!" << std::endl;
                std::cout << "Please connect an MPU6050 to I2C address 0x68" << std::endl;
            }
            
        } catch (const std::exception& e) {
            std::cerr << "Initialization error: " << e.what() << std::endl;
            throw;
        }
    }
    
    ~FlightController() {
        g_shutdown = true;
        if (sensor_thread.joinable()) sensor_thread.join();
        if (control_thread.joinable()) control_thread.join();
    }
    
    // Flight commands
    void arm() {
        std::lock_guard<std::mutex> lock(state_mutex);
        if (!state.armed && std::abs(state.roll) < 0.1f && std::abs(state.pitch) < 0.1f) {
            if (!has_imu) {
                std::cout << "WARNING: Arming without IMU - flight will be VERY unstable!" << std::endl;
                std::cout << "Continue? (y/n): ";
                std::string response;
                std::cin >> response;
                if (response != "y" && response != "Y") {
                    std::cout << "Arming cancelled" << std::endl;
                    return;
                }
            }
            
            state.armed = true;
            motors->arm();
            roll_pid.reset();
            pitch_pid.reset();
            yaw_pid.reset();
            altitude_pid.reset();
            std::cout << "*** ARMED ***" << std::endl;
            
            if (has_lidar) {
                std::cout << "Obstacle avoidance ACTIVE - threshold: " << OBSTACLE_THRESHOLD << "m" << std::endl;
                std::cout << "Type 'scan' to enable 360° environment scanning" << std::endl;
            } else {
                std::cout << "WARNING: No LIDAR - obstacle avoidance DISABLED!" << std::endl;
            }
        } else {
            std::cout << "Cannot arm - drone not level or already armed" << std::endl;
        }
    }
    
    void disarm() {
        std::lock_guard<std::mutex> lock(state_mutex);
        state.armed = false;
        state.in_flight = false;
        target_throttle = 0;
        std::cout << "*** DISARMED ***" << std::endl;
    }
    
    void takeoff(float altitude) {
        if (!state.armed) {
            std::cout << "Cannot takeoff - not armed!" << std::endl;
            return;
        }
        
        target_altitude = altitude;
        target_throttle = 1500;  // Mid throttle for hover
        std::cout << "Taking off to " << altitude << "m" << std::endl;
    }
    
    void land() {
        target_altitude = 0;
        target_throttle = 1200;  // Slow descent
        std::cout << "Landing..." << std::endl;
        
        // Auto-disarm when landed
        std::thread([this]() {
            while (state.z > 0.1f && state.armed) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (state.armed && state.z <= 0.1f) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                disarm();
            }
        }).detach();
    }
    
    void move(float forward_m, float right_m, float up_m) {
        // Check obstacle before allowing forward movement
        if (forward_m > 0 && obstacle_detected) {
            std::cout << "WARNING: Obstacle detected at " << obstacle_distance 
                     << "m - forward movement blocked!" << std::endl;
            return;
        }
        
        // Convert to roll/pitch commands
        target_pitch = forward_m * 0.1f;  // Scale to radians
        target_roll = right_m * 0.1f;
        target_altitude += up_m;
        
        // Constrain
        target_pitch = std::max(-0.5f, std::min(0.5f, target_pitch));
        target_roll = std::max(-0.5f, std::min(0.5f, target_roll));
        target_altitude = std::max(0.0f, std::min(100.0f, target_altitude));
        
        std::cout << "Moving: fwd=" << forward_m << "m right=" << right_m 
                 << "m up=" << up_m << "m" << std::endl;
    }
    
    void rotate(float yaw_rate_deg) {
        target_yaw_rate = yaw_rate_deg * DEG_TO_RAD;
        std::cout << "Rotating at " << yaw_rate_deg << " deg/s" << std::endl;
    }
    
    void hover() {
        target_roll = 0;
        target_pitch = 0;
        target_yaw_rate = 0;
        std::cout << "Hovering" << std::endl;
    }
    
    void enableScanning(bool enable) {
        scanning_enabled = enable;
        if (enable) {
            std::cout << "Obstacle scanning ENABLED - drone will rotate to map surroundings" << std::endl;
        } else {
            std::cout << "Obstacle scanning DISABLED" << std::endl;
            target_yaw_rate = 0;
        }
    }
    
    void printObstacleMap() {
        std::cout << "\n=== Obstacle Map (360°) ===" << std::endl;
        std::cout << "Direction: N=0° E=90° S=180° W=270°" << std::endl;
        
        // Print compass rose
        for (int angle = 0; angle < 360; angle += 30) {
            float dist = obstacle_map[angle];
            std::string indicator = (dist < OBSTACLE_THRESHOLD) ? "!" : 
                                   (dist < 2.0f) ? "*" : 
                                   (dist < 5.0f) ? "." : " ";
            std::cout << angle << "°: " << std::fixed << std::setprecision(1) 
                     << dist << "m " << indicator << "  ";
            if ((angle + 30) % 90 == 0) std::cout << std::endl;
        }
        
        std::cout << "\nClosest obstacle: " << min_obstacle_distance << "m at " 
                 << min_obstacle_angle << "° relative" << std::endl;
    }
    
    void clearObstacleMap() {
        for (int i = 0; i < 360; i++) {
            obstacle_map[i] = 10.0f;
        }
        std::cout << "Obstacle map cleared" << std::endl;
    }
    
    void flyToGPS(double lat, double lon, float alt) {
        if (!has_gps || !gps_data.fix) {
            std::cout << "No GPS fix available!" << std::endl;
            return;
        }
        
        double dlat = lat - gps_data.latitude;
        double dlon = lon - gps_data.longitude;
        
        float distance_north = dlat * 111111.0f;
        float distance_east = dlon * 111111.0f * std::cos(gps_data.latitude * DEG_TO_RAD);
        
        float bearing = std::atan2(distance_east, distance_north);
        float heading_error = bearing - state.yaw;
        
        if (std::sqrt(distance_north*distance_north + distance_east*distance_east) > 2.0f) {
            rotate(heading_error * RAD_TO_DEG);
            move(2.0f, 0, 0);
        } else {
            hover();
        }
        
        target_altitude = alt;
        std::cout << "Flying to GPS: " << lat << ", " << lon << " at " << alt << "m" << std::endl;
    }
    
    void printStatus() {
        std::lock_guard<std::mutex> lock(state_mutex);
        
        std::cout << "\n=== Drone Status ===" << std::endl;
        std::cout << "Armed: " << (state.armed ? "Yes" : "No") << std::endl;
        std::cout << "In Flight: " << (state.in_flight ? "Yes" : "No") << std::endl;
        
        if (has_imu) {
            std::cout << "Attitude: Roll=" << state.roll * RAD_TO_DEG 
                     << "° Pitch=" << state.pitch * RAD_TO_DEG 
                     << "° Yaw=" << state.yaw * RAD_TO_DEG << "°" << std::endl;
        } else {
            std::cout << "Attitude: NO IMU CONNECTED" << std::endl;
        }
        
        if (has_baro) {
            std::cout << "Altitude: " << state.z << "m (Baro: " << imu_data.altitude << "m)" << std::endl;
            std::cout << "Pressure: " << imu_data.pressure << "Pa" << std::endl;
        }
        
        std::cout << "Temperature: " << imu_data.temperature << "°C" << std::endl;
        
        if (has_gps) {
            std::cout << "GPS: " << (gps_data.fix ? "Fixed" : "No Fix") 
                     << " Sats=" << gps_data.satellites << std::endl;
            if (gps_data.fix) {
                std::cout << "  Position: " << std::fixed << std::setprecision(6) 
                         << gps_data.latitude << ", " << gps_data.longitude << std::endl;
            }
        }
        
        if (has_lidar) {
            std::cout << "Lidar: " << min_obstacle_distance << "m @ " << min_obstacle_angle << "°" 
                     << (obstacle_detected ? " [*** OBSTACLE! ***]" : "") << std::endl;
            std::cout << "Scanning: " << (scanning_enabled ? "ACTIVE" : "Disabled") << std::endl;
        } else {
            std::cout << "Lidar: NOT CONNECTED" << std::endl;
        }
        
        std::cout << "Battery: " << state.battery_voltage << "V" << std::endl;
    }
};

// Command interpreter
class CommandInterface {
private:
    FlightController& fc;
    bool running = true;
    
    void printHelp() {
        std::cout << "\n=== Drone Control Commands ===" << std::endl;
        std::cout << "arm        - Arm motors" << std::endl;
        std::cout << "disarm     - Disarm motors" << std::endl;
        std::cout << "takeoff H  - Take off to H meters" << std::endl;
        std::cout << "land       - Land drone" << std::endl;
        std::cout << "move F R U - Move Forward/Right/Up (meters)" << std::endl;
        std::cout << "rotate D   - Rotate D degrees/sec" << std::endl;
        std::cout << "hover      - Hold position" << std::endl;
        std::cout << "goto LAT LON ALT - Fly to GPS coordinate" << std::endl;
        std::cout << "status     - Show drone status" << std::endl;
        std::cout << "help       - Show this help" << std::endl;
        std::cout << "quit       - Exit program" << std::endl;
        std::cout << "\nOBSTACLE AVOIDANCE is automatic when armed!" << std::endl;
    }
    
public:
    CommandInterface(FlightController& controller) : fc(controller) {}
    
    void run() {
        printHelp();
        
        std::string line;
        while (running && std::getline(std::cin, line)) {
            std::istringstream iss(line);
            std::string cmd;
            iss >> cmd;
            
            if (cmd == "arm") {
                fc.arm();
            } else if (cmd == "disarm") {
                fc.disarm();
            } else if (cmd == "takeoff") {
                float height;
                if (iss >> height) {
                    fc.takeoff(height);
                } else {
                    std::cout << "Usage: takeoff HEIGHT" << std::endl;
                }
            } else if (cmd == "land") {
                fc.land();
            } else if (cmd == "move") {
                float fwd, right, up;
                if (iss >> fwd >> right >> up) {
                    fc.move(fwd, right, up);
                } else {
                    std::cout << "Usage: move FORWARD RIGHT UP" << std::endl;
                }
            } else if (cmd == "rotate") {
                float deg;
                if (iss >> deg) {
                    fc.rotate(deg);
                } else {
                    std::cout << "Usage: rotate DEGREES_PER_SEC" << std::endl;
                }
            } else if (cmd == "hover") {
                fc.hover();
            } else if (cmd == "scan") {
                fc.enableScanning(true);
            } else if (cmd == "stopscan") {
                fc.enableScanning(false);
            } else if (cmd == "map") {
                fc.printObstacleMap();
            } else if (cmd == "clearmap") {
                fc.clearObstacleMap();
            } else if (cmd == "goto") {
                double lat, lon;
                float alt;
                if (iss >> lat >> lon >> alt) {
                    fc.flyToGPS(lat, lon, alt);
                } else {
                    std::cout << "Usage: goto LATITUDE LONGITUDE ALTITUDE" << std::endl;
                }
            } else if (cmd == "status") {
                fc.printStatus();
            } else if (cmd == "help") {
                printHelp();
            } else if (cmd == "quit" || cmd == "exit") {
                running = false;
                g_shutdown = true;
            } else if (!cmd.empty()) {
                std::cout << "Unknown command: " << cmd << std::endl;
            }
        }
    }
};

// Signal handler for clean shutdown
void signalHandler(int signum) {
    std::cout << "\nShutdown signal received" << std::endl;
    g_shutdown = true;
}

// Main program
int main() {
    // Set up signal handling
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    std::cout << "==================================" << std::endl;
    std::cout << "   DRONE FLIGHT CONTROL SYSTEM    " << std::endl;
    std::cout << "==================================" << std::endl;
    std::cout << "Version 1.0 - With Obstacle Avoidance" << std::endl;
    std::cout << "\nInitializing..." << std::endl;
    
    try {
        // Create flight controller
        FlightController fc;
        
        // Status monitoring thread
        std::thread status_thread([&fc]() {
            while (!g_shutdown) {
                std::this_thread::sleep_for(std::chrono::seconds(5));
                if (!g_shutdown) {
                    fc.printStatus();
                }
            }
        });
        
        // Command interface
        CommandInterface cmd(fc);
        std::cout << "\nSystem ready! Type 'help' for commands.\n" << std::endl;
        cmd.run();
        
        // Clean shutdown
        g_shutdown = true;
        if (status_thread.joinable()) {
            status_thread.join();
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "Shutdown complete" << std::endl;
    return 0;
}