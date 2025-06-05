// drone_control.cpp - FHL-LD19 LIDAR COMPATIBLE DRONE FLIGHT CONTROL SYSTEM
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
#include <queue>

// Constants
constexpr float PI = 3.14159265359f;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI;
constexpr int PWM_MIN = 1000;
constexpr int PWM_MAX = 2000;
constexpr int PWM_ARM = 1100;
constexpr float OBSTACLE_THRESHOLD = 1.0f; // 1 meter
constexpr float CRITICAL_OBSTACLE_THRESHOLD = 0.5f; // 0.5 meter

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

// Enhanced Lidar data structure for FHL-LD19
struct LidarPoint {
    float angle;      // Angle in degrees
    float distance;   // Distance in meters
    uint8_t confidence; // Confidence/intensity value
};

struct LidarData {
    std::vector<LidarPoint> points;
    float rpm = 0;    // Rotation speed
    bool valid = false;
    std::mutex mutex;
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

// QMC5883L Magnetometer Driver (GEPRC GEP-M10 integrated)
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

// DPS310 Barometer Driver (GEPRC GEP-M10 integrated)
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

// FHL-LD19 360° Lidar Driver
class FHL_LD19_Lidar {
private:
    int fd = -1;
    std::thread read_thread;
    LidarData& lidar_data;
    std::queue<uint8_t> buffer_queue;
    std::mutex queue_mutex;
    
    static constexpr uint8_t HEADER = 0x54;
    static constexpr uint8_t VERLEN = 0x2C;
    static constexpr int PACKET_SIZE = 47;
    static constexpr int POINTS_PER_PACKET = 12;
    
    // CRC calculation table
    static constexpr uint8_t CrcTable[256] = {
        0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
        0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
        0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
        0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
        0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
        0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
        0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
        0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
        0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
        0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
        0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
        0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
        0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
        0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
        0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
        0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
        0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
        0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
        0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
        0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
        0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
        0x7f, 0x32, 0xe5, 0xa8
    };
    
    uint8_t CalCRC8(const uint8_t* data, uint16_t len) {
        uint8_t crc = 0;
        for (uint16_t i = 0; i < len; i++) {
            crc = CrcTable[(crc ^ data[i]) & 0xff];
        }
        return crc;
    }
    
    void readLoop() {
        uint8_t byte;
        while (!g_shutdown) {
            if (::read(fd, &byte, 1) == 1) {
                std::lock_guard<std::mutex> lock(queue_mutex);
                buffer_queue.push(byte);
            }
        }
    }
    
    bool processPacket() {
        std::lock_guard<std::mutex> lock(queue_mutex);
        
        // Need at least a full packet
        if (buffer_queue.size() < PACKET_SIZE) {
            return false;
        }
        
        // Look for header
        while (buffer_queue.size() >= PACKET_SIZE) {
            if (buffer_queue.front() == HEADER) {
                // Check second byte
                uint8_t packet[PACKET_SIZE];
                
                // Copy packet data
                std::queue<uint8_t> temp_queue = buffer_queue;
                for (int i = 0; i < PACKET_SIZE; i++) {
                    packet[i] = temp_queue.front();
                    temp_queue.pop();
                }
                
                if (packet[1] == VERLEN) {
                    // Verify CRC
                    uint8_t crc = CalCRC8(packet, PACKET_SIZE - 1);
                    if (crc == packet[PACKET_SIZE - 1]) {
                        // Valid packet, remove from queue
                        for (int i = 0; i < PACKET_SIZE; i++) {
                            buffer_queue.pop();
                        }
                        
                        // Parse packet
                        parsePacket(packet);
                        return true;
                    }
                }
            }
            
            // Remove one byte and try again
            buffer_queue.pop();
        }
        
        return false;
    }
    
    void parsePacket(const uint8_t* data) {
        // Extract lidar speed (RPM)
        uint16_t speed = (data[3] << 8) | data[2];
        float rpm = speed / 360.0f;
        
        // Extract start angle
        uint16_t start_angle = (data[5] << 8) | data[4];
        float start_angle_deg = start_angle / 100.0f;
        
        // Extract end angle
        uint16_t end_angle = (data[43] << 8) | data[42];
        float end_angle_deg = end_angle / 100.0f;
        
        // Calculate angle step
        float angle_diff = end_angle_deg - start_angle_deg;
        if (angle_diff < 0) angle_diff += 360.0f;
        float angle_step = angle_diff / (POINTS_PER_PACKET - 1);
        
        std::lock_guard<std::mutex> lock(lidar_data.mutex);
        lidar_data.rpm = rpm;
        lidar_data.valid = true;
        
        // Clear old data periodically
        static auto last_clear = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<float>(now - last_clear).count() > 1.0f) {
            lidar_data.points.clear();
            last_clear = now;
        }
        
        // Parse distance data
        for (int i = 0; i < POINTS_PER_PACKET; i++) {
            int base_idx = 6 + i * 3;
            uint16_t distance = (data[base_idx + 1] << 8) | data[base_idx];
            uint8_t confidence = data[base_idx + 2];
            
            if (distance > 0 && confidence > 30) {  // Filter low confidence points
                LidarPoint point;
                point.angle = start_angle_deg + i * angle_step;
                if (point.angle >= 360.0f) point.angle -= 360.0f;
                point.distance = distance / 1000.0f;  // Convert mm to meters
                point.confidence = confidence;
                
                lidar_data.points.push_back(point);
            }
        }
    }
    
public:
    FHL_LD19_Lidar(const char* device, LidarData& data) : lidar_data(data) {
        fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0) {
            std::cerr << "Warning: Failed to open LD19 LIDAR UART " << device << std::endl;
            return;
        }
        
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        if (tcgetattr(fd, &tty) != 0) {
            close(fd);
            fd = -1;
            std::cerr << "Warning: Failed to get LD19 LIDAR UART attributes" << std::endl;
            return;
        }
        
        // Configure for 230400 baud rate (LD19 default)
        cfsetospeed(&tty, B230400);
        cfsetispeed(&tty, B230400);
        
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit chars
        tty.c_iflag &= ~IGNBRK;                       // disable break processing
        tty.c_lflag = 0;                              // no signaling chars, no echo
        tty.c_oflag = 0;                              // no remapping, no delays
        tty.c_cc[VMIN] = 0;                          // read doesn't block
        tty.c_cc[VTIME] = 5;                         // 0.5 seconds read timeout
        
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);      // shut off xon/xoff ctrl
        tty.c_cflag |= (CLOCAL | CREAD);             // ignore modem controls
        tty.c_cflag &= ~(PARENB | PARODD);           // shut off parity
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        
        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            close(fd);
            fd = -1;
            std::cerr << "Warning: Failed to set LD19 LIDAR UART attributes" << std::endl;
            return;
        }
        
        // Start the lidar
        uint8_t start_cmd[] = {0x7C, 0x82, 0x05, 0x00, 0x01, 0xFF};
        ::write(fd, start_cmd, sizeof(start_cmd));
        
        // Start read thread
        read_thread = std::thread(&FHL_LD19_Lidar::readLoop, this);
        
        std::cout << "FHL-LD19 Lidar initialized on " << device << std::endl;
    }
    
    ~FHL_LD19_Lidar() {
        g_shutdown = true;
        if (read_thread.joinable()) {
            read_thread.join();
        }
        if (fd >= 0) {
            // Stop the lidar
            uint8_t stop_cmd[] = {0x7C, 0x82, 0x05, 0x00, 0x00, 0xFF};
            ::write(fd, stop_cmd, sizeof(stop_cmd));
            close(fd);
        }
    }
    
    bool isConnected() const { return fd >= 0; }
    
    void process() {
        while (processPacket()) {
            // Process all available packets
        }
    }
    
    // Get distance at specific angle
    float getDistanceAtAngle(float angle) {
        std::lock_guard<std::mutex> lock(lidar_data.mutex);
        
        float min_distance = 10.0f;
        float angle_tolerance = 5.0f;  // ±5 degrees
        
        for (const auto& point : lidar_data.points) {
            float angle_diff = std::abs(point.angle - angle);
            if (angle_diff > 180.0f) angle_diff = 360.0f - angle_diff;
            
            if (angle_diff <= angle_tolerance) {
                min_distance = std::min(min_distance, point.distance);
            }
        }
        
        return min_distance;
    }
    
    // Get minimum distance in angle range
    float getMinDistanceInRange(float start_angle, float end_angle) {
        std::lock_guard<std::mutex> lock(lidar_data.mutex);
        
        float min_distance = 10.0f;
        
        for (const auto& point : lidar_data.points) {
            bool in_range = false;
            
            if (start_angle <= end_angle) {
                in_range = (point.angle >= start_angle && point.angle <= end_angle);
            } else {
                // Handle wrap around 360 degrees
                in_range = (point.angle >= start_angle || point.angle <= end_angle);
            }
            
            if (in_range) {
                min_distance = std::min(min_distance, point.distance);
            }
        }
        
        return min_distance;
    }
    
    // Get full 360° scan data
    std::vector<float> get360Scan() {
        std::lock_guard<std::mutex> lock(lidar_data.mutex);
        
        std::vector<float> scan(360, 10.0f);  // Initialize with max range
        
        for (const auto& point : lidar_data.points) {
            int angle_idx = static_cast<int>(point.angle) % 360;
            scan[angle_idx] = std::min(scan[angle_idx], point.distance);
        }
        
        return scan;
    }
};

// GPS Parser for GEPRC GEP-M10
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
                if (buffer.find("$GPGGA") == 0 || buffer.find("$GNGGA") == 0) {
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
    // Enhanced obstacle avoidance
    struct ObstacleVector {
        float x = 0;
        float y = 0;
        float magnitude = 0;
    };
    ObstacleVector repulsion_vector;
    float safe_directions[360];  // Safety score for each direction
    float obstacle_map[360];     // Distance map from LD19
    
    // Precision landing
    bool precision_landing_active = false;
    float ground_level = 0.0f;
    float landing_target_x = 0.0f;
    float landing_target_y = 0.0f;
    float descent_rate = 0.5f;  // m/s
    float position_hold_radius = 0.1f;  // meters
    float touchdown_altitude = 0.05f;  // 5cm
    int ground_contact_count = 0;

    // Sensors
    std::unique_ptr<MPU6050> mpu6050;
    std::unique_ptr<QMC5883L> magnetometer;
    std::unique_ptr<DPS310> barometer;
    std::unique_ptr<FHL_LD19_Lidar> lidar;
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
    
    // Obstacle detection
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

    // Calculate repulsion vector from all obstacles
    void calculateRepulsionVector() {
        repulsion_vector.x = 0;
        repulsion_vector.y = 0;
        
        // Check all directions
        for (int angle = 0; angle < 360; angle += 5) {  // Check every 5 degrees
            float distance = obstacle_map[angle];
            
            if (distance < 3.0f) {  // Consider obstacles within 3 meters
                // Calculate repulsion force (inverse square law)
                float force = 1.0f / (distance * distance + 0.1f);  // +0.1 to avoid division by zero
                
                // Convert to cartesian coordinates
                float rad = angle * DEG_TO_RAD;
                repulsion_vector.x += force * std::cos(rad);
                repulsion_vector.y += force * std::sin(rad);
            }
        }
        
        // Calculate magnitude
        repulsion_vector.magnitude = std::sqrt(repulsion_vector.x * repulsion_vector.x + 
                                              repulsion_vector.y * repulsion_vector.y);
        
        // Normalize if needed
        if (repulsion_vector.magnitude > 0) {
            repulsion_vector.x /= repulsion_vector.magnitude;
            repulsion_vector.y /= repulsion_vector.magnitude;
        }
    }

    // Find the safest direction to move
    float findSafestDirection() {
        // Calculate safety score for each direction
        for (int angle = 0; angle < 360; angle++) {
            safe_directions[angle] = 10.0f;  // Start with max safety
            
            // Check a cone of ±30 degrees around this direction
            for (int check = -30; check <= 30; check += 5) {
                int check_angle = (angle + check + 360) % 360;
                float distance = obstacle_map[check_angle];
                
                // Reduce safety based on proximity
                float weight = 1.0f - std::abs(check) / 30.0f;  // Center weighted
                safe_directions[angle] = std::min(safe_directions[angle], distance * weight);
            }
        }
        
        // Find direction with highest safety score
        float max_safety = 0;
        int safest_angle = 0;
        
        for (int angle = 0; angle < 360; angle++) {
            if (safe_directions[angle] > max_safety) {
                max_safety = safe_directions[angle];
                safest_angle = angle;
            }
        }
        
        return safest_angle;
    }

    // Execute precision landing with ground detection
    void executePrecisionLanding(float dt) {
        // Get ground distance from LD19 looking down
        float ground_distance = lidar->getDistanceAtAngle(180);  // Assuming 180° is down
        
        if (ground_distance < 10.0f) {  // Valid ground reading
            // Update ground level estimate
            ground_level = state.z - ground_distance;
            
            // Phase 2: Position hold while descending
            float position_error_x = landing_target_x - state.x;
            float position_error_y = landing_target_y - state.y;
            float position_error = std::sqrt(position_error_x * position_error_x + 
                                           position_error_y * position_error_y);
            
            // Correct position if drifting
            if (position_error > position_hold_radius) {
                target_roll = position_error_y * 0.5f;  // Proportional control
                target_pitch = position_error_x * 0.5f;
                
                // Limit corrections
                target_roll = std::max(-0.1f, std::min(0.1f, target_roll));
                target_pitch = std::max(-0.1f, std::min(0.1f, target_pitch));
            } else {
                target_roll = 0;
                target_pitch = 0;
            }
            
            // Phase 3: Controlled descent
            float altitude_above_ground = ground_distance;
            
            if (altitude_above_ground > 2.0f) {
                // High altitude - normal descent
                descent_rate = 0.5f;
            } else if (altitude_above_ground > 0.5f) {
                // Medium altitude - slow down
                descent_rate = 0.3f;
            } else if (altitude_above_ground > touchdown_altitude) {
                // Final approach - very slow
                descent_rate = 0.1f;
            } else {
                // Touchdown detected
                descent_rate = 0.0f;
                ground_contact_count++;
                
                if (ground_contact_count > 10) {  // Confirm ground contact
                    std::cout << "*** TOUCHDOWN CONFIRMED ***" << std::endl;
                    std::cout << "Landing position error: " << position_error << "m" << std::endl;
                    precision_landing_active = false;
                    target_throttle = PWM_MIN;
                    
                    // Auto-disarm after 2 seconds
                    std::thread([this]() {
                        std::this_thread::sleep_for(std::chrono::seconds(2));
                        disarm();
                    }).detach();
                }
            }
            
            // Update altitude target
            target_altitude -= descent_rate * dt;
            target_altitude = std::max(ground_level, target_altitude);
            
            // Adjust throttle for descent
            if (altitude_above_ground < 0.5f) {
                // Very low - reduce throttle
                target_throttle = 1200 + (altitude_above_ground * 600);
            } else {
                target_throttle = 1400;  // Normal hover throttle
            }
            
            // Status output
            static int status_counter = 0;
            if (++status_counter % 20 == 0) {
                std::cout << "Precision Landing: Alt=" << altitude_above_ground 
                         << "m Pos_err=" << position_error 
                         << "m Descent=" << descent_rate << "m/s" << std::endl;
            }
        } else {
            std::cout << "WARNING: No LIDAR ground data for precision landing!" << std::endl;
        }
    }

    // Emergency obstacle response
    void emergencyObstacleResponse() {
        // Find closest obstacle
        float min_dist = 10.0f;
        int min_angle = 0;
        
        for (int i = 0; i < 360; i++) {
            if (obstacle_map[i] < min_dist) {
                min_dist = obstacle_map[i];
                min_angle = i;
            }
        }
        
        if (min_dist < CRITICAL_OBSTACLE_THRESHOLD) {  // Critical distance
            std::cout << "\n*** EMERGENCY OBSTACLE AVOIDANCE ***" << std::endl;
            std::cout << "Critical obstacle at " << min_angle << "° distance " << min_dist << "m" << std::endl;
            
            // Emergency stop
            target_roll = 0;
            target_pitch = 0;
            target_yaw_rate = 0;
            
            // Move away from obstacle
            float escape_angle = (min_angle + 180) % 360;
            float escape_rad = escape_angle * DEG_TO_RAD;
            
            target_pitch = 0.3f * std::cos(escape_rad);
            target_roll = 0.3f * std::sin(escape_rad);
            
            // Increase altitude if possible
            if (state.z < 10.0f) {
                target_altitude = state.z + 1.0f;
            }
        }
    }

    // Print 360° obstacle status
    void print360ObstacleStatus() {
        std::cout << "\n=== 360° Obstacle Analysis (FHL-LD19) ===" << std::endl;
        
        // Find obstacles in each quadrant
        struct Quadrant {
            std::string name;
            int start_angle;
            int end_angle;
            float min_distance;
            int obstacle_count;
        };
        
        Quadrant quadrants[4] = {
            {"Front", 315, 45, 10.0f, 0},
            {"Right", 45, 135, 10.0f, 0},
            {"Back", 135, 225, 10.0f, 0},
            {"Left", 225, 315, 10.0f, 0}
        };
        
        // Analyze each quadrant
        for (auto& quad : quadrants) {
            for (int angle = quad.start_angle; angle != quad.end_angle; angle = (angle + 1) % 360) {
                if (obstacle_map[angle] < 3.0f) {
                    quad.obstacle_count++;
                    quad.min_distance = std::min(quad.min_distance, obstacle_map[angle]);
                }
            }
            
            std::cout << quad.name << ": ";
            if (quad.obstacle_count > 0) {
                std::cout << quad.obstacle_count << " obstacles, closest at " 
                         << quad.min_distance << "m";
            } else {
                std::cout << "Clear";
            }
            std::cout << std::endl;
        }
        
        std::cout << "Safest direction: " << findSafestDirection() << "°" << std::endl;
        std::cout << "Repulsion vector: X=" << repulsion_vector.x 
                 << " Y=" << repulsion_vector.y 
                 << " Magnitude=" << repulsion_vector.magnitude << std::endl;
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
                
                // FHL-LD19 Lidar processing
                if (lidar && lidar->isConnected()) {
                    lidar->process();  // Process incoming packets
                    
                    // Get 360° scan data
                    auto scan_data = lidar->get360Scan();
                    
                    // Update obstacle map
                    for (int i = 0; i < 360; i++) {
                        obstacle_map[i] = scan_data[i];
                    }
                    
                    // Find minimum obstacle distance in front 90 degree arc
                    min_obstacle_distance = lidar->getMinDistanceInRange(315, 45);
                    
                    // Check critical zones
                    float front_dist = lidar->getMinDistanceInRange(350, 10);
                    float right_dist = lidar->getMinDistanceInRange(80, 100);
                    float back_dist = lidar->getMinDistanceInRange(170, 190);
                    float left_dist = lidar->getMinDistanceInRange(260, 280);
                    
                    // Trigger emergency response if needed
                    if (front_dist < CRITICAL_OBSTACLE_THRESHOLD || 
                        right_dist < CRITICAL_OBSTACLE_THRESHOLD ||
                        back_dist < CRITICAL_OBSTACLE_THRESHOLD ||
                        left_dist < CRITICAL_OBSTACLE_THRESHOLD) {
                        emergencyObstacleResponse();
                    }
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
                // ENHANCED 360° OBSTACLE AVOIDANCE with FHL-LD19
                calculateRepulsionVector();
                float safe_dir = findSafestDirection();
                
                float pitch_avoid = 0.0f;
                float roll_avoid = 0.0f;
                
                if (repulsion_vector.magnitude > 0.1f) {
                    // Apply repulsion vector
                    pitch_avoid = -repulsion_vector.y * 0.2f;  // Scale factor
                    roll_avoid = -repulsion_vector.x * 0.2f;
                    
                    // If current direction is unsafe, rotate towards safest direction
                    float current_heading = std::atan2(target_pitch, target_roll) * RAD_TO_DEG;
                    float heading_diff = safe_dir - current_heading;
                    
                    // Normalize angle difference
                    while (heading_diff > 180) heading_diff -= 360;
                    while (heading_diff < -180) heading_diff += 360;
                    
                    if (std::abs(heading_diff) > 45) {
                        target_yaw_rate = heading_diff > 0 ? 30.0f * DEG_TO_RAD : -30.0f * DEG_TO_RAD;
                    }
                    
                    std::cout << "\n*** 360° OBSTACLE AVOIDANCE ACTIVE (FHL-LD19) ***" << std::endl;
                    std::cout << "Repulsion vector: X=" << repulsion_vector.x 
                             << " Y=" << repulsion_vector.y 
                             << " Safest direction: " << safe_dir << "°" << std::endl;
                }
                
                // Apply avoidance to targets
                float effective_roll = target_roll + roll_avoid;
                float effective_pitch = target_pitch + pitch_avoid;
                
                // PRECISION LANDING MODE
                if (precision_landing_active) {
                    executePrecisionLanding(dt);
                    // Landing overrides normal control
                    effective_roll = target_roll;
                    effective_pitch = target_pitch;
                }
                
                // Calculate control errors
                float roll_error = effective_roll - state.roll;
                float pitch_error = effective_pitch - state.pitch;
                float yaw_rate_error = target_yaw_rate - imu_data.gz;
                float altitude_error = target_altitude - state.z;
                
                // PID control
                float roll_output = roll_pid.update(roll_error, dt);
                float pitch_output = pitch_pid.update(pitch_error, dt);
                float yaw_output = yaw_pid.update(yaw_rate_error, dt);
                float altitude_output = altitude_pid.update(altitude_error, dt);
                
                // Mix outputs for quadcopter X configuration
                float base_throttle = target_throttle + altitude_output;
                
                // Reduce throttle near obstacles
                if (repulsion_vector.magnitude > 0.5f) {
                    base_throttle *= (1.0f - 0.3f * std::min(1.0f, repulsion_vector.magnitude));
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
            
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    
public:
    FlightController() : state{} {
        std::cout << "Initializing Flight Controller with FHL-LD19 Lidar..." << std::endl;
        
        try {
            // Initialize sensors
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
                if (has_mag) std::cout << "✓ QMC5883L Magnetometer (GEPRC GEP-M10) initialized" << std::endl;
                else std::cout << "✗ QMC5883L Magnetometer not found" << std::endl;
            } catch (...) {
                std::cout << "✗ QMC5883L Magnetometer initialization failed" << std::endl;
            }
            
            try {
                barometer = std::make_unique<DPS310>("/dev/i2c-1");
                has_baro = barometer->isConnected();
                if (has_baro) std::cout << "✓ DPS310 Barometer (GEPRC GEP-M10) initialized" << std::endl;
                else std::cout << "✗ DPS310 Barometer not found" << std::endl;
            } catch (...) {
                std::cout << "✗ DPS310 Barometer initialization failed" << std::endl;
            }
            
            try {
                lidar = std::make_unique<FHL_LD19_Lidar>("/dev/ttyUSB0", lidar_data);
                has_lidar = lidar->isConnected();
                if (has_lidar) std::cout << "✓ FHL-LD19 360° Lidar initialized" << std::endl;
                else std::cout << "✗ FHL-LD19 Lidar not found on /dev/ttyUSB0" << std::endl;
            } catch (...) {
                std::cout << "✗ FHL-LD19 Lidar initialization failed" << std::endl;
            }
            
            try {
                gps = std::make_unique<GPSParser>("/dev/ttyUSB1");
                has_gps = gps->isConnected();
                if (has_gps) std::cout << "✓ GEPRC GEP-M10 GPS initialized" << std::endl;
                else std::cout << "✗ GPS not found on /dev/ttyUSB1" << std::endl;
            } catch (...) {
                std::cout << "✗ GPS initialization failed" << std::endl;
            }
            
            // Initialize obstacle map
            for (int i = 0; i < 360; i++) {
                obstacle_map[i] = 10.0f;  // Max range
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
            }
            
            if (has_lidar) {
                std::cout << "\nFHL-LD19 360° Obstacle Detection ACTIVE" << std::endl;
                std::cout << "Obstacle threshold: " << OBSTACLE_THRESHOLD << "m" << std::endl;
                std::cout << "Critical threshold: " << CRITICAL_OBSTACLE_THRESHOLD << "m" << std::endl;
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
                std::cout << "FHL-LD19 360° obstacle avoidance ACTIVE" << std::endl;
                std::cout << "Lidar RPM: " << lidar_data.rpm << std::endl;
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
        precision_landing_active = false;
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
        if (!has_lidar) {
            // Fallback to simple landing
            std::cout << "No LIDAR - using simple landing mode" << std::endl;
            target_altitude = 0;
            target_throttle = 1200;
            
            std::thread([this]() {
                while (state.z > 0.1f && state.armed) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                if (state.armed && state.z <= 0.1f) {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    disarm();
                }
            }).detach();
        } else {
            // Precision landing with ground detection
            std::cout << "*** PRECISION LANDING ENGAGED (FHL-LD19) ***" << std::endl;
            precision_landing_active = true;
            ground_contact_count = 0;
            
            // Save current position as landing target
            landing_target_x = state.x;
            landing_target_y = state.y;
            
            // Start descent
            descent_rate = 0.5f;
        }
    }
    
    void move(float forward_m, float right_m, float up_m) {
        // Check obstacles before movement
        if (forward_m > 0) {
            float front_dist = lidar->getMinDistanceInRange(350, 10);
            if (front_dist < OBSTACLE_THRESHOLD) {
                std::cout << "WARNING: Obstacle ahead at " << front_dist << "m - forward movement blocked!" << std::endl;
                return;
            }
        }
        if (right_m > 0) {
            float right_dist = lidar->getMinDistanceInRange(80, 100);
            if (right_dist < OBSTACLE_THRESHOLD) {
                std::cout << "WARNING: Obstacle on right at " << right_dist << "m - right movement blocked!" << std::endl;
                return;
            }
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
    
    void scanEnvironment() {
        if (!has_lidar) {
            std::cout << "No LIDAR available for scanning" << std::endl;
            return;
        }
        
        std::cout << "Performing 360° environment scan..." << std::endl;
        
        // Rotate slowly while scanning
        float original_yaw = state.yaw;
        target_yaw_rate = 30.0f * DEG_TO_RAD;  // 30 deg/s
        
        auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::duration<float>(std::chrono::steady_clock::now() - start_time).count() < 12.0f) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        target_yaw_rate = 0;
        std::cout << "Scan complete!" << std::endl;
        print360ObstacleStatus();
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
            std::cout << "FHL-LD19 Lidar: RPM=" << lidar_data.rpm << std::endl;
            std::cout << "  Front: " << lidar->getMinDistanceInRange(350, 10) << "m" << std::endl;
            std::cout << "  Right: " << lidar->getMinDistanceInRange(80, 100) << "m" << std::endl;
            std::cout << "  Back: " << lidar->getMinDistanceInRange(170, 190) << "m" << std::endl;
            std::cout << "  Left: " << lidar->getMinDistanceInRange(260, 280) << "m" << std::endl;
            std::cout << "  Min distance: " << min_obstacle_distance << "m" << std::endl;
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
        std::cout << "land       - Precision land with FHL-LD19" << std::endl;
        std::cout << "move F R U - Move Forward/Right/Up (meters)" << std::endl;
        std::cout << "rotate D   - Rotate D degrees/sec" << std::endl;
        std::cout << "hover      - Hold position" << std::endl;
        std::cout << "scan       - Perform 360° environment scan" << std::endl;
        std::cout << "obstacles  - Show 360° obstacle analysis" << std::endl;
        std::cout << "goto LAT LON ALT - Fly to GPS coordinate" << std::endl;
        std::cout << "status     - Show drone status" << std::endl;
        std::cout << "help       - Show this help" << std::endl;
        std::cout << "quit       - Exit program" << std::endl;
        std::cout << "\nFHL-LD19 360° OBSTACLE AVOIDANCE is automatic when armed!" << std::endl;
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
                fc.scanEnvironment();
            } else if (cmd == "obstacles") {
                fc.print360ObstacleStatus();
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
    
    std::cout << "==========================================" << std::endl;
    std::cout << "   DRONE FLIGHT CONTROL SYSTEM v2.0      " << std::endl;
    std::cout << "==========================================" << std::endl;
    std::cout << "FHL-LD19 360° Lidar Obstacle Avoidance" << std::endl;
    std::cout << "GEPRC GEP-M10 GPS/MAG/BARO Module" << std::endl;
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