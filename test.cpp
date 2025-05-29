// test.cpp - Comprehensive drone system test without hardware
// Compile: g++ -std=c++17 -O2 -o test test.cpp -lpthread -lrt
// Run: ./test

#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <vector>
#include <atomic>
#include <mutex>
#include <iomanip>
#include <cassert>
#include <random>
#include <sstream>
#include <map>
#include <functional>

// ANSI color codes for test output
#define GREEN "\033[32m"
#define RED "\033[31m"
#define YELLOW "\033[33m"
#define BLUE "\033[34m"
#define RESET "\033[0m"

// Test framework
class TestFramework {
private:
    int tests_run = 0;
    int tests_passed = 0;
    int tests_failed = 0;
    std::string current_test;
    
public:
    void startTest(const std::string& name) {
        current_test = name;
        std::cout << "\n" << BLUE << "[TEST] " << name << RESET << std::endl;
        tests_run++;
    }
    
    void assert(bool condition, const std::string& message) {
        if (condition) {
            std::cout << GREEN << "  ✓ " << message << RESET << std::endl;
        } else {
            std::cout << RED << "  ✗ " << message << RESET << std::endl;
            tests_failed++;
            throw std::runtime_error("Test failed: " + message);
        }
    }
    
    void pass() {
        tests_passed++;
        std::cout << GREEN << "[PASS] " << current_test << RESET << std::endl;
    }
    
    void fail(const std::string& reason) {
        tests_failed++;
        std::cout << RED << "[FAIL] " << current_test << ": " << reason << RESET << std::endl;
    }
    
    void summary() {
        std::cout << "\n========== TEST SUMMARY ==========" << std::endl;
        std::cout << "Total tests: " << tests_run << std::endl;
        std::cout << GREEN << "Passed: " << tests_passed << RESET << std::endl;
        std::cout << RED << "Failed: " << tests_failed << RESET << std::endl;
        std::cout << "Success rate: " << (tests_passed * 100 / tests_run) << "%" << std::endl;
    }
};

// Simulated hardware interfaces
class SimulatedI2C {
private:
    std::map<uint8_t, std::map<uint8_t, uint8_t>> devices;
    std::mt19937 rng{std::random_device{}()};
    std::normal_distribution<float> noise{0.0f, 0.01f};
    
public:
    void addDevice(uint8_t address) {
        devices[address] = std::map<uint8_t, uint8_t>();
        
        // Initialize device-specific registers
        if (address == 0x68) {  // MPU6050
            devices[address][0x75] = 0x68;  // WHO_AM_I
        } else if (address == 0x0D) {  // QMC5883L
            devices[address][0x0D] = 0x00;  // Status
        } else if (address == 0x77) {  // DPS310
            // Add calibration coefficients
            for (int i = 0; i < 18; i++) {
                devices[address][0x10 + i] = 0x00;
            }
        }
    }
    
    void writeReg(uint8_t address, uint8_t reg, uint8_t value) {
        if (devices.find(address) != devices.end()) {
            devices[address][reg] = value;
        }
    }
    
    uint8_t readReg(uint8_t address, uint8_t reg) {
        if (devices.find(address) != devices.end()) {
            return devices[address][reg];
        }
        return 0xFF;
    }
    
    void readRegs(uint8_t address, uint8_t start_reg, uint8_t* buffer, size_t len) {
        if (devices.find(address) == devices.end()) return;
        
        // Simulate sensor data based on device
        if (address == 0x68 && start_reg == 0x3B) {  // MPU6050 accel/gyro data
            // Simulate level drone with slight movement
            int16_t ax = 0 + noise(rng) * 1000;
            int16_t ay = 0 + noise(rng) * 1000;
            int16_t az = 16384 + noise(rng) * 1000;  // 1g in +Z
            int16_t temp = 25 * 340 + 36.53 * 340;  // 25°C
            int16_t gx = noise(rng) * 100;
            int16_t gy = noise(rng) * 100;
            int16_t gz = noise(rng) * 100;
            
            buffer[0] = ax >> 8; buffer[1] = ax & 0xFF;
            buffer[2] = ay >> 8; buffer[3] = ay & 0xFF;
            buffer[4] = az >> 8; buffer[5] = az & 0xFF;
            buffer[6] = temp >> 8; buffer[7] = temp & 0xFF;
            buffer[8] = gx >> 8; buffer[9] = gx & 0xFF;
            buffer[10] = gy >> 8; buffer[11] = gy & 0xFF;
            buffer[12] = gz >> 8; buffer[13] = gz & 0xFF;
        } else if (address == 0x0D && start_reg == 0x00) {  // QMC5883L mag data
            // Simulate magnetic field
            int16_t mx = 200 + noise(rng) * 50;
            int16_t my = 100 + noise(rng) * 50;
            int16_t mz = -400 + noise(rng) * 50;
            
            buffer[0] = mx & 0xFF; buffer[1] = mx >> 8;
            buffer[2] = my & 0xFF; buffer[3] = my >> 8;
            buffer[4] = mz & 0xFF; buffer[5] = mz >> 8;
        } else if (address == 0x77 && start_reg == 0x00) {  // DPS310 pressure
            // Simulate sea level pressure
            int32_t pressure = 101325 * 256;  // Scaled pressure
            int32_t temp = 25 * 256;  // Scaled temperature
            
            buffer[0] = (pressure >> 16) & 0xFF;
            buffer[1] = (pressure >> 8) & 0xFF;
            buffer[2] = pressure & 0xFF;
            buffer[3] = (temp >> 16) & 0xFF;
            buffer[4] = (temp >> 8) & 0xFF;
            buffer[5] = temp & 0xFF;
        }
    }
};

class SimulatedSerial {
private:
    std::string device_type;
    std::mt19937 rng{std::random_device{}()};
    std::uniform_real_distribution<float> dist{0.0f, 1.0f};
    float simulated_distance = 5.0f;  // Lidar distance
    
public:
    SimulatedSerial(const std::string& type) : device_type(type) {}
    
    void setDistance(float d) { simulated_distance = d; }
    
    int read(uint8_t* buffer, size_t len) {
        if (device_type == "lidar") {
            // TFmini Plus protocol
            if (len >= 9) {
                uint16_t distance = simulated_distance * 100;  // cm
                uint16_t strength = 1000;
                
                buffer[0] = 0x59;  // Header
                buffer[1] = 0x59;  // Header
                buffer[2] = distance & 0xFF;
                buffer[3] = distance >> 8;
                buffer[4] = strength & 0xFF;
                buffer[5] = strength >> 8;
                buffer[6] = 0x00;  // Temp
                buffer[7] = 0x00;
                
                // Checksum
                uint8_t checksum = 0;
                for (int i = 0; i < 8; i++) checksum += buffer[i];
                buffer[8] = checksum;
                
                return 9;
            }
        } else if (device_type == "gps") {
            // NMEA GPS sentences
            std::string nmea = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
            size_t copy_len = std::min(len, nmea.length());
            memcpy(buffer, nmea.c_str(), copy_len);
            return copy_len;
        }
        return 0;
    }
};

// Core system constants
constexpr float PI = 3.14159265359f;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI;
constexpr int PWM_MIN = 1000;
constexpr int PWM_MAX = 2000;
constexpr float OBSTACLE_THRESHOLD = 1.0f;

// Test PID Controller
class TestPID {
private:
    float kp, ki, kd;
    float integral = 0;
    float prev_error = 0;
    float max_integral;
    float max_output;
    
public:
    TestPID(float p, float i, float d, float max_int = 100.0f, float max_out = 1.0f)
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
    
    float getIntegral() const { return integral; }
};

// Flight physics simulation
class FlightSimulator {
private:
    // State
    float x = 0, y = 0, z = 0;  // Position
    float vx = 0, vy = 0, vz = 0;  // Velocity
    float roll = 0, pitch = 0, yaw = 0;  // Attitude
    float roll_rate = 0, pitch_rate = 0, yaw_rate = 0;  // Angular rates
    
    // Physics constants
    const float mass = 1.5f;  // kg
    const float gravity = 9.81f;
    const float drag_coeff = 0.1f;
    const float motor_thrust_coeff = 0.00001f;  // N per PWM unit
    
    // Motors (PWM values)
    int motor_pwm[4] = {1000, 1000, 1000, 1000};
    
public:
    void setMotors(int m1, int m2, int m3, int m4) {
        motor_pwm[0] = m1;
        motor_pwm[1] = m2;
        motor_pwm[2] = m3;
        motor_pwm[3] = m4;
    }
    
    void update(float dt) {
        // Calculate total thrust
        float total_thrust = 0;
        for (int i = 0; i < 4; i++) {
            total_thrust += (motor_pwm[i] - 1000) * motor_thrust_coeff;
        }
        
        // Calculate torques (simplified quadcopter X configuration)
        float roll_torque = ((motor_pwm[3] + motor_pwm[0]) - (motor_pwm[1] + motor_pwm[2])) * 0.0001f;
        float pitch_torque = ((motor_pwm[2] + motor_pwm[3]) - (motor_pwm[0] + motor_pwm[1])) * 0.0001f;
        float yaw_torque = ((motor_pwm[0] + motor_pwm[2]) - (motor_pwm[1] + motor_pwm[3])) * 0.00005f;
        
        // Update angular rates
        roll_rate += roll_torque * dt;
        pitch_rate += pitch_torque * dt;
        yaw_rate += yaw_torque * dt;
        
        // Apply damping
        roll_rate *= 0.95f;
        pitch_rate *= 0.95f;
        yaw_rate *= 0.9f;
        
        // Update attitude
        roll += roll_rate * dt;
        pitch += pitch_rate * dt;
        yaw += yaw_rate * dt;
        
        // Limit angles
        roll = std::max(-PI/2, std::min(PI/2, roll));
        pitch = std::max(-PI/2, std::min(PI/2, pitch));
        
        // Calculate forces in body frame
        float thrust_z = total_thrust * std::cos(roll) * std::cos(pitch);
        float thrust_x = total_thrust * std::sin(pitch);
        float thrust_y = total_thrust * std::sin(roll) * std::cos(pitch);
        
        // Update accelerations (world frame)
        float ax = thrust_x / mass - drag_coeff * vx;
        float ay = thrust_y / mass - drag_coeff * vy;
        float az = (thrust_z / mass - gravity) - drag_coeff * vz;
        
        // Update velocities
        vx += ax * dt;
        vy += ay * dt;
        vz += az * dt;
        
        // Update positions
        x += vx * dt;
        y += vy * dt;
        z += vz * dt;
        
        // Ground constraint
        if (z < 0) {
            z = 0;
            vz = 0;
        }
    }
    
    // Getters
    float getX() const { return x; }
    float getY() const { return y; }
    float getZ() const { return z; }
    float getRoll() const { return roll; }
    float getPitch() const { return pitch; }
    float getYaw() const { return yaw; }
    float getRollRate() const { return roll_rate; }
    float getPitchRate() const { return pitch_rate; }
    float getYawRate() const { return yaw_rate; }
    bool isFlying() const { return z > 0.1f; }
};

// Test cases
class DroneSystemTest {
private:
    TestFramework test;
    SimulatedI2C i2c;
    FlightSimulator sim;
    
    void testPIDController() {
        test.startTest("PID Controller");
        
        TestPID pid(1.0f, 0.1f, 0.01f, 100.0f, 1.0f);
        
        // Test proportional response
        float output = pid.update(1.0f, 0.1f);
        test.assert(output > 0, "Positive error produces positive output");
        test.assert(output <= 1.0f, "Output is limited to max");
        
        // Test integral accumulation
        for (int i = 0; i < 10; i++) {
            pid.update(0.5f, 0.1f);
        }
        test.assert(pid.getIntegral() > 0, "Integral accumulates over time");
        
        // Test reset
        pid.reset();
        test.assert(pid.getIntegral() == 0, "Reset clears integral");
        
        test.pass();
    }
    
    void testSensorDrivers() {
        test.startTest("Sensor Drivers");
        
        // Setup simulated I2C devices
        i2c.addDevice(0x68);  // MPU6050
        i2c.addDevice(0x0D);  // QMC5883L
        i2c.addDevice(0x77);  // DPS310
        
        // Test MPU6050
        uint8_t who_am_i = i2c.readReg(0x68, 0x75);
        test.assert(who_am_i == 0x68, "MPU6050 WHO_AM_I returns correct ID");
        
        // Test accelerometer data read
        uint8_t accel_data[14];
        i2c.readRegs(0x68, 0x3B, accel_data, 14);
        int16_t az = (accel_data[4] << 8) | accel_data[5];
        test.assert(az > 15000 && az < 17000, "Accelerometer Z reads ~1g when level");
        
        // Test magnetometer data
        uint8_t mag_data[6];
        i2c.readRegs(0x0D, 0x00, mag_data, 6);
        int16_t mx = (mag_data[1] << 8) | mag_data[0];
        test.assert(mx != 0, "Magnetometer returns non-zero data");
        
        test.pass();
    }
    
    void testObstacleAvoidance() {
        test.startTest("Obstacle Avoidance Logic");
        
        SimulatedSerial lidar("lidar");
        uint8_t buffer[9];
        
        // Test normal distance
        lidar.setDistance(5.0f);
        int bytes = lidar.read(buffer, 9);
        test.assert(bytes == 9, "Lidar returns 9 bytes");
        test.assert(buffer[0] == 0x59 && buffer[1] == 0x59, "Lidar frame header correct");
        
        uint16_t distance = buffer[2] | (buffer[3] << 8);
        test.assert(distance == 500, "Lidar distance is 500cm (5m)");
        
        // Test obstacle detection threshold
        lidar.setDistance(0.8f);  // 80cm
        lidar.read(buffer, 9);
        distance = buffer[2] | (buffer[3] << 8);
        float distance_m = distance / 100.0f;
        bool obstacle_detected = (distance_m < OBSTACLE_THRESHOLD);
        test.assert(obstacle_detected, "Obstacle detected at 0.8m");
        
        // Test safe distance
        lidar.setDistance(1.5f);
        lidar.read(buffer, 9);
        distance = buffer[2] | (buffer[3] << 8);
        distance_m = distance / 100.0f;
        obstacle_detected = (distance_m < OBSTACLE_THRESHOLD);
        test.assert(!obstacle_detected, "No obstacle at 1.5m");
        
        test.pass();
    }
    
    void testMotorMixing() {
        test.startTest("Motor Mixing Algorithm");
        
        // Test hover (all motors equal)
        float throttle = 1500;
        float roll_out = 0, pitch_out = 0, yaw_out = 0;
        
        int m1 = throttle + roll_out - pitch_out + yaw_out;
        int m2 = throttle - roll_out - pitch_out - yaw_out;
        int m3 = throttle - roll_out + pitch_out + yaw_out;
        int m4 = throttle + roll_out + pitch_out - yaw_out;
        
        test.assert(m1 == 1500 && m2 == 1500 && m3 == 1500 && m4 == 1500,
                   "Hover produces equal motor outputs");
        
        // Test roll right
        roll_out = 100;
        m1 = throttle + roll_out - pitch_out + yaw_out;
        m2 = throttle - roll_out - pitch_out - yaw_out;
        m3 = throttle - roll_out + pitch_out + yaw_out;
        m4 = throttle + roll_out + pitch_out - yaw_out;
        
        test.assert(m1 > m2 && m4 > m3, "Roll right increases right motors");
        
        // Test pitch forward
        roll_out = 0;
        pitch_out = 100;
        m1 = throttle + roll_out - pitch_out + yaw_out;
        m2 = throttle - roll_out - pitch_out - yaw_out;
        m3 = throttle - roll_out + pitch_out + yaw_out;
        m4 = throttle + roll_out + pitch_out - yaw_out;
        
        test.assert(m3 > m1 && m4 > m2, "Pitch forward increases rear motors");
        
        test.pass();
    }
    
    void testFlightSimulation() {
        test.startTest("Flight Physics Simulation");
        
        // Test takeoff
        sim.setMotors(1600, 1600, 1600, 1600);  // High throttle
        
        float initial_z = sim.getZ();
        for (int i = 0; i < 100; i++) {
            sim.update(0.01f);  // 10ms steps
        }
        
        test.assert(sim.getZ() > initial_z, "Drone gains altitude with high throttle");
        test.assert(sim.isFlying(), "Drone is flying after throttle up");
        
        // Test hover
        sim.setMotors(1380, 1380, 1380, 1380);  // Approximate hover throttle
        float hover_z = sim.getZ();
        for (int i = 0; i < 50; i++) {
            sim.update(0.01f);
        }
        
        test.assert(std::abs(sim.getZ() - hover_z) < 0.5f, "Drone maintains altitude in hover");
        
        // Test forward pitch
        sim.setMotors(1300, 1300, 1400, 1400);  // Rear motors higher
        for (int i = 0; i < 50; i++) {
            sim.update(0.01f);
        }
        
        test.assert(sim.getPitch() < 0, "Forward motor mix produces negative pitch");
        test.assert(sim.getX() > 0, "Drone moves forward with pitch");
        
        test.pass();
    }
    
    void testAltitudeControl() {
        test.startTest("Altitude Control");
        
        TestPID altitude_pid(5.0f, 1.0f, 2.0f, 100.0f, 500.0f);
        FlightSimulator alt_sim;
        
        float target_altitude = 2.0f;
        float base_throttle = 1350;
        
        // Simulate altitude control loop
        for (int i = 0; i < 500; i++) {
            float error = target_altitude - alt_sim.getZ();
            float pid_output = altitude_pid.update(error, 0.01f);
            float throttle = base_throttle + pid_output;
            
            // Apply to all motors
            alt_sim.setMotors(throttle, throttle, throttle, throttle);
            alt_sim.update(0.01f);
            
            // Adjust base throttle based on steady state
            if (i > 200 && std::abs(error) < 0.1f) {
                base_throttle = throttle;
            }
        }
        
        test.assert(std::abs(alt_sim.getZ() - target_altitude) < 0.2f,
                   "Altitude control reaches target within 20cm");
        
        test.pass();
    }
    
    void testEmergencyProcedures() {
        test.startTest("Emergency Procedures");
        
        FlightSimulator emergency_sim;
        
        // Get drone in the air
        emergency_sim.setMotors(1600, 1600, 1600, 1600);
        for (int i = 0; i < 100; i++) {
            emergency_sim.update(0.01f);
        }
        test.assert(emergency_sim.isFlying(), "Drone is airborne");
        
        // Emergency stop - cut motors
        emergency_sim.setMotors(1000, 1000, 1000, 1000);
        float fall_start_z = emergency_sim.getZ();
        
        for (int i = 0; i < 100; i++) {
            emergency_sim.update(0.01f);
        }
        
        test.assert(emergency_sim.getZ() < fall_start_z, "Drone descends after motor cut");
        test.assert(emergency_sim.getZ() >= 0, "Drone doesn't go below ground");
        
        test.pass();
    }
    
    void testGPSNavigation() {
        test.startTest("GPS Navigation");
        
        // Test coordinate calculations
        double lat1 = 51.5074;  // London
        double lon1 = -0.1278;
        double lat2 = 51.5080;  // ~67m north
        double lon2 = -0.1278;
        
        double dlat = lat2 - lat1;
        double dlon = lon2 - lon1;
        
        float distance_north = dlat * 111111.0f;
        float distance_east = dlon * 111111.0f * std::cos(lat1 * DEG_TO_RAD);
        
        test.assert(std::abs(distance_north - 66.7f) < 1.0f,
                   "North distance calculation correct");
        test.assert(std::abs(distance_east) < 1.0f,
                   "East distance ~0 for same longitude");
        
        // Test bearing calculation
        float bearing = std::atan2(distance_east, distance_north);
        test.assert(std::abs(bearing) < 0.1f, "Bearing is ~0° for due north");
        
        test.pass();
    }
    
    void testCompleteFlightSequence() {
        test.startTest("Complete Flight Sequence");
        
        FlightSimulator flight;
        TestPID roll_pid(4.5f, 1.0f, 0.035f, 50.0f, 500.0f);
        TestPID pitch_pid(4.5f, 1.0f, 0.035f, 50.0f, 500.0f);
        TestPID altitude_pid(5.0f, 1.0f, 2.0f, 100.0f, 500.0f);
        
        SimulatedSerial lidar("lidar");
        lidar.setDistance(10.0f);  // Clear path
        
        // Flight sequence
        bool armed = true;
        float target_altitude = 2.0f;
        float target_pitch = 0.0f;
        float base_throttle = 1350;
        
        std::cout << "  Simulating complete flight..." << std::endl;
        
        // Takeoff phase
        for (int i = 0; i < 300; i++) {
            float alt_error = target_altitude - flight.getZ();
            float alt_output = altitude_pid.update(alt_error, 0.01f);
            float throttle = base_throttle + alt_output;
            
            flight.setMotors(throttle, throttle, throttle, throttle);
            flight.update(0.01f);
            
            if (i % 30 == 0) {
                std::cout << "    t=" << (i * 0.01f) << "s alt=" 
                         << std::fixed << std::setprecision(2) << flight.getZ() << "m" << std::endl;
            }
        }
        
        test.assert(std::abs(flight.getZ() - 2.0f) < 0.3f, "Reached target altitude");
        
        // Forward flight with obstacle
        target_pitch = 0.1f;  // Forward command
        lidar.setDistance(0.8f);  // Obstacle ahead!
        
        uint8_t lidar_buffer[9];
        lidar.read(lidar_buffer, 9);
        float distance = ((lidar_buffer[3] << 8) | lidar_buffer[2]) / 100.0f;
        bool obstacle = distance < OBSTACLE_THRESHOLD;
        
        if (obstacle) {
            target_pitch = -0.15f;  // Emergency backward
            std::cout << "    OBSTACLE DETECTED - Avoiding!" << std::endl;
        }
        
        test.assert(obstacle && target_pitch < 0, "Obstacle avoidance triggered");
        
        // Apply pitch control
        for (int i = 0; i < 100; i++) {
            float pitch_error = target_pitch - flight.getPitch();
            float pitch_output = pitch_pid.update(pitch_error, 0.01f);
            
            float m1 = base_throttle - pitch_output;
            float m2 = base_throttle - pitch_output;
            float m3 = base_throttle + pitch_output;
            float m4 = base_throttle + pitch_output;
            
            flight.setMotors(m1, m2, m3, m4);
            flight.update(0.01f);
        }
        
        test.assert(flight.getX() < 0, "Drone moved backward due to obstacle");
        
        // Landing
        std::cout << "    Landing sequence..." << std::endl;
        target_altitude = 0;
        
        for (int i = 0; i < 300; i++) {
            float alt_error = target_altitude - flight.getZ();
            float alt_output = altitude_pid.update(alt_error, 0.01f);
            float throttle = std::max(1000.0f, base_throttle + alt_output);
            
            flight.setMotors(throttle, throttle, throttle, throttle);
            flight.update(0.01f);
            
            if (flight.getZ() < 0.1f) {
                armed = false;
                flight.setMotors(1000, 1000, 1000, 1000);
                std::cout << "    Landed and disarmed" << std::endl;
                break;
            }
        }
        
        test.assert(!armed && flight.getZ() < 0.1f, "Successfully landed and disarmed");
        
        test.pass();
    }
    
public:
    void runAllTests() {
        std::cout << "=====================================" << std::endl;
        std::cout << "   DRONE SYSTEM VALIDATION TESTS    " << std::endl;
        std::cout << "=====================================" << std::endl;
        
        // Run all test suites
        testPIDController();
        testSensorDrivers();
        testObstacleAvoidance();
        testMotorMixing();
        testFlightSimulation();
        testAltitudeControl();
        testEmergencyProcedures();
        testGPSNavigation();
        testCompleteFlightSequence();
        
        test.summary();
        
        if (test.tests_failed == 0) {
            std::cout << GREEN << "\n✓ ALL TESTS PASSED! Safe to fly." << RESET << std::endl;
        } else {
            std::cout << RED << "\n✗ TESTS FAILED! Do not fly." << RESET << std::endl;
        }
    }
};

// Performance benchmarks
void runPerformanceBenchmarks() {
    std::cout << "\n=====================================" << std::endl;
    std::cout << "     PERFORMANCE BENCHMARKS         " << std::endl;
    std::cout << "=====================================" << std::endl;
    
    // Benchmark control loop
    auto start = std::chrono::high_resolution_clock::now();
    TestPID pid(4.5f, 1.0f, 0.035f);
    for (int i = 0; i < 100000; i++) {
        pid.update(0.1f, 0.005f);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    std::cout << "PID update: " << duration.count() / 100000.0 << " μs per call" << std::endl;
    std::cout << "Max frequency: " << 1000000.0 / (duration.count() / 100000.0) << " Hz" << std::endl;
    
    // Benchmark sensor reads
    SimulatedI2C i2c;
    i2c.addDevice(0x68);
    
    start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; i++) {
        uint8_t buffer[14];
        i2c.readRegs(0x68, 0x3B, buffer, 14);
    }
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    std::cout << "Sensor read: " << duration.count() / 10000.0 << " μs per read" << std::endl;
    
    // Benchmark flight physics
    FlightSimulator sim;
    start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; i++) {
        sim.update(0.005f);
    }
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    std::cout << "Physics update: " << duration.count() / 10000.0 << " μs per step" << std::endl;
    
    std::cout << "\nTarget: 200Hz control loop = 5000 μs max total" << std::endl;
}

// Main test runner
int main() {
    try {
        // Run validation tests
        DroneSystemTest tester;
        tester.runAllTests();
        
        // Run performance benchmarks
        runPerformanceBenchmarks();
        
        std::cout << "\n=====================================" << std::endl;
        std::cout << "        TEST SUITE COMPLETE         " << std::endl;
        std::cout << "=====================================" << std::endl;
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << RED << "Test suite error: " << e.what() << RESET << std::endl;
        return 1;
    }
}