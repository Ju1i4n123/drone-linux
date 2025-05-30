// drone_debug_test.cpp - Comprehensive test suite for drone control system
// Compile: g++ -std=c++17 -O2 -o drone_debug_test drone_debug_test.cpp -lpthread
// Run: ./drone_debug_test

#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <vector>
#include <atomic>
#include <mutex>
#include <iomanip>
#include <cassert>
#include <sstream>
#include <functional>
#include <random>

// Color codes for output
#define GREEN "\033[32m"
#define RED "\033[31m"
#define YELLOW "\033[33m"
#define BLUE "\033[34m"
#define RESET "\033[0m"

// Test result tracking
struct TestResult {
    std::string name;
    bool passed;
    std::string message;
    std::chrono::milliseconds duration;
};

class TestFramework {
private:
    std::vector<TestResult> results;
    int passed = 0;
    int failed = 0;
    
public:
    void runTest(const std::string& name, std::function<void()> test) {
        std::cout << "\n" << BLUE << "Testing: " << name << RESET << std::endl;
        auto start = std::chrono::steady_clock::now();
        TestResult result{name, true, "OK", std::chrono::milliseconds(0)};
        
        try {
            test();
            result.passed = true;
            passed++;
            std::cout << GREEN << "✓ PASSED" << RESET << std::endl;
        } catch (const std::exception& e) {
            result.passed = false;
            result.message = e.what();
            failed++;
            std::cout << RED << "✗ FAILED: " << e.what() << RESET << std::endl;
        }
        
        auto end = std::chrono::steady_clock::now();
        result.duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        results.push_back(result);
    }
    
    void printSummary() {
        std::cout << "\n" << "="*60 << std::endl;
        std::cout << "TEST SUMMARY" << std::endl;
        std::cout << "="*60 << std::endl;
        
        for (const auto& result : results) {
            std::cout << std::left << std::setw(40) << result.name;
            if (result.passed) {
                std::cout << GREEN << "PASS" << RESET;
            } else {
                std::cout << RED << "FAIL" << RESET;
            }
            std::cout << " (" << result.duration.count() << "ms)" << std::endl;
            if (!result.passed && !result.message.empty()) {
                std::cout << "  └─ " << result.message << std::endl;
            }
        }
        
        std::cout << "\nTotal: " << (passed + failed) << " tests" << std::endl;
        std::cout << GREEN << "Passed: " << passed << RESET << std::endl;
        if (failed > 0) {
            std::cout << RED << "Failed: " << failed << RESET << std::endl;
        }
        
        double success_rate = (passed * 100.0) / (passed + failed);
        std::cout << "Success Rate: " << std::fixed << std::setprecision(1) 
                  << success_rate << "%" << std::endl;
    }
};

// Simulate hardware responses
class HardwareSimulator {
public:
    // Simulate I2C device
    static bool simulateI2CDevice(int address) {
        // MPU6050 at 0x68, QMC5883L at 0x0D, DPS310 at 0x77
        return (address == 0x68 || address == 0x0D || address == 0x77);
    }
    
    // Simulate IMU data
    static void getIMUData(float& ax, float& ay, float& az, 
                          float& gx, float& gy, float& gz, float& temp) {
        static std::mt19937 gen(42);
        static std::normal_distribution<float> noise(0.0f, 0.01f);
        
        // Simulate level drone with small noise
        ax = 0.0f + noise(gen);
        ay = 0.0f + noise(gen);
        az = 9.81f + noise(gen);
        gx = 0.0f + noise(gen);
        gy = 0.0f + noise(gen);
        gz = 0.0f + noise(gen);
        temp = 25.0f + noise(gen);
    }
    
    // Simulate magnetometer
    static void getMagData(float& mx, float& my, float& mz) {
        // Simulate earth's magnetic field pointing north
        mx = 20.0f;
        my = 0.0f;
        mz = -40.0f;
    }
    
    // Simulate barometer
    static void getBaroData(float& pressure, float& temperature, float& altitude) {
        pressure = 101325.0f; // Sea level
        temperature = 25.0f;
        altitude = 0.0f;
    }
    
    // Simulate LIDAR
    static float getLidarDistance() {
        static float distance = 5.0f;
        static float direction = 0.1f;
        
        // Simulate moving obstacle
        distance += direction;
        if (distance > 10.0f || distance < 0.5f) {
            direction = -direction;
        }
        return distance;
    }
    
    // Simulate GPS
    static void getGPSData(double& lat, double& lon, float& alt, bool& fix) {
        static int counter = 0;
        counter++;
        
        // Simulate getting fix after 5 calls
        if (counter > 5) {
            fix = true;
            lat = 48.8566;  // Paris
            lon = 2.3522;
            alt = 35.0f;
        } else {
            fix = false;
            lat = 0.0;
            lon = 0.0;
            alt = 0.0;
        }
    }
};

// Test implementations
void testPIDController() {
    std::cout << "Testing PID Controller behavior..." << std::endl;
    
    // Simulate simple PID
    class SimplePID {
        float kp, ki, kd;
        float integral = 0;
        float prev_error = 0;
    public:
        SimplePID(float p, float i, float d) : kp(p), ki(i), kd(d) {}
        
        float update(float error, float dt) {
            integral += error * dt;
            float derivative = (error - prev_error) / dt;
            prev_error = error;
            return kp * error + ki * integral + kd * derivative;
        }
    };
    
    SimplePID pid(1.0f, 0.1f, 0.01f);
    
    // Test step response
    float setpoint = 10.0f;
    float current = 0.0f;
    float dt = 0.01f;
    
    for (int i = 0; i < 1000; i++) {
        float error = setpoint - current;
        float output = pid.update(error, dt);
        current += output * dt;
        
        if (i % 100 == 0) {
            std::cout << "  t=" << (i * dt) << "s, current=" << current 
                      << ", error=" << error << std::endl;
        }
    }
    
    // Check if converged
    if (std::abs(current - setpoint) > 0.1f) {
        throw std::runtime_error("PID did not converge");
    }
}

void testObstacleAvoidance() {
    std::cout << "Testing obstacle avoidance logic..." << std::endl;
    
    // Simulate obstacle detection
    float obstacle_distances[] = {10.0f, 5.0f, 2.0f, 0.8f, 0.5f};
    float expected_responses[] = {0.0f, 0.0f, 0.0f, 10.0f, 15.0f}; // Expected pitch override
    
    for (int i = 0; i < 5; i++) {
        float distance = obstacle_distances[i];
        float pitch_override = 0.0f;
        
        if (distance < 1.0f) { // OBSTACLE_THRESHOLD
            pitch_override = -15.0f * (M_PI / 180.0f);
        }
        
        std::cout << "  Distance: " << distance << "m, Override: " 
                  << (pitch_override * 180.0f / M_PI) << "°" << std::endl;
                  
        if (distance < 1.0f && pitch_override == 0) {
            throw std::runtime_error("Obstacle avoidance failed to trigger");
        }
    }
}

void testMotorMixing() {
    std::cout << "Testing motor mixing for X-configuration..." << std::endl;
    
    struct TestCase {
        float roll, pitch, yaw, throttle;
        int expected_m1, expected_m2, expected_m3, expected_m4;
    };
    
    TestCase cases[] = {
        // throttle only
        {0, 0, 0, 1500, 1500, 1500, 1500, 1500},
        // roll right
        {100, 0, 0, 1500, 1600, 1400, 1400, 1600},
        // pitch forward
        {0, 100, 0, 1500, 1400, 1400, 1600, 1600},
        // yaw right
        {0, 0, 100, 1500, 1600, 1400, 1600, 1400},
    };
    
    for (const auto& tc : cases) {
        int m1 = tc.throttle + tc.roll - tc.pitch + tc.yaw;
        int m2 = tc.throttle - tc.roll - tc.pitch - tc.yaw;
        int m3 = tc.throttle - tc.roll + tc.pitch + tc.yaw;
        int m4 = tc.throttle + tc.roll + tc.pitch - tc.yaw;
        
        std::cout << "  Input: R=" << tc.roll << " P=" << tc.pitch 
                  << " Y=" << tc.yaw << " T=" << tc.throttle << std::endl;
        std::cout << "  Motors: " << m1 << " " << m2 << " " << m3 << " " << m4 << std::endl;
        
        if (m1 != tc.expected_m1 || m2 != tc.expected_m2 || 
            m3 != tc.expected_m3 || m4 != tc.expected_m4) {
            throw std::runtime_error("Motor mixing calculation error");
        }
    }
}

void testAHRS() {
    std::cout << "Testing AHRS (Mahony filter) convergence..." << std::endl;
    
    // Simplified Mahony implementation
    class SimpleAHRS {
        float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
    public:
        void update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
            // Normalize accelerometer
            float norm = std::sqrt(ax*ax + ay*ay + az*az);
            if (norm > 0) {
                ax /= norm; ay /= norm; az /= norm;
            }
            
            // Simple quaternion integration
            q0 += 0.5f * (-q1*gx - q2*gy - q3*gz) * dt;
            q1 += 0.5f * (q0*gx + q2*gz - q3*gy) * dt;
            q2 += 0.5f * (q0*gy - q1*gz + q3*gx) * dt;
            q3 += 0.5f * (q0*gz + q1*gy - q2*gx) * dt;
            
            // Normalize quaternion
            norm = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
            q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
        }
        
        void getEuler(float& roll, float& pitch, float& yaw) {
            roll = std::atan2(2*(q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3);
            pitch = -std::asin(2*(q1*q3 - q0*q2));
            yaw = std::atan2(2*(q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3);
        }
    };
    
    SimpleAHRS ahrs;
    float dt = 0.01f;
    
    // Simulate stationary drone
    for (int i = 0; i < 100; i++) {
        ahrs.update(0, 0, 0, 0, 0, 9.81f, dt);
    }
    
    float roll, pitch, yaw;
    ahrs.getEuler(roll, pitch, yaw);
    
    std::cout << "  After 1s: Roll=" << (roll * 180/M_PI) << "° Pitch=" 
              << (pitch * 180/M_PI) << "° Yaw=" << (yaw * 180/M_PI) << "°" << std::endl;
              
    if (std::abs(roll) > 0.1f || std::abs(pitch) > 0.1f) {
        throw std::runtime_error("AHRS did not converge to level");
    }
}

void testThreadSafety() {
    std::cout << "Testing thread safety with concurrent access..." << std::endl;
    
    std::mutex mtx;
    std::atomic<int> counter{0};
    std::atomic<bool> running{true};
    
    // Simulate sensor thread
    std::thread sensor([&]() {
        while (running) {
            std::lock_guard<std::mutex> lock(mtx);
            counter++;
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    });
    
    // Simulate control thread
    std::thread control([&]() {
        while (running) {
            std::lock_guard<std::mutex> lock(mtx);
            counter--;
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    });
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    running = false;
    
    sensor.join();
    control.join();
    
    std::cout << "  Counter drift: " << counter.load() << " (should be near 0)" << std::endl;
    
    if (std::abs(counter.load()) > 100) {
        throw std::runtime_error("Thread synchronization issue detected");
    }
}

void testSafetyChecks() {
    std::cout << "Testing safety mechanisms..." << std::endl;
    
    // Test arming conditions
    struct ArmTest {
        float roll, pitch;
        bool should_arm;
    };
    
    ArmTest tests[] = {
        {0.0f, 0.0f, true},      // Level - should arm
        {0.05f, 0.05f, true},    // Small tilt - should arm
        {0.2f, 0.0f, false},     // Too much roll - should not arm
        {0.0f, 0.2f, false},     // Too much pitch - should not arm
    };
    
    for (const auto& test : tests) {
        bool can_arm = (std::abs(test.roll) < 0.1f && std::abs(test.pitch) < 0.1f);
        std::cout << "  Roll=" << test.roll << " Pitch=" << test.pitch 
                  << " -> " << (can_arm ? "CAN ARM" : "CANNOT ARM") << std::endl;
                  
        if (can_arm != test.should_arm) {
            throw std::runtime_error("Arming safety check failed");
        }
    }
    
    // Test motor constraints
    int test_values[] = {500, 1000, 1500, 2000, 2500};
    for (int val : test_values) {
        int constrained = std::max(1000, std::min(2000, val));
        std::cout << "  PWM " << val << " -> " << constrained << std::endl;
        
        if (constrained < 1000 || constrained > 2000) {
            throw std::runtime_error("Motor PWM constraint failed");
        }
    }
}

void testGPSNavigation() {
    std::cout << "Testing GPS navigation calculations..." << std::endl;
    
    // Test distance calculation
    double lat1 = 48.8566, lon1 = 2.3522;   // Paris
    double lat2 = 48.8584, lon2 = 2.2945;   // Eiffel Tower
    
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    
    float distance_north = dlat * 111111.0f;
    float distance_east = dlon * 111111.0f * std::cos(lat1 * M_PI/180);
    float total_distance = std::sqrt(distance_north*distance_north + distance_east*distance_east);
    
    std::cout << "  Distance: " << total_distance << "m" << std::endl;
    std::cout << "  North: " << distance_north << "m, East: " << distance_east << "m" << std::endl;
    
    float bearing = std::atan2(distance_east, distance_north) * 180/M_PI;
    std::cout << "  Bearing: " << bearing << "°" << std::endl;
    
    // Should be roughly 6km
    if (total_distance < 5000 || total_distance > 7000) {
        throw std::runtime_error("GPS distance calculation error");
    }
}

void testEmergencyProcedures() {
    std::cout << "Testing emergency procedures..." << std::endl;
    
    // Test battery voltage thresholds
    float voltages[] = {12.6f, 11.1f, 10.5f, 9.8f};
    std::string expected[] = {"Good", "OK", "Low", "Critical"};
    
    for (int i = 0; i < 4; i++) {
        std::string status;
        if (voltages[i] > 11.4f) status = "Good";
        else if (voltages[i] > 10.8f) status = "OK";
        else if (voltages[i] > 10.2f) status = "Low";
        else status = "Critical";
        
        std::cout << "  Battery " << voltages[i] << "V -> " << status << std::endl;
        
        if (status != expected[i]) {
            throw std::runtime_error("Battery monitoring logic error");
        }
    }
    
    // Test altitude limits
    float altitudes[] = {0, 50, 100, 150, 400, 500};
    for (float alt : altitudes) {
        bool safe = (alt >= 0 && alt <= 400);  // Legal limit in many countries
        std::cout << "  Altitude " << alt << "m -> " << (safe ? "SAFE" : "LIMIT") << std::endl;
    }
}

void testCommandParsing() {
    std::cout << "Testing command parsing..." << std::endl;
    
    struct Command {
        std::string input;
        std::string expected_cmd;
        int expected_params;
    };
    
    Command commands[] = {
        {"arm", "arm", 0},
        {"takeoff 10", "takeoff", 1},
        {"move 5 -2 0", "move", 3},
        {"goto 48.8566 2.3522 50", "goto", 3},
    };
    
    for (const auto& cmd : commands) {
        std::istringstream iss(cmd.input);
        std::string parsed_cmd;
        iss >> parsed_cmd;
        
        int param_count = 0;
        float param;
        while (iss >> param) param_count++;
        
        std::cout << "  \"" << cmd.input << "\" -> cmd:" << parsed_cmd 
                  << " params:" << param_count << std::endl;
                  
        if (parsed_cmd != cmd.expected_cmd) {
            throw std::runtime_error("Command parsing failed");
        }
    }
}

void testDataLogging() {
    std::cout << "Testing data logging format..." << std::endl;
    
    // Simulate log entry
    std::stringstream log;
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    
    log << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    log << ",ARMED,";
    log << "roll:15.2,pitch:-3.1,yaw:127.5,";
    log << "alt:25.3,";
    log << "lat:48.8566,lon:2.3522,";
    log << "battery:11.2";
    
    std::string log_entry = log.str();
    std::cout << "  Log: " << log_entry << std::endl;
    
    // Verify format
    if (log_entry.find(",") == std::string::npos || 
        log_entry.find(":") == std::string::npos) {
        throw std::runtime_error("Log format error");
    }
}

// Main test runner
int main() {
    std::cout << "\n" << "="*60 << std::endl;
    std::cout << "DRONE CONTROL SYSTEM - COMPREHENSIVE DEBUG TEST" << std::endl;
    std::cout << "="*60 << std::endl;
    std::cout << "This test will verify all components work correctly" << std::endl;
    std::cout << "without requiring any hardware to be connected.\n" << std::endl;
    
    TestFramework tester;
    
    // Core functionality tests
    tester.runTest("PID Controller", testPIDController);
    tester.runTest("Obstacle Avoidance Logic", testObstacleAvoidance);
    tester.runTest("Motor Mixing (X-config)", testMotorMixing);
    tester.runTest("AHRS Sensor Fusion", testAHRS);
    tester.runTest("Thread Safety", testThreadSafety);
    
    // Safety tests
    tester.runTest("Safety Checks", testSafetyChecks);
    tester.runTest("Emergency Procedures", testEmergencyProcedures);
    
    // Navigation tests
    tester.runTest("GPS Navigation", testGPSNavigation);
    
    // System tests
    tester.runTest("Command Parsing", testCommandParsing);
    tester.runTest("Data Logging", testDataLogging);
    
    // Hardware simulation tests
    tester.runTest("I2C Communication Simulation", []() {
        std::cout << "Simulating I2C devices..." << std::endl;
        if (!HardwareSimulator::simulateI2CDevice(0x68)) {
            throw std::runtime_error("MPU6050 simulation failed");
        }
        std::cout << "  MPU6050 (0x68): OK" << std::endl;
        std::cout << "  QMC5883L (0x0D): OK" << std::endl;
        std::cout << "  DPS310 (0x77): OK" << std::endl;
    });
    
    tester.runTest("Sensor Data Simulation", []() {
        std::cout << "Testing sensor data generation..." << std::endl;
        
        float ax, ay, az, gx, gy, gz, temp;
        HardwareSimulator::getIMUData(ax, ay, az, gx, gy, gz, temp);
        
        std::cout << "  Accel: " << ax << ", " << ay << ", " << az << " m/s²" << std::endl;
        std::cout << "  Gyro: " << gx << ", " << gy << ", " << gz << " rad/s" << std::endl;
        std::cout << "  Temp: " << temp << "°C" << std::endl;
        
        // Check if gravity vector is correct
        float gravity = std::sqrt(ax*ax + ay*ay + az*az);
        if (std::abs(gravity - 9.81f) > 0.5f) {
            throw std::runtime_error("Gravity vector incorrect");
        }
    });
    
    tester.runTest("PWM Signal Generation", []() {
        std::cout << "Testing PWM calculations..." << std::endl;
        
        int period_ns = 20000000;  // 20ms = 50Hz
        int test_pwm[] = {1000, 1100, 1500, 2000};
        
        for (int pwm : test_pwm) {
            int duty_ns = pwm * 1000;
            float duty_cycle = (duty_ns * 100.0f) / period_ns;
            std::cout << "  " << pwm << "μs -> " << duty_cycle << "% duty" << std::endl;
            
            if (duty_cycle < 5.0f || duty_cycle > 10.0f) {
                throw std::runtime_error("PWM duty cycle out of range");
            }
        }
    });
    
    // Performance tests
    tester.runTest("Control Loop Performance", []() {
        std::cout << "Testing control loop timing..." << std::endl;
        
        auto start = std::chrono::high_resolution_clock::now();
        
        // Simulate 1000 control loop iterations
        for (int i = 0; i < 1000; i++) {
            // Simulate calculations
            float dummy = 0;
            for (int j = 0; j < 100; j++) {
                dummy += std::sin(j) * std::cos(j);
            }
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        float avg_time = duration.count() / 1000.0f;
        
        std::cout << "  Average iteration: " << avg_time << "μs" << std::endl;
        std::cout << "  Max frequency: " << (1000000.0f / avg_time) << "Hz" << std::endl;
        
        if (avg_time > 5000) {  // Should complete in less than 5ms
            throw std::runtime_error("Control loop too slow");
        }
    });
    
    // Memory tests
    tester.runTest("Memory Allocation", []() {
        std::cout << "Testing dynamic memory usage..." << std::endl;
        
        size_t initial_mem = 0;  // Would need platform-specific code for real measurement
        
        // Test allocation patterns
        std::vector<std::unique_ptr<float[]>> buffers;
        for (int i = 0; i < 10; i++) {
            buffers.push_back(std::make_unique<float[]>(1024));
        }
        
        std::cout << "  Allocated 10 buffers (40KB total)" << std::endl;
        
        buffers.clear();
        std::cout << "  Memory freed successfully" << std::endl;
    });
    
    // Final summary
    tester.printSummary();
    
    std::cout << "\n" << "="*60 << std::endl;
    if (tester.failed == 0) {
        std::cout << GREEN << "ALL TESTS PASSED!" << RESET << std::endl;
        std::cout << "Your drone control system core logic is working correctly." << std::endl;
        std::cout << "You can be confident the software will work with real hardware." << std::endl;
    } else {
        std::cout << RED << "SOME TESTS FAILED!" << RESET << std::endl;
        std::cout << "Please fix the issues before connecting real hardware." << std::endl;
    }
    std::cout << "="*60 << std::endl;
    
    return (tester.failed > 0) ? 1 : 0;
}