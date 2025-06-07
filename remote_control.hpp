#ifndef REMOTE_CONTROL_HPP
#define REMOTE_CONTROL_HPP

#include <thread>
#include <atomic>
#include <string>
#include <functional>
#include <unordered_map>
#include <mutex>
#include <queue>
#include <chrono>
#include <memory>
#include <condition_variable>
#include <sstream>
#include <iomanip>
#include <openssl/ssl.h>
#include <openssl/err.h>
#include <openssl/sha.h>
#include <openssl/rand.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <iostream>
#include "nlohmann/json.hpp"

using json = nlohmann::json;

// WebSocket opcodes
enum WSOpcode {
    WS_CONTINUATION = 0x0,
    WS_TEXT = 0x1,
    WS_BINARY = 0x2,
    WS_CLOSE = 0x8,
    WS_PING = 0x9,
    WS_PONG = 0xA
};

// Message types for drone communication
enum MessageType {
    MSG_COMMAND = 1,
    MSG_TELEMETRY = 2,
    MSG_VIDEO = 3,
    MSG_WAYPOINT = 4,
    MSG_CONFIG = 5,
    MSG_EMERGENCY = 6,
    MSG_AUTH = 7,
    MSG_HEARTBEAT = 8,
    MSG_LOG = 9,
    MSG_FILE_TRANSFER = 10
};

// Connection states
enum ConnectionState {
    DISCONNECTED,
    CONNECTING,
    AUTHENTICATING,
    CONNECTED,
    RECONNECTING
};

// Telemetry data structure
struct TelemetryData {
    float roll, pitch, yaw;
    float altitude, speed;
    float battery_voltage;
    float latitude, longitude;
    bool gps_fix;
    int satellites;
    bool armed;
    bool in_flight;
    float temperatures[4];  // Motor temps
    float current_draw;
    int signal_strength;
    std::string flight_mode;
    std::vector<float> obstacle_distances;  // 360° lidar data
    uint64_t timestamp;
};

// Waypoint for autonomous flight
struct Waypoint {
    double latitude;
    double longitude;
    float altitude;
    float speed;
    bool hover;
    int hover_time;
};

// WebSocket frame
struct WSFrame {
    bool fin;
    WSOpcode opcode;
    bool masked;
    uint64_t payload_length;
    std::vector<uint8_t> payload;
};

// Base64 encoding for WebSocket handshake
std::string base64_encode(const unsigned char* data, size_t len) {
    static const char* charset = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string result;
    
    for (size_t i = 0; i < len; i += 3) {
        int b = (data[i] & 0xFC) >> 2;
        result += charset[b];
        
        b = (data[i] & 0x03) << 4;
        if (i + 1 < len) b |= (data[i + 1] & 0xF0) >> 4;
        result += charset[b];
        
        if (i + 1 < len) {
            b = (data[i + 1] & 0x0F) << 2;
            if (i + 2 < len) b |= (data[i + 2] & 0xC0) >> 6;
            result += charset[b];
        } else {
            result += '=';
        }
        
        if (i + 2 < len) {
            b = data[i + 2] & 0x3F;
            result += charset[b];
        } else {
            result += '=';
        }
    }
    
    return result;
}

// Remote Control Client (runs on drone)
class RemoteControlClient {
private:
    std::string server_host;
    int server_port;
    std::string drone_id;
    std::string auth_token;
    
    int socket_fd = -1;
    SSL_CTX* ssl_ctx = nullptr;
    SSL* ssl = nullptr;
    
    std::atomic<ConnectionState> state{DISCONNECTED};
    std::atomic<bool> running{false};
    
    std::thread connection_thread;
    std::thread receive_thread;
    std::thread telemetry_thread;
    std::thread heartbeat_thread;
    
    std::mutex send_mutex;
    std::queue<std::string> send_queue;
    std::condition_variable send_cv;
    
    std::function<void(const json&)> command_handler;
    std::function<TelemetryData()> telemetry_provider;
    std::function<void(ConnectionState)> connection_callback;
    
    std::chrono::steady_clock::time_point last_heartbeat;
    std::chrono::steady_clock::time_point last_telemetry;
    
    // Reconnection parameters
    int reconnect_delay = 1000;  // ms
    const int max_reconnect_delay = 30000;  // ms
    
    // Initialize SSL
    bool initSSL() {
        SSL_library_init();
        SSL_load_error_strings();
        OpenSSL_add_all_algorithms();
        
        ssl_ctx = SSL_CTX_new(TLS_client_method());
        if (!ssl_ctx) {
            std::cerr << "Failed to create SSL context" << std::endl;
            return false;
        }
        
        // Set SSL options for security
        SSL_CTX_set_options(ssl_ctx, SSL_OP_NO_SSLv2 | SSL_OP_NO_SSLv3);
        
        return true;
    }
    
    // Connect to server
    bool connectToServer() {
        // Resolve hostname
        struct hostent* host = gethostbyname(server_host.c_str());
        if (!host) {
            std::cerr << "Failed to resolve hostname: " << server_host << std::endl;
            return false;
        }
        
        // Create socket
        socket_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd < 0) {
            std::cerr << "Failed to create socket" << std::endl;
            return false;
        }
        
        // Set socket options
        int opt = 1;
        setsockopt(socket_fd, SOL_SOCKET, SO_KEEPALIVE, &opt, sizeof(opt));
        
        // Set non-blocking mode
        int flags = fcntl(socket_fd, F_GETFL, 0);
        fcntl(socket_fd, F_SETFL, flags | O_NONBLOCK);
        
        // Connect
        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(server_port);
        memcpy(&server_addr.sin_addr.s_addr, host->h_addr, host->h_length);
        
        int result = connect(socket_fd, (struct sockaddr*)&server_addr, sizeof(server_addr));
        if (result < 0 && errno != EINPROGRESS) {
            std::cerr << "Failed to connect: " << strerror(errno) << std::endl;
            close(socket_fd);
            return false;
        }
        
        // Wait for connection with timeout
        fd_set write_fds;
        FD_ZERO(&write_fds);
        FD_SET(socket_fd, &write_fds);
        
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        
        result = select(socket_fd + 1, nullptr, &write_fds, nullptr, &timeout);
        if (result <= 0) {
            std::cerr << "Connection timeout" << std::endl;
            close(socket_fd);
            return false;
        }
        
        // Check connection status
        int error = 0;
        socklen_t len = sizeof(error);
        getsockopt(socket_fd, SOL_SOCKET, SO_ERROR, &error, &len);
        if (error != 0) {
            std::cerr << "Connection failed: " << strerror(error) << std::endl;
            close(socket_fd);
            return false;
        }
        
        // Setup SSL
        ssl = SSL_new(ssl_ctx);
        SSL_set_fd(ssl, socket_fd);
        
        if (SSL_connect(ssl) <= 0) {
            std::cerr << "SSL handshake failed" << std::endl;
            SSL_free(ssl);
            close(socket_fd);
            return false;
        }
        
        std::cout << "Connected to server with SSL" << std::endl;
        return true;
    }
    
    // WebSocket handshake
    bool performWebSocketHandshake() {
        // Generate WebSocket key
        unsigned char ws_key[16];
        RAND_bytes(ws_key, 16);
        std::string ws_key_base64 = base64_encode(ws_key, 16);
        
        // Build handshake request
        std::stringstream request;
        request << "GET /drone HTTP/1.1\r\n";
        request << "Host: " << server_host << ":" << server_port << "\r\n";
        request << "Upgrade: websocket\r\n";
        request << "Connection: Upgrade\r\n";
        request << "Sec-WebSocket-Key: " << ws_key_base64 << "\r\n";
        request << "Sec-WebSocket-Version: 13\r\n";
        request << "X-Drone-ID: " << drone_id << "\r\n";
        request << "X-Auth-Token: " << auth_token << "\r\n";
        request << "\r\n";
        
        // Send request
        if (SSL_write(ssl, request.str().c_str(), request.str().length()) <= 0) {
            std::cerr << "Failed to send WebSocket handshake" << std::endl;
            return false;
        }
        
        // Read response
        char buffer[1024];
        int bytes = SSL_read(ssl, buffer, sizeof(buffer) - 1);
        if (bytes <= 0) {
            std::cerr << "Failed to read WebSocket handshake response" << std::endl;
            return false;
        }
        
        buffer[bytes] = '\0';
        std::string response(buffer);
        
        // Check for successful upgrade
        if (response.find("HTTP/1.1 101") == std::string::npos) {
            std::cerr << "WebSocket handshake failed: " << response << std::endl;
            return false;
        }
        
        std::cout << "WebSocket handshake successful" << std::endl;
        return true;
    }
    
    // Send WebSocket frame
    bool sendWSFrame(WSOpcode opcode, const std::string& data) {
        std::vector<uint8_t> frame;
        
        // FIN = 1, RSV = 0, Opcode
        frame.push_back(0x80 | opcode);
        
        // Mask = 1 (client must mask), Payload length
        size_t payload_len = data.length();
        if (payload_len < 126) {
            frame.push_back(0x80 | payload_len);
        } else if (payload_len < 65536) {
            frame.push_back(0x80 | 126);
            frame.push_back((payload_len >> 8) & 0xFF);
            frame.push_back(payload_len & 0xFF);
        } else {
            frame.push_back(0x80 | 127);
            for (int i = 7; i >= 0; i--) {
                frame.push_back((payload_len >> (i * 8)) & 0xFF);
            }
        }
        
        // Masking key
        uint8_t mask[4];
        RAND_bytes(mask, 4);
        for (int i = 0; i < 4; i++) {
            frame.push_back(mask[i]);
        }
        
        // Masked payload
        for (size_t i = 0; i < data.length(); i++) {
            frame.push_back(data[i] ^ mask[i % 4]);
        }
        
        // Send frame
        std::lock_guard<std::mutex> lock(send_mutex);
        int sent = SSL_write(ssl, frame.data(), frame.size());
        return sent == static_cast<int>(frame.size());
    }
    
    // Receive WebSocket frame
    bool receiveWSFrame(WSFrame& frame) {
        uint8_t header[2];
        if (SSL_read(ssl, header, 2) != 2) {
            return false;
        }
        
        frame.fin = (header[0] & 0x80) != 0;
        frame.opcode = static_cast<WSOpcode>(header[0] & 0x0F);
        frame.masked = (header[1] & 0x80) != 0;
        
        uint64_t payload_len = header[1] & 0x7F;
        if (payload_len == 126) {
            uint8_t len_bytes[2];
            if (SSL_read(ssl, len_bytes, 2) != 2) return false;
            payload_len = (len_bytes[0] << 8) | len_bytes[1];
        } else if (payload_len == 127) {
            uint8_t len_bytes[8];
            if (SSL_read(ssl, len_bytes, 8) != 8) return false;
            payload_len = 0;
            for (int i = 0; i < 8; i++) {
                payload_len = (payload_len << 8) | len_bytes[i];
            }
        }
        
        frame.payload_length = payload_len;
        
        uint8_t mask[4] = {0};
        if (frame.masked) {
            if (SSL_read(ssl, mask, 4) != 4) return false;
        }
        
        frame.payload.resize(payload_len);
        if (payload_len > 0) {
            if (SSL_read(ssl, frame.payload.data(), payload_len) != static_cast<int>(payload_len)) {
                return false;
            }
            
            if (frame.masked) {
                for (size_t i = 0; i < payload_len; i++) {
                    frame.payload[i] ^= mask[i % 4];
                }
            }
        }
        
        return true;
    }
    
    // Connection thread
    void connectionLoop() {
        while (running) {
            if (state == DISCONNECTED || state == RECONNECTING) {
                state = CONNECTING;
                
                if (connection_callback) {
                    connection_callback(CONNECTING);
                }
                
                if (connectToServer() && performWebSocketHandshake()) {
                    state = CONNECTED;
                    reconnect_delay = 1000;  // Reset delay
                    last_heartbeat = std::chrono::steady_clock::now();
                    
                    if (connection_callback) {
                        connection_callback(CONNECTED);
                    }
                    
                    // Send initial telemetry
                    sendTelemetry();
                    
                } else {
                    state = RECONNECTING;
                    
                    if (connection_callback) {
                        connection_callback(RECONNECTING);
                    }
                    
                    std::cout << "Reconnecting in " << reconnect_delay << "ms..." << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_delay));
                    
                    // Exponential backoff
                    reconnect_delay = std::min(reconnect_delay * 2, max_reconnect_delay);
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
    
    // Receive thread
    void receiveLoop() {
        while (running) {
            if (state == CONNECTED) {
                WSFrame frame;
                if (receiveWSFrame(frame)) {
                    switch (frame.opcode) {
                        case WS_TEXT:
                        case WS_BINARY:
                            handleMessage(std::string(frame.payload.begin(), frame.payload.end()));
                            break;
                            
                        case WS_PING:
                            sendWSFrame(WS_PONG, std::string(frame.payload.begin(), frame.payload.end()));
                            break;
                            
                        case WS_CLOSE:
                            std::cout << "Server closed connection" << std::endl;
                            disconnect();
                            break;
                            
                        default:
                            break;
                    }
                } else if (state == CONNECTED) {
                    std::cerr << "Failed to receive frame" << std::endl;
                    disconnect();
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
    
    // Handle incoming message
    void handleMessage(const std::string& data) {
        try {
            json msg = json::parse(data);
            int type = msg["type"];
            
            switch (type) {
                case MSG_COMMAND:
                    if (command_handler) {
                        command_handler(msg["data"]);
                    }
                    break;
                    
                case MSG_WAYPOINT:
                    handleWaypoint(msg["data"]);
                    break;
                    
                case MSG_CONFIG:
                    handleConfig(msg["data"]);
                    break;
                    
                case MSG_EMERGENCY:
                    handleEmergency(msg["data"]);
                    break;
                    
                case MSG_HEARTBEAT:
                    last_heartbeat = std::chrono::steady_clock::now();
                    break;
                    
                default:
                    std::cout << "Unknown message type: " << type << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "Error handling message: " << e.what() << std::endl;
        }
    }
    
    // Send telemetry
    void sendTelemetry() {
        if (state != CONNECTED || !telemetry_provider) return;
        
        TelemetryData telemetry = telemetry_provider();
        
        json msg;
        msg["type"] = MSG_TELEMETRY;
        msg["drone_id"] = drone_id;
        msg["data"]["roll"] = telemetry.roll;
        msg["data"]["pitch"] = telemetry.pitch;
        msg["data"]["yaw"] = telemetry.yaw;
        msg["data"]["altitude"] = telemetry.altitude;
        msg["data"]["speed"] = telemetry.speed;
        msg["data"]["battery_voltage"] = telemetry.battery_voltage;
        msg["data"]["latitude"] = telemetry.latitude;
        msg["data"]["longitude"] = telemetry.longitude;
        msg["data"]["gps_fix"] = telemetry.gps_fix;
        msg["data"]["satellites"] = telemetry.satellites;
        msg["data"]["armed"] = telemetry.armed;
        msg["data"]["in_flight"] = telemetry.in_flight;
        msg["data"]["temperatures"] = telemetry.temperatures;
        msg["data"]["current_draw"] = telemetry.current_draw;
        msg["data"]["signal_strength"] = telemetry.signal_strength;
        msg["data"]["flight_mode"] = telemetry.flight_mode;
        msg["data"]["obstacle_distances"] = telemetry.obstacle_distances;
        msg["data"]["timestamp"] = telemetry.timestamp;
        
        sendWSFrame(WS_TEXT, msg.dump());
    }
    
    // Telemetry thread
    void telemetryLoop() {
        while (running) {
            if (state == CONNECTED) {
                sendTelemetry();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 10Hz telemetry
        }
    }
    
    // Heartbeat thread
    void heartbeatLoop() {
        while (running) {
            if (state == CONNECTED) {
                json msg;
                msg["type"] = MSG_HEARTBEAT;
                msg["drone_id"] = drone_id;
                msg["timestamp"] = std::chrono::system_clock::now().time_since_epoch().count();
                
                sendWSFrame(WS_TEXT, msg.dump());
                
                // Check for heartbeat timeout
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_heartbeat).count();
                if (elapsed > 30) {
                    std::cerr << "Heartbeat timeout - disconnecting" << std::endl;
                    disconnect();
                }
            }
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }
    
    // Handle waypoint
    void handleWaypoint(const json& data) {
        std::vector<Waypoint> waypoints;
        for (const auto& wp : data["waypoints"]) {
            Waypoint waypoint;
            waypoint.latitude = wp["latitude"];
            waypoint.longitude = wp["longitude"];
            waypoint.altitude = wp["altitude"];
            waypoint.speed = wp.value("speed", 5.0f);
            waypoint.hover = wp.value("hover", false);
            waypoint.hover_time = wp.value("hover_time", 0);
            waypoints.push_back(waypoint);
        }
        
        // Process waypoints (implement autonomous flight)
        std::cout << "Received " << waypoints.size() << " waypoints" << std::endl;
    }
    
    // Handle configuration
    void handleConfig(const json& data) {
        std::cout << "Received configuration update" << std::endl;
        // Apply configuration changes
    }
    
    // Handle emergency
    void handleEmergency(const json& data) {
        std::string action = data["action"];
        std::cout << "EMERGENCY: " << action << std::endl;
        
        if (action == "RTH") {
            // Return to home
        } else if (action == "LAND") {
            // Emergency land
        } else if (action == "KILL") {
            // Kill motors
        }
    }
    
    // Disconnect
    void disconnect() {
        state = DISCONNECTED;
        
        if (ssl) {
            SSL_shutdown(ssl);
            SSL_free(ssl);
            ssl = nullptr;
        }
        
        if (socket_fd >= 0) {
            close(socket_fd);
            socket_fd = -1;
        }
        
        if (connection_callback) {
            connection_callback(DISCONNECTED);
        }
    }
    
public:
    RemoteControlClient(const std::string& host, int port, const std::string& id, const std::string& token)
        : server_host(host), server_port(port), drone_id(id), auth_token(token) {
        
        if (!initSSL()) {
            throw std::runtime_error("Failed to initialize SSL");
        }
    }
    
    ~RemoteControlClient() {
        stop();
        
        if (ssl_ctx) {
            SSL_CTX_free(ssl_ctx);
        }
    }
    
    // Start client
    void start() {
        running = true;
        connection_thread = std::thread(&RemoteControlClient::connectionLoop, this);
        receive_thread = std::thread(&RemoteControlClient::receiveLoop, this);
        telemetry_thread = std::thread(&RemoteControlClient::telemetryLoop, this);
        heartbeat_thread = std::thread(&RemoteControlClient::heartbeatLoop, this);
        
        std::cout << "Remote control client started" << std::endl;
    }
    
    // Stop client
    void stop() {
        running = false;
        
        if (state == CONNECTED) {
            sendWSFrame(WS_CLOSE, "");
        }
        
        disconnect();
        
        if (connection_thread.joinable()) connection_thread.join();
        if (receive_thread.joinable()) receive_thread.join();
        if (telemetry_thread.joinable()) telemetry_thread.join();
        if (heartbeat_thread.joinable()) heartbeat_thread.join();
    }
    
    // Set handlers
    void setCommandHandler(std::function<void(const json&)> handler) {
        command_handler = handler;
    }
    
    void setTelemetryProvider(std::function<TelemetryData()> provider) {
        telemetry_provider = provider;
    }
    
    void setConnectionCallback(std::function<void(ConnectionState)> callback) {
        connection_callback = callback;
    }
    
    // Send log message
    void sendLog(const std::string& level, const std::string& message) {
        if (state != CONNECTED) return;
        
        json msg;
        msg["type"] = MSG_LOG;
        msg["drone_id"] = drone_id;
        msg["data"]["level"] = level;
        msg["data"]["message"] = message;
        msg["data"]["timestamp"] = std::chrono::system_clock::now().time_since_epoch().count();
        
        sendWSFrame(WS_TEXT, msg.dump());
    }
    
    // Get connection state
    ConnectionState getState() const { return state.load(); }
    
    // Send custom message
    void sendMessage(const json& message) {
        if (state != CONNECTED) return;
        sendWSFrame(WS_TEXT, message.dump());
    }
};

// Relay Server (runs on VPS/Cloud)
class RemoteControlRelay {
protected:
    struct Client {
        int socket_fd;
        SSL* ssl;
        std::string drone_id;
        std::chrono::steady_clock::time_point last_seen;
        bool is_drone;
        bool authenticated;
    };
    
    int server_fd = -1;
    int port;
    SSL_CTX* ssl_ctx = nullptr;
    
    std::atomic<bool> running{false};
    std::thread accept_thread;
    std::thread cleanup_thread;
    
    std::mutex clients_mutex;
    std::unordered_map<int, std::shared_ptr<Client>> clients;
    std::unordered_map<std::string, int> drone_map;  // drone_id -> socket_fd
    
    std::string relay_token;
    
    // Initialize SSL server
    bool initSSLServer() {
        SSL_library_init();
        SSL_load_error_strings();
        OpenSSL_add_all_algorithms();
        
        ssl_ctx = SSL_CTX_new(TLS_server_method());
        if (!ssl_ctx) {
            std::cerr << "Failed to create SSL context" << std::endl;
            return false;
        }
        
        // Load certificate and private key (you need to generate these)
        if (SSL_CTX_use_certificate_file(ssl_ctx, "server.crt", SSL_FILETYPE_PEM) <= 0) {
            std::cerr << "Failed to load certificate" << std::endl;
            return false;
        }
        
        if (SSL_CTX_use_PrivateKey_file(ssl_ctx, "server.key", SSL_FILETYPE_PEM) <= 0) {
            std::cerr << "Failed to load private key" << std::endl;
            return false;
        }
        
        return true;
    }
    
    // Accept connections
    void acceptLoop() {
        while (running) {
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            
            int client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_len);
            if (client_fd >= 0) {
                std::thread(&RemoteControlRelay::handleClient, this, client_fd).detach();
            }
        }
    }
    
    // Handle client
    void handleClient(int client_fd) {
        SSL* ssl = SSL_new(ssl_ctx);
        SSL_set_fd(ssl, client_fd);
        
        if (SSL_accept(ssl) <= 0) {
            SSL_free(ssl);
            close(client_fd);
            return;
        }
        
        // Read WebSocket handshake
        char buffer[4096];
        int bytes = SSL_read(ssl, buffer, sizeof(buffer) - 1);
        if (bytes <= 0) {
            SSL_free(ssl);
            close(client_fd);
            return;
        }
        
        buffer[bytes] = '\0';
        std::string request(buffer);
        
        // Parse headers
        std::string drone_id, auth_token;
        size_t pos = request.find("X-Drone-ID: ");
        if (pos != std::string::npos) {
            size_t end = request.find("\r\n", pos);
            drone_id = request.substr(pos + 12, end - pos - 12);
        }
        
        pos = request.find("X-Auth-Token: ");
        if (pos != std::string::npos) {
            size_t end = request.find("\r\n", pos);
            auth_token = request.substr(pos + 14, end - pos - 14);
        }
        
        // Authenticate
        if (auth_token != relay_token) {
            std::string response = "HTTP/1.1 401 Unauthorized\r\n\r\n";
            SSL_write(ssl, response.c_str(), response.length());
            SSL_free(ssl);
            close(client_fd);
            return;
        }
        
        // Accept WebSocket
        pos = request.find("Sec-WebSocket-Key: ");
        if (pos == std::string::npos) {
            SSL_free(ssl);
            close(client_fd);
            return;
        }
        
        size_t key_end = request.find("\r\n", pos);
        std::string ws_key = request.substr(pos + 19, key_end - pos - 19);
        
        // Generate accept key
        std::string magic = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
        std::string accept_string = ws_key + magic;
        
        unsigned char hash[SHA_DIGEST_LENGTH];
        SHA1(reinterpret_cast<const unsigned char*>(accept_string.c_str()), accept_string.length(), hash);
        std::string accept_key = base64_encode(hash, SHA_DIGEST_LENGTH);
        
        // Send response
        std::stringstream response;
        response << "HTTP/1.1 101 Switching Protocols\r\n";
        response << "Upgrade: websocket\r\n";
        response << "Connection: Upgrade\r\n";
        response << "Sec-WebSocket-Accept: " << accept_key << "\r\n";
        response << "\r\n";
        
        SSL_write(ssl, response.str().c_str(), response.str().length());
        
        // Add client
        auto client = std::make_shared<Client>();
        client->socket_fd = client_fd;
        client->ssl = ssl;
        client->drone_id = drone_id;
        client->last_seen = std::chrono::steady_clock::now();
        client->is_drone = !drone_id.empty();
        client->authenticated = true;
        
        {
            std::lock_guard<std::mutex> lock(clients_mutex);
            clients[client_fd] = client;
            if (client->is_drone) {
                drone_map[drone_id] = client_fd;
                std::cout << "Drone " << drone_id << " connected" << std::endl;
            } else {
                std::cout << "Control client connected" << std::endl;
            }
        }
        
        // Handle client messages
        handleClientMessages(client);
        
        // Cleanup
        {
            std::lock_guard<std::mutex> lock(clients_mutex);
            if (client->is_drone) {
                drone_map.erase(drone_id);
                std::cout << "Drone " << drone_id << " disconnected" << std::endl;
            }
            clients.erase(client_fd);
        }
        
        SSL_shutdown(ssl);
        SSL_free(ssl);
        close(client_fd);
    }
    
    // Handle client messages
    void handleClientMessages(std::shared_ptr<Client> client) {
        while (running) {
            WSFrame frame;
            if (!receiveWSFrame(client->ssl, frame)) {
                break;
            }
            
            client->last_seen = std::chrono::steady_clock::now();
            
            switch (frame.opcode) {
                case WS_TEXT:
                case WS_BINARY:
                    routeMessage(client, std::string(frame.payload.begin(), frame.payload.end()));
                    break;
                    
                case WS_PING:
                    sendWSFrame(client->ssl, WS_PONG, std::string(frame.payload.begin(), frame.payload.end()));
                    break;
                    
                case WS_CLOSE:
                    return;
                    
                default:
                    break;
            }
        }
    }
    
    // Route message between clients
    void routeMessage(std::shared_ptr<Client> sender, const std::string& data) {
        try {
            json msg = json::parse(data);
            
            if (sender->is_drone) {
                // Message from drone - broadcast to all control clients
                std::lock_guard<std::mutex> lock(clients_mutex);
                for (const auto& [fd, client] : clients) {
                    if (!client->is_drone && client->authenticated) {
                        sendWSFrame(client->ssl, WS_TEXT, data);
                    }
                }
            } else {
                // Message from control client - route to specific drone
                if (msg.contains("drone_id")) {
                    std::string target_drone = msg["drone_id"];
                    std::lock_guard<std::mutex> lock(clients_mutex);
                    
                    auto it = drone_map.find(target_drone);
                    if (it != drone_map.end()) {
                        auto drone_it = clients.find(it->second);
                        if (drone_it != clients.end()) {
                            sendWSFrame(drone_it->second->ssl, WS_TEXT, data);
                        }
                    }
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Error routing message: " << e.what() << std::endl;
        }
    }
    
    // Cleanup disconnected clients
    void cleanupLoop() {
        while (running) {
            std::this_thread::sleep_for(std::chrono::seconds(30));
            
            auto now = std::chrono::steady_clock::now();
            std::lock_guard<std::mutex> lock(clients_mutex);
            
            for (auto it = clients.begin(); it != clients.end();) {
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - it->second->last_seen).count();
                if (elapsed > 60) {
                    std::cout << "Removing inactive client" << std::endl;
                    if (it->second->is_drone) {
                        drone_map.erase(it->second->drone_id);
                    }
                    it = clients.erase(it);
                } else {
                    ++it;
                }
            }
        }
    }
    
    // WebSocket frame operations (shared with client)
    bool receiveWSFrame(SSL* ssl, WSFrame& frame) {
        uint8_t header[2];
        if (SSL_read(ssl, header, 2) != 2) {
            return false;
        }
        
        frame.fin = (header[0] & 0x80) != 0;
        frame.opcode = static_cast<WSOpcode>(header[0] & 0x0F);
        frame.masked = (header[1] & 0x80) != 0;
        
        uint64_t payload_len = header[1] & 0x7F;
        if (payload_len == 126) {
            uint8_t len_bytes[2];
            if (SSL_read(ssl, len_bytes, 2) != 2) return false;
            payload_len = (len_bytes[0] << 8) | len_bytes[1];
        } else if (payload_len == 127) {
            uint8_t len_bytes[8];
            if (SSL_read(ssl, len_bytes, 8) != 8) return false;
            payload_len = 0;
            for (int i = 0; i < 8; i++) {
                payload_len = (payload_len << 8) | len_bytes[i];
            }
        }
        
        frame.payload_length = payload_len;
        
        uint8_t mask[4] = {0};
        if (frame.masked) {
            if (SSL_read(ssl, mask, 4) != 4) return false;
        }
        
        frame.payload.resize(payload_len);
        if (payload_len > 0) {
            if (SSL_read(ssl, frame.payload.data(), payload_len) != static_cast<int>(payload_len)) {
                return false;
            }
            
            if (frame.masked) {
                for (size_t i = 0; i < payload_len; i++) {
                    frame.payload[i] ^= mask[i % 4];
                }
            }
        }
        
        return true;
    }
    
    bool sendWSFrame(SSL* ssl, WSOpcode opcode, const std::string& data) {
        std::vector<uint8_t> frame;
        
        // FIN = 1, RSV = 0, Opcode
        frame.push_back(0x80 | opcode);
        
        // Mask = 0 (server doesn't mask), Payload length
        size_t payload_len = data.length();
        if (payload_len < 126) {
            frame.push_back(payload_len);
        } else if (payload_len < 65536) {
            frame.push_back(126);
            frame.push_back((payload_len >> 8) & 0xFF);
            frame.push_back(payload_len & 0xFF);
        } else {
            frame.push_back(127);
            for (int i = 7; i >= 0; i--) {
                frame.push_back((payload_len >> (i * 8)) & 0xFF);
            }
        }
        
        // Payload (unmasked for server)
        frame.insert(frame.end(), data.begin(), data.end());
        
        int sent = SSL_write(ssl, frame.data(), frame.size());
        return sent == static_cast<int>(frame.size());
    }
    
public:
    RemoteControlRelay(int port, const std::string& token) : port(port), relay_token(token) {
        if (!initSSLServer()) {
            throw std::runtime_error("Failed to initialize SSL server");
        }
    }
    
    ~RemoteControlRelay() {
        stop();
        
        if (ssl_ctx) {
            SSL_CTX_free(ssl_ctx);
        }
    }
    
    // Start relay server
    bool start() {
        // Create socket
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0) {
            std::cerr << "Failed to create socket" << std::endl;
            return false;
        }
        
        // Allow reuse
        int opt = 1;
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        // Bind
        struct sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);
        
        if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
            std::cerr << "Failed to bind to port " << port << std::endl;
            close(server_fd);
            return false;
        }
        
        // Listen
        if (listen(server_fd, 10) < 0) {
            std::cerr << "Failed to listen" << std::endl;
            close(server_fd);
            return false;
        }
        
        running = true;
        accept_thread = std::thread(&RemoteControlRelay::acceptLoop, this);
        cleanup_thread = std::thread(&RemoteControlRelay::cleanupLoop, this);
        
        std::cout << "Remote control relay started on port " << port << std::endl;
        return true;
    }
    
    // Stop relay server
    void stop() {
        running = false;
        
        if (server_fd >= 0) {
            close(server_fd);
        }
        
        if (accept_thread.joinable()) accept_thread.join();
        if (cleanup_thread.joinable()) cleanup_thread.join();
    }
    
    // Get connected drones
    std::vector<std::string> getConnectedDrones() {
        std::lock_guard<std::mutex> lock(clients_mutex);
        std::vector<std::string> drones;
        for (const auto& [id, fd] : drone_map) {
            drones.push_back(id);
        }
        return drones;
    }
};

#endif // REMOTE_CONTROL_HPP