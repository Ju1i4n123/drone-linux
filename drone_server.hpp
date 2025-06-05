#ifndef DRONE_SERVER_HPP
#define DRONE_SERVER_HPP

#include <thread>
#include <atomic>
#include <string>
#include <functional>
#include <unordered_map>
#include <mutex>
#include <queue>
#include <chrono>
#include <sstream>
#include <nlohmann/json.hpp>

// Simple HTTP server implementation
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

using json = nlohmann::json;

class DroneServer {
private:
    int server_fd = -1;
    int port;
    std::thread server_thread;
    std::atomic<bool> running{false};
    std::string auth_token;
    
    // Command callback function
    std::function<void(const json&)> command_handler;
    
    // Status callback function
    std::function<json()> status_provider;
    
    // Simple HTTP response builder
    std::string buildHttpResponse(int code, const std::string& status, 
                                  const std::string& content, 
                                  const std::string& content_type = "application/json") {
        std::stringstream response;
        response << "HTTP/1.1 " << code << " " << status << "\r\n";
        response << "Content-Type: " << content_type << "\r\n";
        response << "Content-Length: " << content.length() << "\r\n";
        response << "Access-Control-Allow-Origin: *\r\n";
        response << "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n";
        response << "Access-Control-Allow-Headers: Content-Type, Authorization\r\n";
        response << "\r\n";
        response << content;
        return response.str();
    }
    
    // Parse HTTP request
    struct HttpRequest {
        std::string method;
        std::string path;
        std::string body;
        std::unordered_map<std::string, std::string> headers;
    };
    
    HttpRequest parseRequest(const std::string& request) {
        HttpRequest req;
        std::istringstream stream(request);
        std::string line;
        
        // Parse request line
        if (std::getline(stream, line)) {
            std::istringstream line_stream(line);
            line_stream >> req.method >> req.path;
        }
        
        // Parse headers
        while (std::getline(stream, line) && line != "\r") {
            size_t colon = line.find(':');
            if (colon != std::string::npos) {
                std::string key = line.substr(0, colon);
                std::string value = line.substr(colon + 2);
                value.pop_back(); // Remove \r
                req.headers[key] = value;
            }
        }
        
        // Get body
        std::string body;
        while (std::getline(stream, line)) {
            body += line + "\n";
        }
        if (!body.empty()) {
            body.pop_back(); // Remove last \n
        }
        req.body = body;
        
        return req;
    }
    
    // Handle client connection
    void handleClient(int client_socket) {
        char buffer[4096] = {0};
        int bytes_read = read(client_socket, buffer, 4096);
        
        if (bytes_read <= 0) {
            close(client_socket);
            return;
        }
        
        HttpRequest request = parseRequest(std::string(buffer));
        std::string response;
        
        // Check authentication
        if (request.headers.find("Authorization") == request.headers.end() ||
            request.headers["Authorization"] != "Bearer " + auth_token) {
            response = buildHttpResponse(401, "Unauthorized", 
                                       "{\"error\": \"Invalid authentication token\"}");
            send(client_socket, response.c_str(), response.length(), 0);
            close(client_socket);
            return;
        }
        
        // Handle requests
        if (request.method == "GET" && request.path == "/status") {
            // Get drone status
            json status = status_provider();
            response = buildHttpResponse(200, "OK", status.dump());
            
        } else if (request.method == "POST" && request.path == "/command") {
            // Execute command
            try {
                json command = json::parse(request.body);
                command_handler(command);
                response = buildHttpResponse(200, "OK", "{\"success\": true}");
            } catch (const std::exception& e) {
                json error;
                error["error"] = e.what();
                response = buildHttpResponse(400, "Bad Request", error.dump());
            }
            
        } else if (request.method == "GET" && request.path == "/") {
            // Serve simple web interface
            std::string html = getWebInterface();
            response = buildHttpResponse(200, "OK", html, "text/html");
            
        } else if (request.method == "OPTIONS") {
            // Handle CORS preflight
            response = buildHttpResponse(200, "OK", "");
            
        } else {
            response = buildHttpResponse(404, "Not Found", 
                                       "{\"error\": \"Endpoint not found\"}");
        }
        
        send(client_socket, response.c_str(), response.length(), 0);
        close(client_socket);
    }
    
    // Server loop
    void serverLoop() {
        while (running) {
            sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            
            int client_socket = accept(server_fd, (struct sockaddr*)&client_addr, &client_len);
            if (client_socket >= 0) {
                std::thread client_thread(&DroneServer::handleClient, this, client_socket);
                client_thread.detach();
            }
        }
    }
    
    // Simple web interface
    std::string getWebInterface() {
        return R"(
<!DOCTYPE html>
<html>
<head>
    <title>Drone Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; margin: 20px; background: #f0f0f0; }
        .container { max-width: 600px; margin: auto; background: white; padding: 20px; border-radius: 10px; }
        button { margin: 5px; padding: 10px 20px; font-size: 16px; }
        .status { background: #e0e0e0; padding: 10px; margin: 10px 0; border-radius: 5px; }
        .armed { background: #ffcccc; }
        .flying { background: #ccffcc; }
        input { margin: 5px; padding: 5px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Drone Remote Control</h1>
        <div id="status" class="status">Connecting...</div>
        
        <h3>Basic Controls</h3>
        <button onclick="sendCommand('arm')">ARM</button>
        <button onclick="sendCommand('disarm')">DISARM</button>
        <button onclick="sendCommand('hover')">HOVER</button>
        <button onclick="sendCommand('land')">LAND</button>
        
        <h3>Takeoff</h3>
        <input type="number" id="altitude" placeholder="Altitude (m)" value="2">
        <button onclick="takeoff()">TAKEOFF</button>
        
        <h3>Movement</h3>
        <div>
            <button onclick="move(1, 0, 0)">Forward</button>
            <button onclick="move(-1, 0, 0)">Backward</button>
            <button onclick="move(0, 1, 0)">Right</button>
            <button onclick="move(0, -1, 0)">Left</button>
            <button onclick="move(0, 0, 1)">Up</button>
            <button onclick="move(0, 0, -1)">Down</button>
        </div>
        
        <h3>Rotation</h3>
        <button onclick="rotate(-30)">← Rotate Left</button>
        <button onclick="rotate(30)">Rotate Right →</button>
    </div>
    
    <script>
        const TOKEN = prompt("Enter authentication token:");
        
        async function sendCommand(cmd, params = {}) {
            try {
                const response = await fetch('/command', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                        'Authorization': 'Bearer ' + TOKEN
                    },
                    body: JSON.stringify({ command: cmd, ...params })
                });
                if (!response.ok) throw new Error('Command failed');
            } catch (e) {
                alert('Error: ' + e.message);
            }
        }
        
        function takeoff() {
            const alt = document.getElementById('altitude').value;
            sendCommand('takeoff', { altitude: parseFloat(alt) });
        }
        
        function move(fwd, right, up) {
            sendCommand('move', { forward: fwd, right: right, up: up });
        }
        
        function rotate(deg) {
            sendCommand('rotate', { degrees: deg });
        }
        
        async function updateStatus() {
            try {
                const response = await fetch('/status', {
                    headers: { 'Authorization': 'Bearer ' + TOKEN }
                });
                const status = await response.json();
                
                const statusDiv = document.getElementById('status');
                statusDiv.innerHTML = `
                    Armed: ${status.armed ? 'YES' : 'NO'}<br>
                    Flying: ${status.in_flight ? 'YES' : 'NO'}<br>
                    Altitude: ${status.altitude.toFixed(2)}m<br>
                    Battery: ${status.battery_voltage.toFixed(2)}V<br>
                    GPS: ${status.gps_fix ? 'Fixed' : 'No Fix'}
                `;
                
                statusDiv.className = 'status';
                if (status.armed) statusDiv.className += ' armed';
                if (status.in_flight) statusDiv.className += ' flying';
                
            } catch (e) {
                document.getElementById('status').innerHTML = 'Connection error';
            }
        }
        
        setInterval(updateStatus, 1000);
    </script>
</body>
</html>
        )";
    }
    
public:
    DroneServer(int port = 8080) : port(port) {
        // Generate random auth token
        auth_token = generateAuthToken();
        std::cout << "\n*** DRONE SERVER AUTH TOKEN: " << auth_token << " ***\n" << std::endl;
    }
    
    ~DroneServer() {
        stop();
    }
    
    // Generate random authentication token
    std::string generateAuthToken() {
        const char charset[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
        std::string token;
        for (int i = 0; i < 32; i++) {
            token += charset[rand() % (sizeof(charset) - 1)];
        }
        return token;
    }
    
    // Set command handler
    void setCommandHandler(std::function<void(const json&)> handler) {
        command_handler = handler;
    }
    
    // Set status provider
    void setStatusProvider(std::function<json()> provider) {
        status_provider = provider;
    }
    
    // Start server
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
        sockaddr_in address;
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
        server_thread = std::thread(&DroneServer::serverLoop, this);
        
        std::cout << "Drone server started on port " << port << std::endl;
        return true;
    }
    
    // Stop server
    void stop() {
        running = false;
        if (server_fd >= 0) {
            close(server_fd);
        }
        if (server_thread.joinable()) {
            server_thread.join();
        }
    }
    
    std::string getAuthToken() const { return auth_token; }
};

#endif // DRONE_SERVER_HPP