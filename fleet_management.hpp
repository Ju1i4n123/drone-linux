#ifndef FLEET_MANAGEMENT_HPP
#define FLEET_MANAGEMENT_HPP

#include <thread>
#include <atomic>
#include <string>
#include <functional>
#include <unordered_map>
#include <map>
#include <mutex>
#include <queue>
#include <chrono>
#include <memory>
#include <condition_variable>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <random>
#include <curl/curl.h>
#include "nlohmann/json.hpp"
#include "remote_control.hpp"

using json = nlohmann::json;

// Mission types
enum MissionType {
    MISSION_DELIVERY,
    MISSION_SURVEILLANCE,
    MISSION_EMERGENCY,
    MISSION_RETURN_HOME,
    MISSION_INSPECTION,
    MISSION_CUSTOM
};

// Mission status
enum MissionStatus {
    MISSION_PENDING,
    MISSION_ASSIGNED,
    MISSION_IN_PROGRESS,
    MISSION_COMPLETED,
    MISSION_FAILED,
    MISSION_CANCELLED
};

// Drone status for fleet
enum DroneFleetStatus {
    DRONE_OFFLINE,
    DRONE_IDLE,
    DRONE_BUSY,
    DRONE_RETURNING,
    DRONE_CHARGING,
    DRONE_MAINTENANCE,
    DRONE_EMERGENCY
};

// Location structure
struct Location {
    double latitude;
    double longitude;
    float altitude;
    std::string address;
    std::string name;
    
    // Calculate distance to another location (in meters)
    double distanceTo(const Location& other) const {
        const double R = 6371000; // Earth radius in meters
        double lat1_rad = latitude * M_PI / 180.0;
        double lat2_rad = other.latitude * M_PI / 180.0;
        double delta_lat = (other.latitude - latitude) * M_PI / 180.0;
        double delta_lon = (other.longitude - longitude) * M_PI / 180.0;
        
        double a = sin(delta_lat/2) * sin(delta_lat/2) +
                   cos(lat1_rad) * cos(lat2_rad) *
                   sin(delta_lon/2) * sin(delta_lon/2);
        double c = 2 * atan2(sqrt(a), sqrt(1-a));
        
        return R * c;
    }
};

// Mission structure
struct Mission {
    std::string mission_id;
    MissionType type;
    MissionStatus status;
    std::string drone_id;
    std::string user_id;
    Location pickup_location;
    Location destination;
    std::vector<Location> waypoints;
    std::chrono::system_clock::time_point created_time;
    std::chrono::system_clock::time_point assigned_time;
    std::chrono::system_clock::time_point completed_time;
    float priority;
    json metadata;  // Custom data (package info, etc.)
    std::string notes;
};

// Enhanced drone info for fleet
struct DroneFleetInfo {
    std::string drone_id;
    DroneFleetStatus status;
    Location current_location;
    Location home_location;
    float battery_percentage;
    float max_range;  // km
    float cruise_speed;  // m/s
    float payload_capacity;  // kg
    float current_payload;  // kg
    bool has_camera;
    bool has_lidar;
    std::string current_mission_id;
    std::chrono::system_clock::time_point last_update;
    int total_flights;
    float total_distance;  // km
    json capabilities;  // Special capabilities
    std::vector<std::string> maintenance_log;
};

// Geofence zone
struct GeofenceZone {
    std::string zone_id;
    std::string name;
    std::vector<Location> boundaries;  // Polygon points
    float max_altitude;
    float min_altitude;
    bool is_exclusion_zone;  // true = no-fly zone
    
    // Check if point is inside polygon
    bool containsPoint(double lat, double lon) const {
        int n = boundaries.size();
        bool inside = false;
        
        double p1_lat = boundaries[0].latitude;
        double p1_lon = boundaries[0].longitude;
        
        for (int i = 1; i <= n; i++) {
            double p2_lat = boundaries[i % n].latitude;
            double p2_lon = boundaries[i % n].longitude;
            
            if (lon > std::min(p1_lon, p2_lon)) {
                if (lon <= std::max(p1_lon, p2_lon)) {
                    if (lat <= std::max(p1_lat, p2_lat)) {
                        if (p1_lon != p2_lon) {
                            double lat_intersection = (lon - p1_lon) * (p2_lat - p1_lat) / (p2_lon - p1_lon) + p1_lat;
                            if (p1_lat == p2_lat || lat <= lat_intersection) {
                                inside = !inside;
                            }
                        }
                    }
                }
            }
            p1_lat = p2_lat;
            p1_lon = p2_lon;
        }
        
        return inside;
    }
};

// REST API request/response
struct APIRequest {
    std::string method;
    std::string path;
    json body;
    std::unordered_map<std::string, std::string> headers;
    std::string client_ip;
    std::string auth_token;
};

struct APIResponse {
    int status_code;
    json body;
    std::unordered_map<std::string, std::string> headers;
};

// Write callback for CURL
size_t curl_write_callback(void* contents, size_t size, size_t nmemb, std::string* userp) {
    userp->append((char*)contents, size * nmemb);
    return size * nmemb;
}

// Geocoding service
class GeocodingService {
private:
    std::string api_key;
    CURL* curl;
    
public:
    GeocodingService(const std::string& key = "") : api_key(key) {
        curl_global_init(CURL_GLOBAL_DEFAULT);
        curl = curl_easy_init();
    }
    
    ~GeocodingService() {
        if (curl) curl_easy_cleanup(curl);
        curl_global_cleanup();
    }
    
    // Convert address to coordinates
    Location geocodeAddress(const std::string& address) {
        Location loc;
        loc.address = address;
        
        if (!curl) return loc;
        
        // Using OpenStreetMap Nominatim (free, no API key required)
        char* escaped = curl_easy_escape(curl, address.c_str(), address.length());
        std::string url = "https://nominatim.openstreetmap.org/search?format=json&q=";
        if (escaped) {
            url += escaped;
            curl_free(escaped);
        }
        
        std::string response;
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write_callback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
        curl_easy_setopt(curl, CURLOPT_USERAGENT, "DroneFleetManager/1.0");
        
        CURLcode res = curl_easy_perform(curl);
        if (res == CURLE_OK) {
            try {
                json data = json::parse(response);
                if (!data.empty() && data.is_array()) {
                    loc.latitude = std::stod(data[0]["lat"].get<std::string>());
                    loc.longitude = std::stod(data[0]["lon"].get<std::string>());
                    loc.name = data[0].value("display_name", address);
                }
            } catch (...) {
                std::cerr << "Geocoding failed for: " << address << std::endl;
            }
        }
        
        return loc;
    }
    
    // Reverse geocoding (coordinates to address)
    std::string reverseGeocode(double lat, double lon) {
        if (!curl) return "";
        
        std::string url = "https://nominatim.openstreetmap.org/reverse?format=json&lat=" +
                         std::to_string(lat) + "&lon=" + std::to_string(lon);
        
        std::string response;
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write_callback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
        curl_easy_setopt(curl, CURLOPT_USERAGENT, "DroneFleetManager/1.0");
        
        CURLcode res = curl_easy_perform(curl);
        if (res == CURLE_OK) {
            try {
                json data = json::parse(response);
                return data.value("display_name", "");
            } catch (...) {}
        }
        
        return "";
    }
};

// Fleet Management Server
class FleetManagementServer : public RemoteControlRelay {
private:
    // Fleet data
    std::mutex fleet_mutex;
    std::unordered_map<std::string, DroneFleetInfo> drones;
    std::unordered_map<std::string, Mission> missions;
    std::queue<std::string> mission_queue;
    std::vector<GeofenceZone> geofences;
    
    // Services
    std::unique_ptr<GeocodingService> geocoding;
    
    // Threads
    std::thread dispatch_thread;
    std::thread api_thread;
    std::thread monitoring_thread;
    
    // REST API server
    int api_port;
    int api_fd = -1;
    std::atomic<bool> api_running{false};
    
    // Configuration
    float dispatch_interval = 5.0f;  // seconds
    float drone_timeout = 30.0f;     // seconds
    float min_battery_for_mission = 20.0f;  // percentage
    
    // Statistics
    std::atomic<int> total_missions{0};
    std::atomic<int> completed_missions{0};
    std::atomic<int> failed_missions{0};
    
    // API handlers
    std::unordered_map<std::string, std::function<APIResponse(const APIRequest&)>> api_handlers;
    
    // Initialize API handlers
    void initializeAPIHandlers() {
        // Create mission
        api_handlers["POST /api/v1/missions"] = [this](const APIRequest& req) {
            return handleCreateMission(req);
        };
        
        // Get mission status
        api_handlers["GET /api/v1/missions/:id"] = [this](const APIRequest& req) {
            return handleGetMission(req);
        };
        
        // List all missions
        api_handlers["GET /api/v1/missions"] = [this](const APIRequest& req) {
            return handleListMissions(req);
        };
        
        // Cancel mission
        api_handlers["DELETE /api/v1/missions/:id"] = [this](const APIRequest& req) {
            return handleCancelMission(req);
        };
        
        // Get fleet status
        api_handlers["GET /api/v1/fleet"] = [this](const APIRequest& req) {
            return handleGetFleet(req);
        };
        
        // Get specific drone
        api_handlers["GET /api/v1/drones/:id"] = [this](const APIRequest& req) {
            return handleGetDrone(req);
        };
        
        // Command specific drone
        api_handlers["POST /api/v1/drones/:id/command"] = [this](const APIRequest& req) {
            return handleDroneCommand(req);
        };
        
        // Get/Set geofences
        api_handlers["GET /api/v1/geofences"] = [this](const APIRequest& req) {
            return handleGetGeofences(req);
        };
        
        // Analytics
        api_handlers["GET /api/v1/analytics"] = [this](const APIRequest& req) {
            return handleGetAnalytics(req);
        };
        
        // Find nearest available drone
        api_handlers["POST /api/v1/drones/nearest"] = [this](const APIRequest& req) {
            return handleFindNearestDrone(req);
        };
    }
    
    // Create mission handler
    APIResponse handleCreateMission(const APIRequest& req) {
        APIResponse resp;
        
        try {
            // Validate request
            if (!req.body.contains("destination")) {
                resp.status_code = 400;
                resp.body["error"] = "Missing destination";
                return resp;
            }
            
            // Create mission
            Mission mission;
            mission.mission_id = generateMissionId();
            mission.type = static_cast<MissionType>(req.body.value("type", 0));
            mission.status = MISSION_PENDING;
            mission.user_id = req.auth_token;  // Simple auth
            mission.created_time = std::chrono::system_clock::now();
            mission.priority = req.body.value("priority", 1.0f);
            mission.notes = req.body.value("notes", "");
            mission.metadata = req.body.value("metadata", json::object());
            
            // Parse destination
            auto dest = req.body["destination"];
            if (dest.is_string()) {
                // Address provided - geocode it
                mission.destination = geocoding->geocodeAddress(dest);
                if (mission.destination.latitude == 0 && mission.destination.longitude == 0) {
                    resp.status_code = 400;
                    resp.body["error"] = "Failed to geocode address";
                    return resp;
                }
            } else if (dest.is_object()) {
                // Coordinates provided
                mission.destination.latitude = dest["latitude"];
                mission.destination.longitude = dest["longitude"];
                mission.destination.altitude = dest.value("altitude", 50.0f);
                mission.destination.name = dest.value("name", "Destination");
            }
            
            // Parse pickup location if provided
            if (req.body.contains("pickup")) {
                auto pickup = req.body["pickup"];
                if (pickup.is_string()) {
                    mission.pickup_location = geocoding->geocodeAddress(pickup);
                } else if (pickup.is_object()) {
                    mission.pickup_location.latitude = pickup["latitude"];
                    mission.pickup_location.longitude = pickup["longitude"];
                    mission.pickup_location.altitude = pickup.value("altitude", 50.0f);
                }
            } else if (req.body.contains("user_location")) {
                // Use user's current location as pickup
                auto user_loc = req.body["user_location"];
                mission.pickup_location.latitude = user_loc["latitude"];
                mission.pickup_location.longitude = user_loc["longitude"];
                mission.pickup_location.altitude = 0;
            }
            
            // Add waypoints if provided
            if (req.body.contains("waypoints")) {
                for (const auto& wp : req.body["waypoints"]) {
                    Location waypoint;
                    waypoint.latitude = wp["latitude"];
                    waypoint.longitude = wp["longitude"];
                    waypoint.altitude = wp.value("altitude", 50.0f);
                    mission.waypoints.push_back(waypoint);
                }
            }
            
            // Check geofences
            if (!isPathSafe(mission)) {
                resp.status_code = 400;
                resp.body["error"] = "Mission path crosses no-fly zone";
                return resp;
            }
            
            // Add to queue
            {
                std::lock_guard<std::mutex> lock(fleet_mutex);
                missions[mission.mission_id] = mission;
                mission_queue.push(mission.mission_id);
                total_missions++;
            }
            
            // Response
            resp.status_code = 201;
            resp.body["mission_id"] = mission.mission_id;
            resp.body["status"] = "pending";
            resp.body["estimated_wait_time"] = estimateWaitTime();
            resp.body["destination"] = {
                {"latitude", mission.destination.latitude},
                {"longitude", mission.destination.longitude},
                {"address", mission.destination.address}
            };
            
        } catch (const std::exception& e) {
            resp.status_code = 500;
            resp.body["error"] = e.what();
        }
        
        return resp;
    }
    
    // Find nearest drone handler
    APIResponse handleFindNearestDrone(const APIRequest& req) {
        APIResponse resp;
        
        try {
            Location user_location;
            
            // Parse location
            if (req.body.contains("latitude") && req.body.contains("longitude")) {
                user_location.latitude = req.body["latitude"];
                user_location.longitude = req.body["longitude"];
            } else if (req.body.contains("address")) {
                user_location = geocoding->geocodeAddress(req.body["address"]);
                if (user_location.latitude == 0 && user_location.longitude == 0) {
                    resp.status_code = 400;
                    resp.body["error"] = "Failed to geocode address";
                    return resp;
                }
            } else {
                resp.status_code = 400;
                resp.body["error"] = "Location required";
                return resp;
            }
            
            // Find nearest available drone
            std::string nearest_drone_id;
            double min_distance = std::numeric_limits<double>::max();
            double min_eta = std::numeric_limits<double>::max();
            
            {
                std::lock_guard<std::mutex> lock(fleet_mutex);
                for (const auto& [id, drone] : drones) {
                    if (drone.status == DRONE_IDLE && drone.battery_percentage > min_battery_for_mission) {
                        double distance = drone.current_location.distanceTo(user_location);
                        double eta = distance / drone.cruise_speed;  // seconds
                        
                        if (distance < min_distance) {
                            min_distance = distance;
                            min_eta = eta;
                            nearest_drone_id = id;
                        }
                    }
                }
            }
            
            if (nearest_drone_id.empty()) {
                resp.status_code = 404;
                resp.body["error"] = "No available drones";
                resp.body["total_drones"] = drones.size();
                resp.body["busy_drones"] = countBusyDrones();
            } else {
                resp.status_code = 200;
                resp.body["drone_id"] = nearest_drone_id;
                resp.body["distance_meters"] = min_distance;
                resp.body["eta_seconds"] = min_eta;
                resp.body["eta_minutes"] = min_eta / 60.0;
                
                auto& drone = drones[nearest_drone_id];
                resp.body["drone_info"] = {
                    {"battery_percentage", drone.battery_percentage},
                    {"current_location", {
                        {"latitude", drone.current_location.latitude},
                        {"longitude", drone.current_location.longitude}
                    }},
                    {"capabilities", drone.capabilities}
                };
            }
            
        } catch (const std::exception& e) {
            resp.status_code = 500;
            resp.body["error"] = e.what();
        }
        
        return resp;
    }
    
    // Get fleet status
    APIResponse handleGetFleet(const APIRequest& req) {
        APIResponse resp;
        resp.status_code = 200;
        
        json fleet_data = json::array();
        
        std::lock_guard<std::mutex> lock(fleet_mutex);
        for (const auto& [id, drone] : drones) {
            json drone_data;
            drone_data["drone_id"] = drone.drone_id;
            drone_data["status"] = droneStatusToString(drone.status);
            drone_data["location"] = {
                {"latitude", drone.current_location.latitude},
                {"longitude", drone.current_location.longitude},
                {"altitude", drone.current_location.altitude}
            };
            drone_data["battery_percentage"] = drone.battery_percentage;
            drone_data["current_mission"] = drone.current_mission_id;
            drone_data["total_flights"] = drone.total_flights;
            drone_data["capabilities"] = drone.capabilities;
            
            fleet_data.push_back(drone_data);
        }
        
        resp.body["drones"] = fleet_data;
        resp.body["total_drones"] = drones.size();
        resp.body["available_drones"] = countAvailableDrones();
        resp.body["active_missions"] = countActiveMissions();
        
        return resp;
    }
    
    // Dispatch thread - assigns missions to drones
    void dispatchLoop() {
        while (api_running) {
            std::this_thread::sleep_for(std::chrono::seconds(static_cast<int>(dispatch_interval)));
            
            std::lock_guard<std::mutex> lock(fleet_mutex);
            
            // Process mission queue
            std::queue<std::string> temp_queue;
            
            while (!mission_queue.empty()) {
                std::string mission_id = mission_queue.front();
                mission_queue.pop();
                
                auto& mission = missions[mission_id];
                
                if (mission.status == MISSION_PENDING) {
                    // Find best drone for mission
                    std::string best_drone_id = findBestDroneForMission(mission);
                    
                    if (!best_drone_id.empty()) {
                        // Assign mission to drone
                        assignMissionToDrone(mission_id, best_drone_id);
                    } else {
                        // No drone available - keep in queue
                        temp_queue.push(mission_id);
                    }
                }
            }
            
            // Restore unassigned missions to queue
            while (!temp_queue.empty()) {
                mission_queue.push(temp_queue.front());
                temp_queue.pop();
            }
            
            // Update drone statuses
            updateDroneStatuses();
        }
    }
    
    // Find best drone for mission
    std::string findBestDroneForMission(const Mission& mission) {
        std::string best_drone_id;
        double best_score = std::numeric_limits<double>::max();
        
        Location start_location = mission.pickup_location.latitude != 0 ? 
                                 mission.pickup_location : mission.destination;
        
        for (auto& [id, drone] : drones) {
            if (drone.status != DRONE_IDLE) continue;
            if (drone.battery_percentage < min_battery_for_mission) continue;
            
            // Calculate mission feasibility
            double total_distance = 0;
            
            // Distance to pickup/start
            total_distance += drone.current_location.distanceTo(start_location);
            
            // Distance for mission
            if (mission.pickup_location.latitude != 0) {
                total_distance += mission.pickup_location.distanceTo(mission.destination);
            }
            
            // Add waypoint distances
            Location last_loc = mission.pickup_location.latitude != 0 ? 
                               mission.pickup_location : drone.current_location;
            for (const auto& wp : mission.waypoints) {
                total_distance += last_loc.distanceTo(wp);
                last_loc = wp;
            }
            total_distance += last_loc.distanceTo(mission.destination);
            
            // Distance back home
            total_distance += mission.destination.distanceTo(drone.home_location);
            
            // Check if drone has enough range
            double required_range = total_distance / 1000.0;  // km
            double battery_range = (drone.battery_percentage / 100.0) * drone.max_range;
            
            if (battery_range < required_range * 1.2) continue;  // 20% safety margin
            
            // Check payload capacity
            float required_payload = mission.metadata.value("payload_weight", 0.0f);
            if (required_payload > drone.payload_capacity) continue;
            
            // Check capabilities
            if (mission.type == MISSION_SURVEILLANCE && !drone.has_camera) continue;
            
            // Calculate score (lower is better)
            double score = 0;
            score += drone.current_location.distanceTo(start_location);  // Distance weight
            score += (100 - drone.battery_percentage) * 10;  // Battery weight
            score += drone.total_flights * 0.1;  // Wear balancing
            score -= mission.priority * 1000;  // Priority weight
            
            if (score < best_score) {
                best_score = score;
                best_drone_id = id;
            }
        }
        
        return best_drone_id;
    }
    
    // Assign mission to drone
    void assignMissionToDrone(const std::string& mission_id, const std::string& drone_id) {
        auto& mission = missions[mission_id];
        auto& drone = drones[drone_id];
        
        mission.drone_id = drone_id;
        mission.status = MISSION_ASSIGNED;
        mission.assigned_time = std::chrono::system_clock::now();
        
        drone.status = DRONE_BUSY;
        drone.current_mission_id = mission_id;
        
        // Send mission to drone via WebSocket
        json command;
        command["type"] = MSG_WAYPOINT;
        command["drone_id"] = drone_id;
        command["data"]["mission_id"] = mission_id;
        command["data"]["type"] = mission.type;
        command["data"]["priority"] = mission.priority;
        
        // Build waypoint list
        json waypoints = json::array();
        
        // Add pickup if exists
        if (mission.pickup_location.latitude != 0) {
            waypoints.push_back({
                {"latitude", mission.pickup_location.latitude},
                {"longitude", mission.pickup_location.longitude},
                {"altitude", mission.pickup_location.altitude},
                {"hover", true},
                {"hover_time", 30}  // 30 seconds for pickup
            });
        }
        
        // Add intermediate waypoints
        for (const auto& wp : mission.waypoints) {
            waypoints.push_back({
                {"latitude", wp.latitude},
                {"longitude", wp.longitude},
                {"altitude", wp.altitude},
                {"hover", false}
            });
        }
        
        // Add destination
        waypoints.push_back({
            {"latitude", mission.destination.latitude},
            {"longitude", mission.destination.longitude},
            {"altitude", mission.destination.altitude},
            {"hover", true},
            {"hover_time", mission.metadata.value("destination_hover_time", 60)}
        });
        
        // Add return home
        waypoints.push_back({
            {"latitude", drone.home_location.latitude},
            {"longitude", drone.home_location.longitude},
            {"altitude", drone.home_location.altitude},
            {"hover", false}
        });
        
        command["data"]["waypoints"] = waypoints;
        command["data"]["metadata"] = mission.metadata;
        
        // Route to drone
        auto it = drone_map.find(drone_id);
        if (it != drone_map.end()) {
            auto client_it = clients.find(it->second);
            if (client_it != clients.end()) {
                sendWSFrame(client_it->second->ssl, WS_TEXT, command.dump());
                
                std::cout << "Mission " << mission_id << " assigned to drone " << drone_id << std::endl;
                
                // Update mission status
                mission.status = MISSION_IN_PROGRESS;
            }
        }
    }
    
    // Update drone statuses based on telemetry
    void updateDroneStatuses() {
        auto now = std::chrono::system_clock::now();
        
        for (auto& [id, drone] : drones) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                now - drone.last_update).count();
            
            // Mark offline if no update for timeout period
            if (elapsed > drone_timeout) {
                if (drone.status != DRONE_OFFLINE) {
                    std::cout << "Drone " << id << " went offline" << std::endl;
                    drone.status = DRONE_OFFLINE;
                    
                    // Cancel active mission
                    if (!drone.current_mission_id.empty()) {
                        auto& mission = missions[drone.current_mission_id];
                        mission.status = MISSION_FAILED;
                        failed_missions++;
                        
                        // Requeue mission
                        mission.status = MISSION_PENDING;
                        mission.drone_id.clear();
                        mission_queue.push(drone.current_mission_id);
                        
                        drone.current_mission_id.clear();
                    }
                }
            }
            
            // Auto-charging logic
            if (drone.status == DRONE_IDLE && drone.battery_percentage < 20.0f) {
                // Check if drone is at home
                double home_distance = drone.current_location.distanceTo(drone.home_location);
                if (home_distance < 10.0) {  // Within 10 meters of home
                    drone.status = DRONE_CHARGING;
                    std::cout << "Drone " << id << " started charging" << std::endl;
                } else {
                    // Send return home command
                    sendReturnHomeCommand(id);
                    drone.status = DRONE_RETURNING;
                }
            }
            
            // Charging complete
            if (drone.status == DRONE_CHARGING && drone.battery_percentage > 95.0f) {
                drone.status = DRONE_IDLE;
                std::cout << "Drone " << id << " finished charging" << std::endl;
            }
        }
    }
    
    // Check if mission path is safe (geofences)
    bool isPathSafe(const Mission& mission) {
        // Check all waypoints and destination
        std::vector<Location> path_points;
        
        if (mission.pickup_location.latitude != 0) {
            path_points.push_back(mission.pickup_location);
        }
        
        path_points.insert(path_points.end(), mission.waypoints.begin(), mission.waypoints.end());
        path_points.push_back(mission.destination);
        
        for (const auto& point : path_points) {
            for (const auto& zone : geofences) {
                if (zone.is_exclusion_zone && 
                    zone.containsPoint(point.latitude, point.longitude)) {
                    return false;
                }
            }
        }
        
        return true;
    }
    
    // Generate unique mission ID
    std::string generateMissionId() {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_int_distribution<> dis(0, 15);
        
        const char* hex = "0123456789ABCDEF";
        std::string id = "MSN-";
        
        for (int i = 0; i < 12; i++) {
            id += hex[dis(gen)];
        }
        
        return id;
    }
    
    // Send return home command
    void sendReturnHomeCommand(const std::string& drone_id) {
        json command;
        command["type"] = MSG_COMMAND;
        command["drone_id"] = drone_id;
        command["data"]["command"] = "return_home";
        
        auto it = drone_map.find(drone_id);
        if (it != drone_map.end()) {
            auto client_it = clients.find(it->second);
            if (client_it != clients.end()) {
                sendWSFrame(client_it->second->ssl, WS_TEXT, command.dump());
            }
        }
    }
    
    // Handle incoming telemetry
    void handleTelemetry(const std::string& drone_id, const json& telemetry) {
        std::lock_guard<std::mutex> lock(fleet_mutex);
        
        auto& drone = drones[drone_id];
        
        // Update drone info
        drone.current_location.latitude = telemetry["latitude"];
        drone.current_location.longitude = telemetry["longitude"];
        drone.current_location.altitude = telemetry["altitude"];
        drone.battery_percentage = (telemetry["battery_voltage"].get<float>() - 10.0f) / 2.6f * 100.0f;
        drone.last_update = std::chrono::system_clock::now();
        
        // Update status based on telemetry
        if (drone.status == DRONE_OFFLINE) {
            drone.status = DRONE_IDLE;
            std::cout << "Drone " << drone_id << " came online" << std::endl;
        }
        
        // Check mission progress
        if (!drone.current_mission_id.empty()) {
            auto& mission = missions[drone.current_mission_id];
            
            // Check if reached destination
            double dest_distance = drone.current_location.distanceTo(mission.destination);
            if (dest_distance < 10.0 && mission.status == MISSION_IN_PROGRESS) {
                // Mission completed
                mission.status = MISSION_COMPLETED;
                mission.completed_time = std::chrono::system_clock::now();
                completed_missions++;
                
                std::cout << "Mission " << mission.mission_id << " completed!" << std::endl;
                
                // Clear mission from drone
                drone.current_mission_id.clear();
                drone.status = DRONE_RETURNING;
            }
        }
        
        // Check if returned home
        if (drone.status == DRONE_RETURNING) {
            double home_distance = drone.current_location.distanceTo(drone.home_location);
            if (home_distance < 10.0) {
                drone.status = DRONE_IDLE;
                std::cout << "Drone " << drone_id << " returned home" << std::endl;
            }
        }
    }
    
    // REST API thread
    void apiLoop() {
        while (api_running) {
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            
            int client_fd = accept(api_fd, (struct sockaddr*)&client_addr, &client_len);
            if (client_fd >= 0) {
                std::thread(&FleetManagementServer::handleAPIClient, this, client_fd).detach();
            }
        }
    }
    
    // Handle API client
    void handleAPIClient(int client_fd) {
        char buffer[4096];
        int bytes = read(client_fd, buffer, sizeof(buffer) - 1);
        
        if (bytes <= 0) {
            close(client_fd);
            return;
        }
        
        buffer[bytes] = '\0';
        
        // Parse HTTP request
        APIRequest request;
        std::istringstream stream(buffer);
        std::string line;
        
        // Parse request line
        if (std::getline(stream, line)) {
            std::istringstream line_stream(line);
            line_stream >> request.method >> request.path;
        }
        
        // Parse headers
        while (std::getline(stream, line) && line != "\r") {
            size_t colon = line.find(':');
            if (colon != std::string::npos) {
                std::string key = line.substr(0, colon);
                std::string value = line.substr(colon + 2);
                if (!value.empty() && value.back() == '\r') {
                    value.pop_back();
                }
                request.headers[key] = value;
            }
        }
        
        // Get body
        std::string body;
        while (std::getline(stream, line)) {
            body += line + "\n";
        }
        
        // Parse JSON body
        if (!body.empty()) {
            try {
                request.body = json::parse(body);
            } catch (...) {
                request.body = json::object();
            }
        }
        
        // Extract auth token
        if (request.headers.find("Authorization") != request.headers.end()) {
            std::string auth = request.headers["Authorization"];
            if (auth.substr(0, 7) == "Bearer ") {
                request.auth_token = auth.substr(7);
            }
        }
        
        // Route request
        APIResponse response = routeAPIRequest(request);
        
        // Build HTTP response
        std::stringstream resp;
        resp << "HTTP/1.1 " << response.status_code << " ";
        
        switch (response.status_code) {
            case 200: resp << "OK"; break;
            case 201: resp << "Created"; break;
            case 400: resp << "Bad Request"; break;
            case 401: resp << "Unauthorized"; break;
            case 404: resp << "Not Found"; break;
            case 500: resp << "Internal Server Error"; break;
            default: resp << "Unknown"; break;
        }
        
        resp << "\r\n";
        resp << "Content-Type: application/json\r\n";
        resp << "Access-Control-Allow-Origin: *\r\n";
        resp << "Access-Control-Allow-Methods: GET, POST, DELETE, OPTIONS\r\n";
        resp << "Access-Control-Allow-Headers: Content-Type, Authorization\r\n";
        
        std::string body_str = response.body.dump();
        resp << "Content-Length: " << body_str.length() << "\r\n";
        resp << "\r\n";
        resp << body_str;
        
        send(client_fd, resp.str().c_str(), resp.str().length(), 0);
        close(client_fd);
    }
    
    // Route API request to handler
    APIResponse routeAPIRequest(const APIRequest& request) {
        APIResponse response;
        
        // Check authentication
        if (request.auth_token.empty()) {
            response.status_code = 401;
            response.body["error"] = "Authentication required";
            return response;
        }
        
        // Find matching handler
        std::string route = request.method + " " + request.path;
        
        // Try exact match first
        auto it = api_handlers.find(route);
        if (it != api_handlers.end()) {
            return it->second(request);
        }
        
        // Try pattern matching (for :id parameters)
        for (const auto& [pattern, handler] : api_handlers) {
            if (matchesPattern(route, pattern)) {
                return handler(request);
            }
        }
        
        // Not found
        response.status_code = 404;
        response.body["error"] = "Endpoint not found";
        return response;
    }
    
    // Pattern matching for routes with parameters
    bool matchesPattern(const std::string& route, const std::string& pattern) {
        std::vector<std::string> route_parts = split(route, '/');
        std::vector<std::string> pattern_parts = split(pattern, '/');
        
        if (route_parts.size() != pattern_parts.size()) {
            return false;
        }
        
        for (size_t i = 0; i < route_parts.size(); i++) {
            if (pattern_parts[i].empty()) continue;
            if (pattern_parts[i][0] == ':') continue;  // Parameter
            if (route_parts[i] != pattern_parts[i]) return false;
        }
        
        return true;
    }
    
    // String split helper
    std::vector<std::string> split(const std::string& str, char delimiter) {
        std::vector<std::string> tokens;
        std::stringstream ss(str);
        std::string token;
        
        while (std::getline(ss, token, delimiter)) {
            tokens.push_back(token);
        }
        
        return tokens;
    }
    
    // Helper functions
    int countAvailableDrones() {
        int count = 0;
        for (const auto& [id, drone] : drones) {
            if (drone.status == DRONE_IDLE && drone.battery_percentage > min_battery_for_mission) {
                count++;
            }
        }
        return count;
    }
    
    int countBusyDrones() {
        int count = 0;
        for (const auto& [id, drone] : drones) {
            if (drone.status == DRONE_BUSY) {
                count++;
            }
        }
        return count;
    }
    
    int countActiveMissions() {
        int count = 0;
        for (const auto& [id, mission] : missions) {
            if (mission.status == MISSION_IN_PROGRESS || mission.status == MISSION_ASSIGNED) {
                count++;
            }
        }
        return count;
    }
    
    int estimateWaitTime() {
        // Simple estimation based on queue size and available drones
        int queue_size = mission_queue.size();
        int available_drones = countAvailableDrones();
        
        if (available_drones > 0) {
            return (queue_size / available_drones) * 300;  // 5 minutes per mission estimate
        } else {
            return 1800;  // 30 minutes if no drones available
        }
    }
    
    std::string droneStatusToString(DroneFleetStatus status) {
        switch (status) {
            case DRONE_OFFLINE: return "offline";
            case DRONE_IDLE: return "idle";
            case DRONE_BUSY: return "busy";
            case DRONE_RETURNING: return "returning";
            case DRONE_CHARGING: return "charging";
            case DRONE_MAINTENANCE: return "maintenance";
            case DRONE_EMERGENCY: return "emergency";
            default: return "unknown";
        }
    }
    
    // Other API handlers (implement these similarly)
    APIResponse handleGetMission(const APIRequest& req) {
        APIResponse resp;
        // Extract mission ID from path
        std::string mission_id = extractIdFromPath(req.path);
        
        std::lock_guard<std::mutex> lock(fleet_mutex);
        auto it = missions.find(mission_id);
        if (it != missions.end()) {
            resp.status_code = 200;
            resp.body = missionToJson(it->second);
        } else {
            resp.status_code = 404;
            resp.body["error"] = "Mission not found";
        }
        
        return resp;
    }
    
    APIResponse handleListMissions(const APIRequest& req) {
        APIResponse resp;
        resp.status_code = 200;
        
        json missions_array = json::array();
        std::lock_guard<std::mutex> lock(fleet_mutex);
        
        for (const auto& [id, mission] : missions) {
            // Filter by user if not admin
            if (req.auth_token != "admin" && mission.user_id != req.auth_token) {
                continue;
            }
            missions_array.push_back(missionToJson(mission));
        }
        
        resp.body["missions"] = missions_array;
        resp.body["total"] = missions_array.size();
        
        return resp;
    }
    
    APIResponse handleCancelMission(const APIRequest& req) {
        APIResponse resp;
        std::string mission_id = extractIdFromPath(req.path);
        
        std::lock_guard<std::mutex> lock(fleet_mutex);
        auto it = missions.find(mission_id);
        
        if (it != missions.end()) {
            auto& mission = it->second;
            
            // Check authorization
            if (req.auth_token != "admin" && mission.user_id != req.auth_token) {
                resp.status_code = 403;
                resp.body["error"] = "Unauthorized";
                return resp;
            }
            
            if (mission.status == MISSION_PENDING || mission.status == MISSION_ASSIGNED) {
                mission.status = MISSION_CANCELLED;
                
                // If assigned to drone, free it
                if (!mission.drone_id.empty()) {
                    auto& drone = drones[mission.drone_id];
                    drone.current_mission_id.clear();
                    drone.status = DRONE_IDLE;
                    
                    // Send cancel command to drone
                    json command;
                    command["type"] = MSG_COMMAND;
                    command["drone_id"] = mission.drone_id;
                    command["data"]["command"] = "cancel_mission";
                    
                    // Route to drone
                    auto drone_it = drone_map.find(mission.drone_id);
                    if (drone_it != drone_map.end()) {
                        auto client_it = clients.find(drone_it->second);
                        if (client_it != clients.end()) {
                            sendWSFrame(client_it->second->ssl, WS_TEXT, command.dump());
                        }
                    }
                }
                
                resp.status_code = 200;
                resp.body["status"] = "cancelled";
            } else {
                resp.status_code = 400;
                resp.body["error"] = "Mission cannot be cancelled in current state";
            }
        } else {
            resp.status_code = 404;
            resp.body["error"] = "Mission not found";
        }
        
        return resp;
    }
    
    APIResponse handleGetDrone(const APIRequest& req) {
        APIResponse resp;
        std::string drone_id = extractIdFromPath(req.path);
        
        std::lock_guard<std::mutex> lock(fleet_mutex);
        auto it = drones.find(drone_id);
        
        if (it != drones.end()) {
            resp.status_code = 200;
            resp.body = droneToJson(it->second);
        } else {
            resp.status_code = 404;
            resp.body["error"] = "Drone not found";
        }
        
        return resp;
    }
    
    APIResponse handleDroneCommand(const APIRequest& req) {
        APIResponse resp;
        std::string drone_id = extractIdFromPath(req.path);
        
        // Admin only
        if (req.auth_token != "admin") {
            resp.status_code = 403;
            resp.body["error"] = "Admin access required";
            return resp;
        }
        
        // Send command to drone
        json command;
        command["type"] = MSG_COMMAND;
        command["drone_id"] = drone_id;
        command["data"] = req.body;
        
        // Route to drone
        auto it = drone_map.find(drone_id);
        if (it != drone_map.end()) {
            auto client_it = clients.find(it->second);
            if (client_it != clients.end()) {
                sendWSFrame(client_it->second->ssl, WS_TEXT, command.dump());
                resp.status_code = 200;
                resp.body["status"] = "command sent";
            } else {
                resp.status_code = 404;
                resp.body["error"] = "Drone not connected";
            }
        } else {
            resp.status_code = 404;
            resp.body["error"] = "Drone not found";
        }
        
        return resp;
    }
    
    APIResponse handleGetGeofences(const APIRequest& req) {
        APIResponse resp;
        resp.status_code = 200;
        
        json zones = json::array();
        for (const auto& zone : geofences) {
            json zone_data;
            zone_data["id"] = zone.zone_id;
            zone_data["name"] = zone.name;
            zone_data["is_exclusion"] = zone.is_exclusion_zone;
            zone_data["max_altitude"] = zone.max_altitude;
            zone_data["min_altitude"] = zone.min_altitude;
            
            json boundaries = json::array();
            for (const auto& point : zone.boundaries) {
                boundaries.push_back({
                    {"latitude", point.latitude},
                    {"longitude", point.longitude}
                });
            }
            zone_data["boundaries"] = boundaries;
            
            zones.push_back(zone_data);
        }
        
        resp.body["geofences"] = zones;
        return resp;
    }
    
    APIResponse handleGetAnalytics(const APIRequest& req) {
        APIResponse resp;
        resp.status_code = 200;
        
        resp.body["total_missions"] = total_missions.load();
        resp.body["completed_missions"] = completed_missions.load();
        resp.body["failed_missions"] = failed_missions.load();
        resp.body["success_rate"] = total_missions > 0 ? 
            (float)completed_missions / total_missions * 100 : 0;
        
        // Fleet utilization
        int total_drones = drones.size();
        int busy_drones = countBusyDrones();
        resp.body["fleet_utilization"] = total_drones > 0 ? 
            (float)busy_drones / total_drones * 100 : 0;
        
        // Average mission time
        float total_time = 0;
        int completed_count = 0;
        for (const auto& [id, mission] : missions) {
            if (mission.status == MISSION_COMPLETED) {
                auto duration = std::chrono::duration_cast<std::chrono::minutes>(
                    mission.completed_time - mission.assigned_time).count();
                total_time += duration;
                completed_count++;
            }
        }
        resp.body["average_mission_time_minutes"] = completed_count > 0 ? 
            total_time / completed_count : 0;
        
        return resp;
    }
    
    // Helper to extract ID from path like /api/v1/drones/:id
    std::string extractIdFromPath(const std::string& path) {
        auto parts = split(path, '/');
        if (!parts.empty()) {
            return parts.back();
        }
        return "";
    }
    
    // Convert mission to JSON
    json missionToJson(const Mission& mission) {
        json data;
        data["mission_id"] = mission.mission_id;
        data["type"] = mission.type;
        data["status"] = missionStatusToString(mission.status);
        data["drone_id"] = mission.drone_id;
        data["created_time"] = std::chrono::system_clock::to_time_t(mission.created_time);
        data["priority"] = mission.priority;
        data["notes"] = mission.notes;
        data["metadata"] = mission.metadata;
        
        data["destination"] = {
            {"latitude", mission.destination.latitude},
            {"longitude", mission.destination.longitude},
            {"address", mission.destination.address}
        };
        
        if (mission.pickup_location.latitude != 0) {
            data["pickup_location"] = {
                {"latitude", mission.pickup_location.latitude},
                {"longitude", mission.pickup_location.longitude},
                {"address", mission.pickup_location.address}
            };
        }
        
        return data;
    }
    
    // Convert drone to JSON
    json droneToJson(const DroneFleetInfo& drone) {
        json data;
        data["drone_id"] = drone.drone_id;
        data["status"] = droneStatusToString(drone.status);
        data["battery_percentage"] = drone.battery_percentage;
        data["max_range"] = drone.max_range;
        data["cruise_speed"] = drone.cruise_speed;
        data["payload_capacity"] = drone.payload_capacity;
        data["current_payload"] = drone.current_payload;
        data["total_flights"] = drone.total_flights;
        data["total_distance"] = drone.total_distance;
        data["current_mission"] = drone.current_mission_id;
        data["capabilities"] = drone.capabilities;
        
        data["location"] = {
            {"latitude", drone.current_location.latitude},
            {"longitude", drone.current_location.longitude},
            {"altitude", drone.current_location.altitude}
        };
        
        data["home_location"] = {
            {"latitude", drone.home_location.latitude},
            {"longitude", drone.home_location.longitude}
        };
        
        return data;
    }
    
    std::string missionStatusToString(MissionStatus status) {
        switch (status) {
            case MISSION_PENDING: return "pending";
            case MISSION_ASSIGNED: return "assigned";
            case MISSION_IN_PROGRESS: return "in_progress";
            case MISSION_COMPLETED: return "completed";
            case MISSION_FAILED: return "failed";
            case MISSION_CANCELLED: return "cancelled";
            default: return "unknown";
        }
    }
    
public:
    FleetManagementServer(int relay_port, int api_port, const std::string& token)
        : RemoteControlRelay(relay_port, token), api_port(api_port) {
        
        geocoding = std::make_unique<GeocodingService>();
        initializeAPIHandlers();
        
        // Add sample geofences (airports, restricted areas, etc.)
        GeofenceZone airport_zone;
        airport_zone.zone_id = "AIRPORT-001";
        airport_zone.name = "Local Airport";
        airport_zone.is_exclusion_zone = true;
        airport_zone.max_altitude = 500;
        // Add boundary points for your local airport
        geofences.push_back(airport_zone);
    }
    
    ~FleetManagementServer() {
        stopAPI();
    }
    
    // Override start to also start fleet management
    bool start() {
        if (!RemoteControlRelay::start()) {
            return false;
        }
        
        // Start REST API server
        api_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (api_fd < 0) {
            std::cerr << "Failed to create API socket" << std::endl;
            return false;
        }
        
        int opt = 1;
        setsockopt(api_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        struct sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(api_port);
        
        if (bind(api_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
            std::cerr << "Failed to bind API port " << api_port << std::endl;
            close(api_fd);
            return false;
        }
        
        if (listen(api_fd, 10) < 0) {
            std::cerr << "Failed to listen on API port" << std::endl;
            close(api_fd);
            return false;
        }
        
        api_running = true;
        
        // Start threads
        dispatch_thread = std::thread(&FleetManagementServer::dispatchLoop, this);
        api_thread = std::thread(&FleetManagementServer::apiLoop, this);
        monitoring_thread = std::thread(&FleetManagementServer::monitoringLoop, this);
        
        std::cout << "Fleet Management Server started" << std::endl;
        std::cout << "WebSocket relay on port " << port << std::endl;
        std::cout << "REST API on port " << api_port << std::endl;
        
        return true;
    }
    
    // Stop API server
    void stopAPI() {
        api_running = false;
        
        if (api_fd >= 0) {
            close(api_fd);
        }
        
        if (dispatch_thread.joinable()) dispatch_thread.join();
        if (api_thread.joinable()) api_thread.join();
        if (monitoring_thread.joinable()) monitoring_thread.join();
    }
    
    // Register drone (called when drone connects)
    void registerDrone(const DroneFleetInfo& info) {
        std::lock_guard<std::mutex> lock(fleet_mutex);
        drones[info.drone_id] = info;
        std::cout << "Registered drone: " << info.drone_id << std::endl;
    }
    
    // Update drone info from telemetry
    void updateDroneFromTelemetry(const std::string& drone_id, const json& telemetry) {
        handleTelemetry(drone_id, telemetry);
    }
    
    // Monitoring thread
    void monitoringLoop() {
        while (api_running) {
            std::this_thread::sleep_for(std::chrono::seconds(60));
            
            // Log statistics
            std::cout << "\n=== Fleet Statistics ===" << std::endl;
            std::cout << "Total drones: " << drones.size() << std::endl;
            std::cout << "Available drones: " << countAvailableDrones() << std::endl;
            std::cout << "Active missions: " << countActiveMissions() << std::endl;
            std::cout << "Missions in queue: " << mission_queue.size() << std::endl;
            std::cout << "Total missions: " << total_missions << std::endl;
            std::cout << "Success rate: " << (total_missions > 0 ? 
                (float)completed_missions / total_missions * 100 : 0) << "%" << std::endl;
            std::cout << "========================\n" << std::endl;
        }
    }
};

#endif // FLEET_MANAGEMENT_HPP