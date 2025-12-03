#include "http_server.hpp"
#include "config.hpp"
#include <httplib.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <exception>
#include <thread>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>
#include <mutex>

using json = nlohmann::json;

namespace http_server {

// 全局日志文件流和互斥锁
static std::ofstream g_log_file;
static std::mutex g_log_mutex;
static std::string g_current_log_filename;

// 获取当前日期和小时格式的日志文件名：YYYY-MM-DD_HH.log
static std::string get_log_filename() {
    std::time_t now = std::time(nullptr);
    std::tm* timeinfo = std::localtime(&now);
    
    std::ostringstream filename_stream;
    filename_stream << "log/" 
                    << std::setfill('0') << std::setw(4) << (1900 + timeinfo->tm_year) << "-"
                    << std::setfill('0') << std::setw(2) << (timeinfo->tm_mon + 1) << "-"
                    << std::setfill('0') << std::setw(2) << timeinfo->tm_mday << "_"
                    << std::setfill('0') << std::setw(2) << timeinfo->tm_hour << ".log";
    return filename_stream.str();
}

// 确保日志文件已打开（如果小时变化则切换文件）
static void ensure_log_file_open() {
    std::string log_filename = get_log_filename();
    
    // 如果文件流未打开或小时变化，需要打开/切换文件
    if (!g_log_file.is_open() || g_current_log_filename != log_filename) {
        // 关闭旧文件（如果打开）
        if (g_log_file.is_open()) {
            g_log_file.close();
        }
        
        // 创建log文件夹（如果不存在）
        struct stat info;
        if (stat("log", &info) != 0) {
            mkdir("log", 0755);
        }
        
        // 打开新文件（追加模式）
        g_log_file.open(log_filename, std::ios::app);
        g_current_log_filename = log_filename;
    }
}

// 全局日志函数：将内容追加到按小时命名的日志文件中
static void write_log(const std::string& message) {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    
    // 获取当前时间
    std::time_t now = std::time(nullptr);
    std::tm* timeinfo = std::localtime(&now);
    
    // 确保日志文件已打开
    ensure_log_file_open();
    
    // 生成时间戳：YYYY-MM-DD HH:MM:SS
    std::ostringstream timestamp_stream;
    timestamp_stream << std::setfill('0') 
                     << std::setw(4) << (1900 + timeinfo->tm_year) << "-"
                     << std::setfill('0') << std::setw(2) << (timeinfo->tm_mon + 1) << "-"
                     << std::setfill('0') << std::setw(2) << timeinfo->tm_mday << " "
                     << std::setfill('0') << std::setw(2) << timeinfo->tm_hour << ":"
                     << std::setfill('0') << std::setw(2) << timeinfo->tm_min << ":"
                     << std::setfill('0') << std::setw(2) << timeinfo->tm_sec;
    
    // 写入日志文件
    if (g_log_file.is_open()) {
        g_log_file << "[" << timestamp_stream.str() << "] " << message << std::endl;
    }
    
    // 同时输出到控制台
    // std::cout << message << std::endl;
}

void start_pointcloud_server(const std::string& host, int port,DetectorFunc detector, bool* running) 
{
    httplib::Server svr;
    
    // 自定义线程池配置：支持每秒20次请求，每个请求处理6秒
    // 理论需求：20 QPS × 6秒 = 120个并发线程
    // 设置为150个线程以应对峰值和系统开销
    // 队列大小设为0表示无限制，可以接收所有请求
    svr.new_task_queue = [&]() {
        // 创建 ThreadPool：150个线程，队列无限制
        return new httplib::ThreadPool(150, 0);
    };
    
    // 设置超时时间：每个请求处理6秒，设置读写超时为8秒（留有余量）
    // 默认读写超时只有5秒，不足以支持6秒的处理时间
    svr.set_read_timeout(8, 0);   // 读取超时8秒
    svr.set_write_timeout(8, 0);  // 写入超时8秒
    
    
    // Handle POST requests to /pointcloud/detect
    svr.Post("/pointcloud/detect", [detector](const httplib::Request& req, httplib::Response& res) 
    {
        write_log("Body: " + req.body);
        
        // 生成时间戳
        std::time_t now = std::time(nullptr);
        std::tm* timeinfo = std::localtime(&now);
        std::ostringstream time_stream;
        time_stream << std::setfill('0') 
                    << std::setw(4) << (1900 + timeinfo->tm_year) << "-"
                    << std::setfill('0') << std::setw(2) << (timeinfo->tm_mon + 1) << "-"
                    << std::setfill('0') << std::setw(2) << timeinfo->tm_mday << " "
                    << std::setfill('0') << std::setw(2) << timeinfo->tm_hour << ":"
                    << std::setfill('0') << std::setw(2) << timeinfo->tm_min << ":"
                    << std::setfill('0') << std::setw(2) << timeinfo->tm_sec;
        std::string timestamp_str = time_stream.str();
        
        std::ostringstream log_msg;
        log_msg << "[REQUEST_RECEIVED] " << timestamp_str 
                << " - Body: " << req.body;
        write_log(log_msg.str());
        
        try {
            // Parse JSON request
            json req_data = json::parse(req.body);
            
            // Validate request_type
            if (!req_data.contains("req_type") || req_data["req_type"] != "get_point_cloud_detect_request") {
                json error_resp = {
                    {"ret_type", "get_point_cloud_detect_response"},
                    {"ret_header", {
                        {"code", 1},
                        {"message", "Invalid request"}
                    }},
                    {"ret_body", json::object()}
                };
                res.set_content(error_resp.dump(), "application/json");
                return;
            }
            
            // Extract req_body
            if (!req_data.contains("req_body")) {
                json error_resp = {
                    {"ret_type", "get_point_cloud_detect_response"},
                    {"ret_header", {
                        {"code", 1},
                        {"message", "Missing req_body"}
                    }},
                    {"ret_body", json::object()}
                };
                res.set_content(error_resp.dump(), "application/json");
                return;
            }
            
            json req_body = req_data["req_body"];
            std::string unique_id = req_body.value("unique_id", "");
            int road_id = req_body.value("channel", 0);
            
            if (unique_id.empty()) {
                json error_resp = {
                    {"ret_type", "get_point_cloud_detect_response"},
                    {"ret_header", {
                        {"code", 2},
                        {"message", "Missing unique_id"}
                    }},
                    {"ret_body", json::object()}
                };
                res.set_content(error_resp.dump(), "application/json");
                return;
            }
            
            //std::cout << "unique_id: " << unique_id << ", road_id: " << road_id << std::endl;
            
            // Call detector function if available
            DetectionResult result;
            if (detector) {
                result = detector(unique_id, road_id);
            } else {
                // Fallback placeholder
                result.length = 0.0f;
                result.width = 0.0f;
                result.height = 0.0f;
                result.centre_length = 0.0f;
                result.centre_width = 0.0f;
                result.centre_height = 0.0f;
                result.speed = 0.0f;
                result.score = 0.0f;
            }
            
            json success_resp = {
                {"ret_type", "get_point_cloud_detect_response"},
                {"ret_header", {
                    {"code", 0}
                }},
                {"ret_body", {
                    {"PointCloudsMessage", {
                        {"unique_id", unique_id},
                        {"length", result.length*1000},// Convert m to mm
                        {"width", result.width*1000},// Convert m to mm
                        {"height", result.height*1000},// Convert m to mm
                        {"centre_length", result.centre_length*1000},// Convert m to mm
                        {"centre_width", result.centre_width*1000},// Convert m to mm
                        {"centre_height", result.centre_height*1000},// Convert m to mm
                        {"speed", result.speed * 3.6f},  // Convert m/s to km/h
                        {"score", result.score},
                        {"coordinate_system", "ECEF"},
                        {"sensor_type", "LiDAR"},
                        {"timestamp", timestamp_str}
                    }}
                }}
            };
            write_log(success_resp.dump());
            res.set_content(success_resp.dump(), "application/json");
            
        } catch (const json::parse_error& e) {
            json error_resp = {
                {"ret_type", "get_point_cloud_detect_response"},
                {"ret_header", {
                    {"code", 500},
                    {"message", "JSON parse error"}
                }},
                {"ret_body", json::object()}
            };
            res.set_content(error_resp.dump(), "application/json");
        } catch (const std::exception& e) {
            std::ostringstream forward_msg;
            forward_msg <<"Error:" << req.body << " Error processing request: " << e.what()  ;
            write_log(forward_msg.str());
            json error_resp = {
                {"ret_type", "get_point_cloud_detect_response"},
                {"ret_header", {
                    {"code", 500},
                    {"message", "Internal Server Error"}
                }},
                {"ret_body", json::object()}
            };
            res.set_content(error_resp.dump(), "application/json");
        }
    }
    );
    
    std::cout << "[INFO] Starting HTTP server on " << host << ":" << port << std::endl;
    
    try {
        svr.listen(host.c_str(), port);
    } catch (const std::exception& e) {
        std::cerr << "HTTP Server error: " << e.what() << std::endl;
        if (running) {
            *running = false;
        }
    }
    
    std::cout << "HTTP Server stopped." << std::endl;
}

void async_forward_to_other_service(const std::string& unique_id,
                                   const tracking::MultiObjectTracker::BestResult& best,
                                   std::vector<std::array<float, 4>> &point_cloud,
                                   std::vector<std::array<float, 4>> &points_max_car,
                                   int road_id ,const std::string &lidar_tpye) {
    std::thread([unique_id, best, point_cloud = std::move(point_cloud), points_max_car = std::move(points_max_car), road_id, lidar_tpye]() mutable {
        nlohmann::json payload;
        double factor = 1000;
        payload["vehicle_width"] = std::round(best.width * factor);
        payload["vehicle_height"] = std::round(best.height * factor);
        payload["vehicle_length"] = std::round(best.length * factor);
        payload["vehicle_centre_width"] = std::round(best.centre_w * factor);
        payload["vehicle_centre_height"] = std::round(best.centre_h * factor);
        payload["vehicle_centre_length"] = std::round(best.centre_l * factor);
        payload["vehicle_score"] = std::round(best.score * factor) / factor;
        payload["vehicle_speed"] = std::round(best.speed * 3.6f * factor) / factor;
        payload["vehicle_serial_number"] = unique_id;
        payload["vehicle_lidar_type"] = lidar_tpye;
        payload["vehicle_detect_time"] = "";
        write_log(payload.dump());
        // 全图点云
        payload["vehicle_radar_points"] = nlohmann::json::array();
        for (const auto &p : point_cloud) {
            payload["vehicle_radar_points"].push_back(
                {std::round(p[0] * factor) / factor, std::round(p[1] * factor) / factor, 
                std::round(p[2] * factor) / factor, std::round(p[3] * factor) / factor}
            );
        }
        // 最大单车点云
        payload["vehicle_car_points"] = nlohmann::json::array();
        for (const auto &p : points_max_car) {
            payload["vehicle_car_points"].push_back(
                {std::round(p[0] * factor) / factor, std::round(p[1] * factor) / factor, 
                std::round(p[2] * factor) / factor, std::round(p[3] * factor) / factor}
            );
        }

        std::string body = payload.dump();

        const auto& config = get_config();
        const std::string& web_ip = config.web_service_ip;
        const int web_port = config.web_service_port;
        const int timeout_sec = config.web_service_timeout_sec;
        const std::string web_url = "http://" + web_ip + ":" + std::to_string(web_port);

        const int max_attempts = 2;
        for (int attempt = 0; attempt < max_attempts; ++attempt) {
            const bool is_last_attempt = (attempt + 1) == max_attempts;
            try {
                httplib::Client client(web_ip.c_str(), web_port);
                client.set_connection_timeout(timeout_sec, 0);
                client.set_read_timeout(timeout_sec, 0);
                client.set_write_timeout(timeout_sec, 0);

                auto res = client.Post("/api/ocm?type=3", body, "application/json");
                if (res) {
                    std::ostringstream forward_msg;
                    forward_msg << "[FORWARD] Sent to " << web_url
                                << " (unique_id=" << unique_id << ") - status: " << res->status
                                << " body: " << res->body;
                    write_log(forward_msg.str());
                    break;
                } else {
                    auto err = res.error();
                    if (!is_last_attempt) {
                        std::ostringstream timeout_msg;
                        timeout_msg << "[FORWARD] Timeout sending to " << web_url
                                     << " (unique_id=" << unique_id << ") - retrying ("
                                     << (max_attempts - attempt - 1) << " attempts left) - "
                                     << httplib::to_string(err);
                        write_log(timeout_msg.str());
                    } else {
                        std::ostringstream timeout_msg2;
                        timeout_msg2 << "[FORWARD] Timeout sending to " << web_url
                                      << " (unique_id=" << unique_id << ") - max retries reached - "
                                      << httplib::to_string(err);
                        write_log(timeout_msg2.str());
                    }
                }
            } catch (const std::exception &e) {
                if (!is_last_attempt) {
                    std::ostringstream exception_msg;
                    exception_msg << "[FORWARD] Exception while sending (retrying, "
                                  << (max_attempts - attempt - 1) << " attempts left): "
                                  << e.what();
                    write_log(exception_msg.str());
                } else {
                    std::ostringstream exception_msg2;
                    exception_msg2 << "[FORWARD] Exception while sending (max retries reached): "
                                   << e.what();
                    write_log(exception_msg2.str());
                }
            }
        }
    }).detach();
}

} // namespace http_server

