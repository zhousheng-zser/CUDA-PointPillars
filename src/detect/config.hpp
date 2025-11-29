#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <mutex>
#include <string>

//---------------------------
// 结构体定义
//---------------------------
struct LineConfig {
    float start_x = 0, start_y = 0, start_z = 0;
    float end_x = 0, end_y = 0, end_z = 0;
};

struct RangeConfig {
    float center_x = 0, center_y = 0, center_z = 0;
    float range_x = 0, range_y = 0, range_z = 0;

    float min_x = 0, min_y = 0, min_z = 0;
    float max_x = 0, max_y = 0, max_z = 0;

    float ry = 0;
    int lane_count = 0;
    
    std::string lidar_model;
    
    // Lidar SDK configuration
    float lidar_roll = 0;      // 翻滚
    float lidar_pitch = 0;      // 上下
    float lidar_yaw = 0;        // 转动
    float lidar_x = 0;
    float lidar_y = 0;
    float lidar_z = 0;
    std::string lidar_ip_address;  // lidar ip or pcap file path

    // Web service configuration
    std::string web_service_ip;    // web service IP address
    int web_service_port;           // web service port
    int web_service_timeout_sec;    // 超时时间connection/read/write timeout in seconds

    // Tracking configuration
    float min_distance;             // 最多离触发线多少米 

    // Detection configuration
    float detection_score_threshold;  // 检测框分数阈值（后处理过滤）

    // Draw configuration
    float point_cloud_draw_step;  // 点云绘图步长（用于SaveBoxesAsPCD函数）

    // Ground plane parameters (ax + by + cz + d = 0)
    double ground_plane_a;  // 地平面参数 a
    double ground_plane_b;  // 地平面参数 b
    double ground_plane_c;  // 地平面参数 c
    double ground_plane_d;  // 地平面参数 d

    LineConfig line1_config;
    LineConfig line2_config;
};

//---------------------------
// 单例类定义
//---------------------------
class RangeConfigSingleton {
public:
    // 线程安全、无锁的 C++11 单例
    static RangeConfig& getInstance();
    
    // 设置 JSON 配置文件路径（必须在第一次调用 getInstance() 之前调用）
    static void setConfigFilePath(const std::string& json_file_path);

    RangeConfigSingleton(const RangeConfigSingleton&) = delete;
    RangeConfigSingleton& operator=(const RangeConfigSingleton&) = delete;

private:
    RangeConfigSingleton() = default;
    ~RangeConfigSingleton() = default;

    static void initializeConfig(RangeConfig& config, const std::string& json_file_path);
    static std::string config_file_path_;
};

// 全局简单接口
inline RangeConfig& get_config() {
    return RangeConfigSingleton::getInstance();
}

#endif // CONFIG_HPP
