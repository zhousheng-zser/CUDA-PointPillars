#include <cuda_runtime.h>

#include <math.h>
#include <cmath>
#include <iostream>
#include <array>
#include <thread>
#include <mutex>
#include <chrono>
#include <string>
#include <algorithm>
#include <limits>
#include <functional>

#include <httplib.h>
#include <nlohmann/json.hpp>
#include "common/check.hpp"
#include "hsai_sdk_wrapper.h"
#include "detect/draw_meshlab.hpp"
#include "detect/http_server.hpp"
#include "detect/process.hpp"
#include "thread_pool.hpp"
#include "detect/tracking.hpp"
#include "detect/config.hpp"
#include "pointpillar.hpp"

const double PI = 3.141592653589793;

// Thread-safe storage for latest point cloud data
std::vector<float> points_queue;
uint64_t time_queue = 0;
std::mutex queue_lock;
bool running = true;

thread_pool* pool = nullptr;
std::unordered_map<std::thread::id, std::shared_ptr<pointpillar::lidar::Core> > thread_algo_ptr;
cudaStream_t stream;
tracking::MultiObjectTracker* mot = nullptr;

void GetDeviceInfo(void)
{
  cudaDeviceProp prop;

  int count = 0;
  cudaGetDeviceCount(&count);
  printf("\nGPU has cuda devices: %d\n", count);
  for (int i = 0; i < count; ++i) {
    cudaGetDeviceProperties(&prop, i);
    printf("----device id: %d info----\n", i);
    printf("  GPU : %s \n", prop.name);
    printf("  Capbility: %d.%d\n", prop.major, prop.minor);
    printf("  Global memory: %luMB\n", prop.totalGlobalMem >> 20);
    printf("  Const memory: %luKB\n", prop.totalConstMem  >> 10);
    printf("  SM in a block: %luKB\n", prop.sharedMemPerBlock >> 10);
    printf("  warp size: %d\n", prop.warpSize);
    printf("  threads in a block: %d\n", prop.maxThreadsPerBlock);
    printf("  block dim: (%d,%d,%d)\n", prop.maxThreadsDim[0], prop.maxThreadsDim[1], prop.maxThreadsDim[2]);
    printf("  grid dim: (%d,%d,%d)\n", prop.maxGridSize[0], prop.maxGridSize[1], prop.maxGridSize[2]);
  }
  printf("\n");
}

void start_lidar()
{
    try {
        const auto& config = get_config();
        HesaiSDK::InitHesaiSDK(config.lidar_ip_address, config.lidar_roll, config.lidar_pitch, 
                               config.lidar_yaw, config.lidar_x, config.lidar_y, config.lidar_z);
        std::cout << "Hesai SDK started. " << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Hesai error: " << e.what() << std::endl;
    }
}

std::shared_ptr<pointpillar::lidar::Core> create_core() 
{
    pointpillar::lidar::VoxelizationParameter vp;
    vp.min_range = nvtype::Float3(0.0, -39.68f, -3.0);
    vp.max_range = nvtype::Float3(69.12f, 39.68f, 1.0);
    vp.voxel_size = nvtype::Float3(0.16f, 0.16f, 4.0f);
    vp.grid_size =
        vp.compute_grid_size(vp.max_range, vp.min_range, vp.voxel_size);
    vp.max_voxels = 40000;
    vp.max_points_per_voxel = 32;
    vp.max_points = 300000;
    vp.num_feature = 4;

    pointpillar::lidar::PostProcessParameter pp;
    pp.min_range = vp.min_range;
    pp.max_range = vp.max_range;
    pp.feature_size = nvtype::Int2(vp.grid_size.x/2, vp.grid_size.y/2);

    pointpillar::lidar::CoreParameter param;
    param.voxelization = vp;
    param.lidar_model = get_config().lidar_model;
    param.lidar_post = pp;
    auto core =  pointpillar::lidar::create_core(param);
    if (core == nullptr) {
        std::cerr << "Core creation failed in create_core." << std::endl;
        return nullptr;
    }
    core->set_timer(false);
    return core;
}

void loop_get_lidar_data() 
{
    std::cout << "Point cloud collection thread started." << std::endl;

    while (running) {
        try {
            // Get point cloud from Hesai SDK
            std::vector<HesaiSDK::PointData> pc_data = HesaiSDK::GetPointCloudData();
            
            // Wait until we have enough points (similar to Python version)
            int attempts = 0; 
            while (pc_data.empty() || pc_data.size() < 20000) {
                if (attempts++ > 100) break; // Avoid infinite loop
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                pc_data = HesaiSDK::GetPointCloudData();
            }
            //std::cout << "pc_data.size() = " << pc_data.size() << std::endl;
            if (!pc_data.empty() && pc_data.size() >= 20000) {
                std::vector<float> points_xyzi;
                for (const auto& p : pc_data) 
                {
                    points_xyzi.push_back(p.x);
                    points_xyzi.push_back(p.y);
                    points_xyzi.push_back(p.z);
                    points_xyzi.push_back((float)p.intensity);//  *0.0039215686274  等价除以255.0
                }
                
                // Update latest point cloud data
                {
                    std::lock_guard<std::mutex> lock(queue_lock);
                    points_queue = std::move(points_xyzi);
                    time_queue = pc_data[0].timestamp;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(45)); 
        } catch (const std::exception& e) {
            std::cerr << "Error in loop_get_lidar_data: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            running = false;
        }
    }

    std::cout << "Point cloud collection thread stopped." << std::endl;
}

// Detection function called by HTTP server
http_server::DetectionResult handle_detection_request(const std::string& unique_id, int road_id) {
    http_server::DetectionResult result;
    tracking::MultiObjectTracker::BestResult best={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    std::vector<std::array<float, 4>> rendered_points;
    bool flag =mot->set_unique_id_for_closest_vehicle(unique_id, road_id,rendered_points); //去设置unique_id
    if(flag)
    {
        int T = 30;   // 30*200ms = 6s
        while(mot->result_map_[unique_id].status_code != 1 && T)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            T--;
        }
        if(T == 0)
        { 
            mot->result_map_[unique_id].status_code = 2;
        }
        else
        {
            best = mot->result_map_[unique_id].result;
        }
    }
    result.length = best.length;
    result.width = best.width;
    result.height = best.height;
    result.centre_length = best.centre_l;
    result.centre_width = best.centre_w;
    result.centre_height = best.centre_h;
    result.speed = best.speed;
    result.score = best.score;
    http_server::async_forward_to_other_service(unique_id, best, rendered_points, road_id);
    return result;
}

int cnt_zser = 0 ;
// 检测 前处理->推理->后处理
void detect_task_lidar(std::vector<float> &points, std::vector<pointpillar::lidar::BoundingBox> &bboxes_result) 
{
    bboxes_result.clear();
    
    if (points.empty()) 
        return;
    std::vector<float> points_filtered;
    detect::pre_processing(points, points_filtered, get_config());
    
    std::thread::id id_ = std::this_thread::get_id();
    if (thread_algo_ptr[id_] == nullptr) {
        thread_algo_ptr[id_] = create_core();
        if (!thread_algo_ptr[id_]) {
            std::cerr << "Failed to create detector core" << std::endl;
            return;
        }
    }
    auto ptr = thread_algo_ptr[id_];
    
    if (points_filtered.empty()) 
        return;
    
    int points_size = points_filtered.size() / 4;
    //printf("points_size = %d\n", points_size);
    
    std::vector<pointpillar::lidar::BoundingBox> bboxes = ptr->forward(points_filtered.data(), points_size, stream); 
    detect::post_processing(bboxes, bboxes_result, points_filtered);

    // 暂时不用在这儿画框
    // if(cnt_zser % 100 ==0)
    // {
    //     auto current_time = std::chrono::system_clock::now();
    //     auto current_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    //                             current_time.time_since_epoch())
    //                             .count();
    //     std::string save_pcd_name ="../train/" + std::to_string(current_ms) + ".pcd";
    //     std::vector<std::array<float, 4>> rendered_points;
    //     detect::SaveBoxesAsPCD({}, points_filtered.data(), points_filtered.size()/4, save_pcd_name, get_config().point_cloud_draw_step, rendered_points);
    // }
    // cnt_zser++;
}

// 雷达实时检测
void point_cloud_detect() {
    while (running) {
        try {
            // Get latest point cloud data
            std::vector<float> points;
            uint64_t time;
            {
                std::lock_guard<std::mutex> lock(queue_lock);
                if (!points_queue.empty()) 
                {
                    points = std::move(points_queue);   // 取最新的点云数据，move后队列自动清空
                    time = time_queue;
                }
            }
            if(points.empty()) 
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }
            
             //先一次跑全部车道 之后再看是不是每个车道开个线程去跑
            std::vector<pointpillar::lidar::BoundingBox> bboxes;
            auto result_pool = pool->enqueue(detect_task_lidar, std::ref(points), std::ref(bboxes));
            result_pool.get();
            
            // 计算车道id
            std::vector<tracking::BBox3D> detections_frame;
            for (const auto& box : bboxes) {
                const float center_x = get_config().center_x;
                const float range_x  = get_config().range_x;
                const float max_x    = center_x + range_x;
                const int lane_count = std::max(1, get_config().lane_count);
                const float total_width = range_x * 2.0f;

                int road_id = 1;
                if (lane_count > 1 && total_width > 0.0f) {
                    const float lane_span = total_width / static_cast<float>(lane_count);
                    float best_dist = std::numeric_limits<float>::max();
                    int best_lane = 1;
                    for (int lane = 1; lane <= lane_count; ++lane) {
                        const float lane_center = max_x - (lane - 1 + 0.5f) * lane_span;
                        const float dist = std::fabs(box.x - lane_center);
                        if (dist < best_dist) {
                            best_dist = dist;
                            best_lane = lane;
                        }
                    }
                    road_id = best_lane;
                }

                //雷达坐标   
                // w-长 l-宽 h-高
                // y-前后 x-左右 z-上下
                //车前下点作为车中心
                detections_frame.emplace_back(
                     box.y-0.5*box.w, box.x, box.z,    box.w, box.l, box.h, box.rt, static_cast<float>(road_id), box.score);
            }

            // Run tracking on both frames
            mot->update(detections_frame, time/1000, points);
            // {// roi框
            //     const float cx = get_config().center_x;
            //     const float cy = get_config().center_y;
            //     const float cz = get_config().min_z;
            //     const float w  = get_config().range_x * 2.0f;
            //     const float l  = get_config().range_y * 2.0f;
            //     const float h  = get_config().range_z;
            //     pointpillar::lidar::BoundingBox range_box(cx, cy, cz, w, l, h, 0.0f, -1.0, 1.0f);
            //     bboxes.push_back(range_box);
            // }
            
            // {//推理框
            //     const float cx = 34.56; 
            //     const float cy = 0;
            //     const float cz = -3;
            //     const float w  = 34.56 * 2.0f;
            //     const float l  = 39.680 * 2.0f;
            //     const float h  = 4;
            //     pointpillar::lidar::BoundingBox range_box(cx, cy, cz, w, l, h, 0.0f, -1.0, 1.0f);
            //     bboxes.push_back(range_box);
            // }

            // auto current_time = std::chrono::system_clock::now();
            // auto current_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            //                       current_time.time_since_epoch())
            //                       .count();
            // std::string save_pcd_name =
            //     "../results/detection_" + std::to_string(time) + "_" + std::to_string(current_ms) + ".pcd";
            
        } catch (const std::exception& e) {
            std::cerr << "Error in handle_detection_request: " << e.what() << std::endl;
        }

    }
}


int main(int argc, char** argv) {
    // Set config file path before initializing config
    RangeConfigSingleton::setConfigFilePath("/home/zser/CUDA-PointPillars/config-hsai.json");
    
    // Construct MultiObjectTracker after config is initialized
    // Config is automatically initialized when getInstance() is first called
    mot = new tracking::MultiObjectTracker(0.75f, 20,
        get_config().line_config.start_x,
        get_config().line_config.start_y,
        get_config().line_config.start_z,
        get_config().line_config.end_x,
        get_config().line_config.end_y,
        get_config().line_config.end_z,
        tracking::DimensionStrategy::TRIMMED_MAX);
    
    pool = new thread_pool(1);
    cudaStreamCreate(&stream);
    GetDeviceInfo();
    //启动雷达
    start_lidar();
    //抓雷达数据线程
    std::thread get_lidar_thread(loop_get_lidar_data);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    // 雷达检测线程
    std::thread detect_thread(point_cloud_detect);

    // HTTP服务器线程 接收AII请求并进行检测
    std::thread server_thread(http_server::start_pointcloud_server, "0.0.0.0", 8100, handle_detection_request, &running);
    
    // Keep main thread alive
    try {
        while (running) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    } catch (...) {
        running = false;
    }
    
    // Wait for threads to finish
    get_lidar_thread.join();
    detect_thread.join();
    server_thread.join();
    
    // Stop lidar before exit
    HesaiSDK::UninitHesaiSDK();
    
    // Cleanup CUDA stream
    cudaStreamDestroy(stream);
    
    // Cleanup thread pool
    delete pool;
    
    // Cleanup MultiObjectTracker
    delete mot;
    
    return 0;
}
