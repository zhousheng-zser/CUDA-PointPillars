#ifndef DBSCAN_HPP
#define DBSCAN_HPP

#include <vector>
#include <array>
#include <algorithm>
#include <limits>
#include "common/dtype.hpp"

namespace detect {

// 简单的 3D KD-tree 用于加速半径搜索
// 时间复杂度：构建 O(n log n)，查询 O(log n + k)，其中 k 是邻居数量
class SimpleKDTree3D {
private:
    struct Node {
        int point_idx;
        int axis;  // 分割轴：0=x, 1=y, 2=z
        Node* left;
        Node* right;
        Node(int idx, int a) : point_idx(idx), axis(a), left(nullptr), right(nullptr) {}
    };
    
    const std::vector<nvtype::Float3>* points_;
    Node* root_;
    
    // 计算两点之间的欧氏距离平方
    static float distance_squared(const nvtype::Float3& p1, const nvtype::Float3& p2) {
        float dx = p1.x - p2.x;
        float dy = p1.y - p2.y;
        float dz = p1.z - p2.z;
        return dx * dx + dy * dy + dz * dz;
    }
    
    // 获取点在指定轴上的值
    static float get_axis_value(const nvtype::Float3& p, int axis) {
        if (axis == 0) return p.x;
        if (axis == 1) return p.y;
        return p.z;
    }
    
    // 构建 KD-tree（递归）
    Node* build_tree(std::vector<int>& indices, int depth, int start, int end) {
        if (start >= end) return nullptr;
        
        int axis = depth % 3;  // 轮换使用 x, y, z 轴
        
        // 使用 nth_element 找到中位数：O(n) 平均复杂度
        int mid = start + (end - start) / 2;
        std::nth_element(
            indices.begin() + start,
            indices.begin() + mid,
            indices.begin() + end,
            [this, axis](int a, int b) {
                return get_axis_value((*points_)[a], axis) < get_axis_value((*points_)[b], axis);
            }
        );
        
        Node* node = new Node(indices[mid], axis);
        node->left = build_tree(indices, depth + 1, start, mid);
        node->right = build_tree(indices, depth + 1, mid + 1, end);
        
        return node;
    }
    
    // 半径搜索（递归）
    void radius_search_recursive(Node* node, const nvtype::Float3& query, 
                                 float eps_squared, std::vector<int>& result) const {
        if (node == nullptr) return;
        
        const nvtype::Float3& node_point = (*points_)[node->point_idx];
        float dist_sq = distance_squared(query, node_point);
        
        if (dist_sq <= eps_squared && dist_sq > 0.0f) {  // 排除自己
            result.push_back(node->point_idx);
        }
        
        int axis = node->axis;
        float query_axis_val = get_axis_value(query, axis);
        float node_axis_val = get_axis_value(node_point, axis);
        float diff = query_axis_val - node_axis_val;
        float diff_sq = diff * diff;
        
        // 决定搜索哪个子树
        Node* near = (query_axis_val < node_axis_val) ? node->left : node->right;
        Node* far = (query_axis_val < node_axis_val) ? node->right : node->left;
        
        if (near) {
            radius_search_recursive(near, query, eps_squared, result);
        }
        
        // 如果分割平面与查询球相交，也需要搜索远子树
        if (far && diff_sq <= eps_squared) {
            radius_search_recursive(far, query, eps_squared, result);
        }
    }
    
    // 清理树
    void destroy_tree(Node* node) {
        if (node) {
            destroy_tree(node->left);
            destroy_tree(node->right);
            delete node;
        }
    }
    
public:
    SimpleKDTree3D(const std::vector<nvtype::Float3>* points) : points_(points), root_(nullptr) {
        if (points->empty()) return;
        
        // 创建索引数组
        std::vector<int> indices(points->size());
        for (size_t i = 0; i < indices.size(); ++i) {
            indices[i] = i;
        }
        
        // 构建树
        root_ = build_tree(indices, 0, 0, indices.size());
    }
    
    ~SimpleKDTree3D() {
        destroy_tree(root_);
    }
    
    // 半径搜索：找到 query 点 eps 范围内的所有点
    std::vector<int> radius_search(int query_idx, float eps) const {
        std::vector<int> result;
        if (root_ == nullptr || query_idx < 0 || query_idx >= (int)points_->size()) {
            return result;
        }
        
        float eps_squared = eps * eps;
        const nvtype::Float3& query = (*points_)[query_idx];
        radius_search_recursive(root_, query, eps_squared, result);
        
        return result;
    }
};

// DBSCAN 聚类：过滤掉点数远小于最大簇且z值较高的簇（如树叶等高空噪声）
// 时间复杂度：O(n log n)，使用 KD-tree 优化邻居查找
//   - 构建 KD-tree: O(n log n)
//   - 每次半径查询: O(log n + k)，其中 k 是邻居数量
//   - n 次查询: O(n log n + nk)
// 对于 15000 个点，约 15000 * log₂(15000) ≈ 210,000 次操作
// 相比 O(n²) 的 225,000,000 次操作，性能提升约 1000 倍
// 直接修改传入的 points_in_box 和 box_points（若过滤后为空则清空），无返回值
void dbscan_filter(
    std::vector<nvtype::Float3>& points_in_box,
    std::vector<std::array<float, 4>>& box_points,
    float eps,
    float max_cluster_ratio,
    float z_threshold)
{
    if (points_in_box.empty() || points_in_box.size() != box_points.size()) {
        points_in_box.clear();
        box_points.clear();
        return;
    }

    std::vector<nvtype::Float3> filtered_points;
    std::vector<int> filtered_indices;
    
    const int n = points_in_box.size();
    const int min_points = 2;  // 固定值：至少需要2个邻居才能成为核心点
    
    // 构建 KD-tree：O(n log n)
    SimpleKDTree3D kdtree(&points_in_box);
    
    // 标记：-1=未访问, 0=噪声, >0=簇ID
    std::vector<int> labels(n, -1);
    int cluster_id = 0;
    
    // 找到点的所有邻居（在 eps 范围内）- 使用 KD-tree 优化：O(log n + k)
    auto get_neighbors = [&](int point_idx) -> std::vector<int> {
        return kdtree.radius_search(point_idx, eps);
    };
    
    // DBSCAN 主算法
    for (int i = 0; i < n; ++i) {
        if (labels[i] != -1) continue;  // 已访问过
        
        // 找到邻居
        std::vector<int> neighbors = get_neighbors(i);
        
        if (neighbors.size() < min_points) {
            // 标记为噪声（暂时，后续可能被其他簇吸收）
            labels[i] = 0;
            continue;
        }
        
        // 创建新簇
        cluster_id++;
        labels[i] = cluster_id;
        
        // 扩展簇：使用队列处理所有密度可达的点
        std::vector<int> seed_set = neighbors;
        for (size_t j = 0; j < seed_set.size(); ++j) {
            int neighbor_idx = seed_set[j];
            
            if (labels[neighbor_idx] == 0) {
                // 噪声点被重新标记为当前簇
                labels[neighbor_idx] = cluster_id;
            } else if (labels[neighbor_idx] != -1) {
                // 已属于其他簇，跳过
                continue;
            }
            
            // 标记为当前簇
            labels[neighbor_idx] = cluster_id;
            
            // 检查邻居的邻居（密度可达）
            std::vector<int> neighbor_neighbors = get_neighbors(neighbor_idx);
            if (neighbor_neighbors.size() >= min_points) {
                // 将新发现的邻居加入队列
                for (int nn : neighbor_neighbors) {
                    if (labels[nn] == -1 || labels[nn] == 0) {
                        seed_set.push_back(nn);
                    }
                }
            }
        }
    }
    
    // 统计每个簇的点数和最大z值
    std::vector<int> cluster_counts(cluster_id + 1, 0);
    std::vector<float> cluster_max_z(cluster_id + 1, -std::numeric_limits<float>::infinity());
    
    for (int i = 0; i < n; ++i) {
        if (labels[i] > 0) {
            cluster_counts[labels[i]]++;
            cluster_max_z[labels[i]] = std::max(cluster_max_z[labels[i]], points_in_box[i].z);
        }
    }
    
    // 找到最大簇的点数和最大z值（主簇）
    int max_cluster_size = 0;
    int max_cluster_id = 0;
    for (int cid = 1; cid <= cluster_id; ++cid) {
        if (cluster_counts[cid] > max_cluster_size) {
            max_cluster_size = cluster_counts[cid];
            max_cluster_id = cid;
        }
    }
    
    // 获取主簇的最大z值
    float main_cluster_max_z = (max_cluster_id > 0) ? cluster_max_z[max_cluster_id] : 0.0f;
    float z_limit = main_cluster_max_z + z_threshold;  // 如果簇的最大z值 > 主簇最大z值 + z_threshold，则删除
    
    // 过滤条件：保留满足以下条件的簇
    // 1. 簇的点数 >= 最大簇点数 * max_cluster_ratio，或者
    // 2. 簇的最大z值 <= 主簇最大z值 + z_threshold（不是高空噪声）
    float size_threshold = max_cluster_size * max_cluster_ratio;
    
    for (int i = 0; i < n; ++i) {
        if (labels[i] > 0) {
            int cid = labels[i];
            int cluster_size = cluster_counts[cid];
            
            // 保留条件：点数足够大 或 z值不超过主簇的z_threshold范围（不是高空噪声）
            if (cluster_size >= size_threshold || points_in_box[i].z <= z_limit) {
                filtered_points.push_back(points_in_box[i]);
                filtered_indices.push_back(i);
            }
        }
    }
    
    // 如果过滤后为空，直接清空并返回
    if (filtered_points.empty()) {
        points_in_box.clear();
        box_points.clear();
        return;
    }
    
    // 更新传入的参数
    points_in_box = std::move(filtered_points);
    std::vector<std::array<float, 4>> filtered_box_points;
    for (int idx : filtered_indices) {
        filtered_box_points.push_back(box_points[idx]);
    }
    box_points = std::move(filtered_box_points);
}

} // namespace detect

#endif // DBSCAN_HPP

