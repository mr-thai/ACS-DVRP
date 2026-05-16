#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

using namespace std;
#
#include "data-model.cpp"
class GRASPDVRP {
private:
    vector<Node> nodes;
    int capacity_inst;
    vector<vector<double>> dist;
    Config cfg;

    vector<Vehicle> fleet;

    /*
    input: danh sách các nút
    output: ma trận khoảng cách Euclidean)
    Tính toán ma trận khoảng cách Euclidean giữa tất cả các cặp nút
    */
    void build_dist() {
        int n = nodes.size();
        dist.assign(n, vector<double>(n));
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                double dx = nodes[i].x - nodes[j].x;
                double dy = nodes[i].y - nodes[j].y;
                dist[i][j] = sqrt(dx * dx + dy * dy);
            }
        }
    }

    /*
    input: tour (tuyến xe), time_limit_sec (giới hạn thời gian)
    output: tour được cải thiện bằng local search
    Áp dụng local search trong giới hạn thời gian để cải thiện tuyến xe
    */
    void local_search_impl(vector<int>& tour, double time_limit_sec) {
        if (tour.size() < 2) return;
        auto search_start = chrono::steady_clock::now();
        bool improved = true;
        int pass = 0;
        const int max_passes = 3;
        while (improved && pass < max_passes) {
            if (chrono::duration<double>(chrono::steady_clock::now() - search_start).count() >= time_limit_sec) {
                break;
            }
            improved = false;
            for (int i = 0; i < (int)tour.size(); ++i) {
                if (chrono::duration<double>(chrono::steady_clock::now() - search_start).count() >= time_limit_sec) {
                    break;
                }
                int node = tour[i];
                tour.erase(tour.begin() + i);

                double best_added_cost = 1e18;
                int best_pos = -1;

                for (int j = 0; j <= (int)tour.size(); ++j) {
                    int prev = (j == 0) ? 0 : tour[j-1];
                    int n_node = (j < (int)tour.size()) ? tour[j] : 0;
                    double added = dist[prev][node] + dist[node][n_node] - dist[prev][n_node];
                    if (added < best_added_cost) {
                        best_added_cost = added;
                        best_pos = j;
                    }
                }
                tour.insert(tour.begin() + best_pos, node);
                if (best_pos != i) improved = true;
            }
            ++pass;
        }
    }

    /*
    input: tours (danh sách các tuyến xe)
    output: total distance (tổng khoảng cách của tất cả các tuyến)
    Tính tổng khoảng cách từ depot đến tất cả các nút và quay về depot
    */
    double calculate_total_dist(const vector<vector<int>>& tours) const {
        double total = 0.0;
        for (const auto& t : tours) {
            int prev = 0;
            for (int n : t) {
                total += dist[prev][n];
                prev = n;
            }
            total += dist[prev][0];
        }
        return total;
    }

    /*
    input: curr (nút hiện tại), allowed (danh sách nút có thể chọn)
    output: next_node (nút được chọn theo quy tắc GRASP)
    Chọn nút tiếp theo từ Restricted Candidate List (RCL) dựa trên tham số delta_g
    */
    int select_next_grasp(int curr, const vector<int>& allowed) const {
        if (allowed.empty()) return 0;

        double tt_min = numeric_limits<double>::infinity();
        double tt_max = -numeric_limits<double>::infinity();
        for (int j : allowed) {
            double d = dist[curr][j];
            if (d < tt_min) tt_min = d;
            if (d > tt_max) tt_max = d;
        }
        double threshold = tt_min + cfg.delta_g * (tt_max - tt_min);
        vector<int> rcl;
        for (int j : allowed) if (dist[curr][j] <= threshold) rcl.push_back(j);
        if (rcl.empty()) return allowed[rand() % allowed.size()];
        return rcl[rand() % rcl.size()];
    }

    /*
    input: pending (danh sách nút chưa phục vụ)
    output: tours (danh sách các tuyến xe được xây dựng)
    Xây dựng một giải pháp ngẫu nhiên bằng cách gán các nút cho xe theo quy tắc GRASP
    */
    vector<vector<int>> construct_randomized_solution(const vector<int>& pending) {
        vector<int> unserved = pending;
        vector<vector<int>> tours;

        for (size_t vi = 0; vi < fleet.size(); ++vi) {
            int cap = fleet[vi].remaining_cap;
            int curr = fleet[vi].last_node;
            vector<int> tour;

            while (!unserved.empty()) {
                vector<int> feasible;
                for (int node_idx : unserved) if (nodes[node_idx].demand <= cap) feasible.push_back(node_idx);
                if (feasible.empty()) {
                    if (curr != 0) tour.push_back(0);
                    break;
                }
                int next = select_next_grasp(curr, feasible);
                tour.push_back(next);
                cap -= nodes[next].demand;
                curr = next;
                unserved.erase(remove(unserved.begin(), unserved.end(), next), unserved.end());
            }
            if (!tour.empty()) {
                tours.push_back(tour);
            }
        }
        return tours;
    }

public:
    /*
    input: Instance inst (nodes, capacity, vehicles), Config config
    output: Initialized GRASPDVRP object với distance matrix
    Khởi tạo giải thuật GRASP-DVRP với dữ liệu bài toán và tham số cấu hình
    */
    GRASPDVRP(const Instance& inst, Config config) : nodes(inst.nodes), capacity_inst(inst.capacity), cfg(config) {
        build_dist();
        for (int i = 0; i < inst.num_vehicles; ++i) fleet.push_back({i, 0, inst.capacity, 0.0});
    }

    /*
    input: Instance data, Config parameters
    output: total_cost (tổng chi phí của tất cả các tuyến)
    Chạy mô phỏng toàn bộ quá trình DVRP theo thời gian thực, qua từng lát cắt thời gian bằng thuật toán GRASP
    */
    double run_simulation() {
        double total_cost = 0.0;
        double slice_duration = cfg.T / cfg.nts;
        double t_ls_limit = cfg.T / (6.0 * cfg.nts);

        vector<vector<int>> committed_per_vehicle(fleet.size());

        for (int slice = 0; slice < cfg.nts; ++slice) {
            double current_time = slice * slice_duration;
            double slice_deadline = current_time + slice_duration + cfg.T_ac;

            vector<int> pending;
            for (size_t i = 1; i < nodes.size(); ++i) {
                if (nodes[i].appearance_time <= slice_deadline && nodes[i].appearance_time < cfg.T_co) pending.push_back(i);
            }

            if (pending.empty()) continue;

            auto slice_start = chrono::steady_clock::now();
            double best_slice_cost = numeric_limits<double>::infinity();
            vector<vector<int>> best_slice_tours;

            while (chrono::duration<double>(chrono::steady_clock::now() - slice_start).count() < slice_duration) {
                auto tours = construct_randomized_solution(pending);
                for (auto& t : tours) local_search_impl(t, t_ls_limit);
                double cost = calculate_total_dist(tours);
                if (cost < best_slice_cost) {
                    best_slice_cost = cost;
                    best_slice_tours = tours;
                }
            }

            if (best_slice_cost < numeric_limits<double>::infinity()) total_cost += best_slice_cost;
            for (size_t t = 0; t < best_slice_tours.size(); ++t) {
                int vid = (int)t % (int)committed_per_vehicle.size();
                for (int node_idx : best_slice_tours[t]) {
                    if (nodes[node_idx].appearance_time <= slice_deadline) committed_per_vehicle[vid].push_back(node_idx);
                }
            }
        }

        return total_cost;
    }
};