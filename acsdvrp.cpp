#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

class ACSDVRP {
private:
    vector<Node> nodes;
    int capacity_inst;
    vector<vector<double>> dist;
    vector<vector<double>> eta;
    vector<vector<double>> tau;
    double tau0;
    Config cfg;
    bool debug_mode = false;

    vector<Vehicle> fleet;
    vector<int> committed;
    vector<int> pending;

public:
    void set_debug(bool d) { debug_mode = d; }
    ACSDVRP(const Instance& inst, Config config, int seed = 42)
        : nodes(inst.nodes), capacity_inst(inst.capacity), cfg(config) {
        (void)seed;

        int n = nodes.size();
        dist.assign(n, vector<double>(n));
        eta.assign(n, vector<double>(n));
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                double dx = nodes[i].x - nodes[j].x;
                double dy = nodes[i].y - nodes[j].y;
                dist[i][j] = sqrt(dx * dx + dy * dy);
                eta[i][j] = 1.0 / (dist[i][j] + 1e-9);
            }
        }

        tau0 = 1.0 / (n * 100.0);
        tau.assign(n, vector<double>(n, tau0));

        for (int i = 0; i < inst.num_vehicles; ++i) {
            fleet.push_back({i, 0, inst.capacity, 0.0});
        }
    }

    // Bảo tồn Pheromone giữa các lát cắt thời gian
    void preserve_pheromone() {
        int n = nodes.size();
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                tau[i][j] = (1.0 - cfg.gamma_r) * tau[i][j] + cfg.gamma_r * tau0;
            }
        }
    }

    // Quy tắc chuyển trạng thái của ACS
    int select_next(int curr, const vector<int>& allowed, double beta) {
        if (allowed.empty()) return 0;

        int best_node = allowed[0];
        double max_val = -1.0;
        for (int j : allowed) {
            double eta_val = eta[curr][j];
            double val = (beta == 1.0) ? (tau[curr][j] * eta_val) : (tau[curr][j] * pow(eta_val, beta));
            if (val > max_val || (val == max_val && j < best_node)) {
                max_val = val;
                best_node = j;
            }
        }
        return best_node;
    }

    // Kiểm tra xem nút có đã được phục vụ chưa
    bool is_node_served(int node_idx, const vector<vector<int>>& committed_per_vehicle) const {
        for (const auto& vec : committed_per_vehicle) {
            for (int c : vec) if (c == node_idx) return true;
        }
        for (int p : pending) if (p == node_idx) return true;
        return false;
    }

    // Cập nhật danh sách đơn hàng đã xuất hiện
    void update_pending_orders(double slice_deadline, const vector<vector<int>>& committed_per_vehicle) {
        for (size_t i = 1; i < nodes.size(); ++i) {
            if (!is_node_served(i, committed_per_vehicle) && nodes[i].appearance_time <= slice_deadline && nodes[i].appearance_time < cfg.T_co) {
                pending.push_back(i);
            }
        }
    }

    // Thực hiện tối ưu hóa ACS trong một lát cắt thời gian
    void optimize_current_slice(double slice_duration, const vector<int>& unserved_init, vector<vector<int>>& best_slice_tours, vector<Vehicle>& best_fleet_state, vector<int>& best_slice_vehicle_ids, double& best_slice_cost, bool& found_feasible_solution) {
        vector<Vehicle> slice_start_fleet = fleet;
        best_slice_cost = 1e18;
        found_feasible_solution = false;
        auto slice_end = chrono::steady_clock::now() + chrono::duration<double>(slice_duration);

        while (chrono::steady_clock::now() < slice_end) {
            vector<Vehicle> trial_fleet = slice_start_fleet;
            vector<vector<int>> iteration_tours;
            vector<int> iteration_tour_vehicle;
            vector<int> unserved = unserved_init;

            for (size_t vi = 0; vi < trial_fleet.size(); ++vi) {
                auto& v = trial_fleet[vi];
                vector<int> tour;
                int curr = v.last_node;
                int cap = v.remaining_cap;

                while (!unserved.empty()) {
                    vector<int> feasible;
                    for (int node_idx : unserved) {
                        if (nodes[node_idx].demand <= cap) feasible.push_back(node_idx);
                    }
                    if (feasible.empty()) {
                        if (curr != 0) tour.push_back(0);
                        break;
                    }

                    int next = select_next(curr, feasible, cfg.beta);
                    tau[curr][next] = (1.0 - cfg.rho) * tau[curr][next] + cfg.rho * tau0;

                    tour.push_back(next);
                    cap -= nodes[next].demand;
                    curr = next;
                    unserved.erase(remove(unserved.begin(), unserved.end(), next), unserved.end());
                }

                if (!tour.empty()) {
                    local_search(tour);
                    v.last_node = curr;
                    v.remaining_cap = cap;
                    bool capacity_full = (cap == 0);
                    v.is_idling = !capacity_full && unserved.empty();
                    iteration_tours.push_back(tour);
                    iteration_tour_vehicle.push_back((int)vi);
                }
            }

            if (!unserved.empty()) continue;

            found_feasible_solution = true;
            double current_iter_cost = 0;
            for (auto& t : iteration_tours) {
                int prev = 0;
                for (int n : t) {
                    current_iter_cost += dist[prev][n];
                    prev = n;
                }
                current_iter_cost += dist[prev][0];
            }

            if (current_iter_cost > 0 && current_iter_cost < best_slice_cost) {
                best_slice_cost = current_iter_cost;
                best_slice_tours = iteration_tours;
                best_fleet_state = trial_fleet;
                best_slice_vehicle_ids = iteration_tour_vehicle;

                if (debug_mode) {
                    cout << "  [Slice] Cost: " << fixed << setprecision(2) << current_iter_cost
                         << " | Tours: " << iteration_tours.size() << " | Nodes: ";
                    for (const auto& t : iteration_tours) {
                        for (int n : t) cout << n << " ";
                        cout << "| ";
                    }
                    cout << endl;
                }

                for (int i = 0; i < (int)nodes.size(); ++i) {
                    for (int j = 0; j < (int)nodes.size(); ++j) {
                        tau[i][j] *= (1.0 - cfg.rho);
                    }
                }
                for (auto& t : best_slice_tours) {
                    int prev = 0;
                    for (int n : t) {
                        tau[prev][n] = (1.0 - cfg.rho) * tau[prev][n] + cfg.rho * (1.0 / best_slice_cost);
                        prev = n;
                    }
                }
            }
        }
    }

    // Áp dụng chính sách cam kết và cập nhật trạng thái xe
    void apply_commitment_policy(const vector<vector<int>>& best_slice_tours, const vector<int>& best_slice_vehicle_ids, double slice_deadline, double current_time, vector<vector<int>>& committed_per_vehicle) {
        bool time_beyond_cutoff = (current_time >= cfg.T_co);
        for (auto& v : fleet) {
            bool capacity_full = (v.remaining_cap == 0);
            if (capacity_full) {
                v.last_node = 0;
                v.remaining_cap = capacity_inst;
                v.is_idling = false;
            } else if (time_beyond_cutoff && pending.empty()) {
                v.last_node = 0;
                v.remaining_cap = capacity_inst;
                v.is_idling = false;
            } else if (!capacity_full && pending.empty()) {
                v.is_idling = true;
            }
        }

        vector<int> best_vehicle_ids = committed;
        committed.clear();
        vector<int> next_pending;
        for (size_t t = 0; t < best_slice_tours.size(); ++t) {
            int vid = (t < best_vehicle_ids.size()) ? best_vehicle_ids[t] : -1;
            if (vid < 0 || vid >= (int)committed_per_vehicle.size()) continue;
            for (int node_idx : best_slice_tours[t]) {
                if (nodes[node_idx].appearance_time <= slice_deadline) {
                    committed_per_vehicle[vid].push_back(node_idx);
                }
            }
        }
        for (int node_idx : pending) {
            bool is_committed = false;
            for (const auto& vec : committed_per_vehicle) for (int c : vec) if (c == node_idx) is_committed = true;
            if (!is_committed) next_pending.push_back(node_idx);
        }
        pending.swap(next_pending);

        if (debug_mode && !best_slice_tours.empty()) {
            cout << "  Committed: " << pending.size() << " pending nodes: ";
            for (int p : pending) cout << p << " ";
            cout << endl;
        }
    }

    // Tối ưu hóa cục bộ (Local Search)
    void local_search(vector<int>& tour) {
        if (tour.size() < 2) return;
        bool improved = true;
        int pass = 0;
        const int max_passes = 3;
        while (improved && pass < max_passes) {
            improved = false;
            for (int i = 0; i < (int)tour.size(); ++i) {
                int node = tour[i];
                tour.erase(tour.begin() + i);

                double best_added_cost = 1e18;
                int best_pos = -1;

                for (int j = 0; j <= (int)tour.size(); ++j) {
                    int prev = (j == 0) ? 0 : tour[j - 1];
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

    // Event manager: cập nhật thời gian lát cắt và các đơn hàng đã xuất hiện
    bool event_manager_step(double current_time, double slice_duration, vector<vector<int>>& committed_per_vehicle, double& slice_deadline) {
        slice_deadline = current_time + slice_duration + cfg.T_ac;
        update_pending_orders(slice_deadline, committed_per_vehicle);
        return !pending.empty();
    }

    // ACS solver: tối ưu hóa tuyến trong lát cắt hiện tại
    bool acs_solver_step(double slice_duration, vector<vector<int>>& best_slice_tours, vector<Vehicle>& best_fleet_state, vector<int>& best_slice_vehicle_ids, double& best_slice_cost) {
        bool found_feasible_solution = false;
        optimize_current_slice(slice_duration, pending, best_slice_tours, best_fleet_state, best_slice_vehicle_ids, best_slice_cost, found_feasible_solution);
        return found_feasible_solution;
    }

    // Commitment policy: chốt các tuyến được chọn cho lát cắt
    void commitment_policy_step(const vector<vector<int>>& best_slice_tours, const vector<int>& best_slice_vehicle_ids, double slice_deadline, double current_time, vector<vector<int>>& committed_per_vehicle) {
        apply_commitment_policy(best_slice_tours, best_slice_vehicle_ids, slice_deadline, current_time, committed_per_vehicle);
    }

    // Pheromone strategy: bảo tồn pheromone giữa các lát cắt thời gian
    void pheromone_strategy_step() {
        preserve_pheromone();
    }

    // Chạy mô phỏng thực tế
    double run_simulation() {
        double total_cost = 0.0;
        double slice_duration = cfg.T / cfg.nts;
        vector<vector<int>> committed_per_vehicle(fleet.size());
        pending.clear();

        for (int slice = 0; slice < cfg.nts; ++slice) {
            double current_time = slice * slice_duration;
            double slice_deadline = 0.0;

            // Bước 1: Event manager
            if (!event_manager_step(current_time, slice_duration, committed_per_vehicle, slice_deadline)) {
                pheromone_strategy_step();
                continue;
            }

            // Bước 2: ACS solver
            vector<vector<int>> best_slice_tours;
            vector<Vehicle> best_fleet_state = fleet;
            vector<int> best_slice_vehicle_ids;
            double best_slice_cost = 0.0;
            if (!acs_solver_step(slice_duration, best_slice_tours, best_fleet_state, best_slice_vehicle_ids, best_slice_cost)) {
                pheromone_strategy_step();
                continue;
            }
            fleet = best_fleet_state;
            committed = best_slice_vehicle_ids;

            // Bước 3: Commitment policy
            commitment_policy_step(best_slice_tours, best_slice_vehicle_ids, slice_deadline, current_time, committed_per_vehicle);

            total_cost += best_slice_cost;

            // Bước 4: Pheromone strategy
            pheromone_strategy_step();
        }

        return total_cost;
    }
};