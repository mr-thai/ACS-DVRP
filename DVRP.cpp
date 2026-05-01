#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <limits>
#include <fstream>
#include <string>
#include <filesystem>
#include <sstream>
#include <map>

using namespace std;
namespace fs = std::filesystem;

// Cấu trúc dữ liệu cho các nút (khách hàng và kho)
struct Node {
    int id;
    double x, y;
    int demand;
    int appearance_time; // Thời điểm xuất hiện (giây)
};

// Cấu trúc dữ liệu cho một bài toán (bộ test)
struct Instance {
    string name;
    int capacity;
    int num_vehicles;
    vector<Node> nodes;
};

// Cấu trúc dữ liệu cho phương tiện di chuyển
struct Vehicle {
    int id;
    int last_node;      
    int remaining_cap;  
    double available_at; 
};

// Cấu trúc cấu hình thuật toán
struct Config {
    double q0 = 0.9;
    double beta = 1.0;
    double rho = 0.1;
    double gamma_r = 0.3;
    int m = 3;          
    int nts = 25;       
    double T = 1500.0;  
    double T_co = 750.0; 
    double T_ac = 0.0; 
};

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
        for(int i=0; i<n; ++i) {
            for(int j=0; j<n; ++j) {
                double dx = nodes[i].x - nodes[j].x;
                double dy = nodes[i].y - nodes[j].y;
                dist[i][j] = sqrt(dx * dx + dy * dy);
                eta[i][j] = 1.0 / (dist[i][j] + 1e-9);
            }
        }

        tau0 = 1.0 / (n * 100.0); 
        tau.assign(n, vector<double>(n, tau0));

        for(int i=0; i < inst.num_vehicles; ++i) {
            fleet.push_back({i, 0, inst.capacity, 0.0});
        }
    }

    // Bảo tồn Pheromone giữa các lát cắt thời gian
    void preserve_pheromone() {
        int n = nodes.size();
        for(int i=0; i<n; ++i) {
            for(int j=0; j<n; ++j) {
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

    // Chạy mô phỏng thực tế
    double run_simulation() {
        double current_best_total_cost = 0;
        double slice_duration = cfg.T / cfg.nts;

        // per-vehicle committed orders
        vector<vector<int>> committed_per_vehicle(fleet.size());
        pending.clear();

        for (int slice = 0; slice < cfg.nts; ++slice) {
            double current_time = slice * slice_duration;
            double slice_deadline = current_time + slice_duration + cfg.T_ac;

            for (size_t i = 1; i < nodes.size(); ++i) {
                bool already_in = false;
                for (const auto &vec : committed_per_vehicle) for (int c : vec) if (c == (int)i) already_in = true;
                for (int p : pending) if (p == (int)i) already_in = true;

                if (!already_in && nodes[i].appearance_time <= slice_deadline && nodes[i].appearance_time < cfg.T_co) {
                    pending.push_back(i);
                }
            }

            if (pending.empty()) {
                preserve_pheromone();
                continue;
            }

            vector<Vehicle> slice_start_fleet = fleet;
            vector<Vehicle> best_fleet_state = slice_start_fleet;
            vector<vector<int>> best_slice_tours;
            double best_slice_cost = 1e18;
            bool found_feasible_solution = false;

            auto slice_end = chrono::steady_clock::now() + chrono::duration<double>(slice_duration);

            while (chrono::steady_clock::now() < slice_end) {
                vector<Vehicle> trial_fleet = slice_start_fleet;
                vector<vector<int>> iteration_tours;
                vector<int> iteration_tour_vehicle;
                vector<int> unserved = pending;

                for (size_t vi = 0; vi < trial_fleet.size(); ++vi) {
                    auto &v = trial_fleet[vi];
                    vector<int> tour;
                    int curr = v.last_node;
                    int cap = v.remaining_cap;

                    while (!unserved.empty()) {
                        vector<int> feasible;
                        for (int node_idx : unserved) {
                            if (nodes[node_idx].demand <= cap) feasible.push_back(node_idx);
                        }
                        if (feasible.empty()) break;

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
                        iteration_tours.push_back(tour);
                        iteration_tour_vehicle.push_back((int)vi);
                    }
                }

                if (!unserved.empty()) {
                    continue;
                }

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
                    vector<int> best_slice_vehicle_ids = iteration_tour_vehicle;
                    
                    if (debug_mode) {
                        cout << "  [Slice " << slice << "] Cost: " << fixed << setprecision(2) << current_iter_cost 
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
                    committed.clear();
                    for (int id : best_slice_vehicle_ids) committed.push_back(id);
                }
            }

            if (!found_feasible_solution) {
                preserve_pheromone();
                continue;
            }

            fleet = best_fleet_state;

            // Commit per-vehicle: best_slice_tours aligns with vehicle ids stored in 'committed' vector
            vector<int> best_vehicle_ids = committed; // retrieved mapping
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
            // rebuild pending: keep those not yet committed
            for (int node_idx : pending) {
                bool is_committed = false;
                for (const auto &vec : committed_per_vehicle) for (int c : vec) if (c == node_idx) is_committed = true;
                if (!is_committed) next_pending.push_back(node_idx);
            }
            pending.swap(next_pending);
            current_best_total_cost += best_slice_cost;

            if (debug_mode && !best_slice_tours.empty()) {
                cout << "  [Slice " << slice << "] Committed: " << pending.size() << " pending, pending nodes: ";
                for (int p : pending) cout << p << " ";
                cout << endl;
            }

            preserve_pheromone();
        }

        return current_best_total_cost;
    }
};

// Bộ phân tích file .dat chuyên dụng
Instance load_instance(string path) {
    Instance inst;
    inst.capacity = 100; 
    inst.num_vehicles = 50; 
    ifstream file(path);
    if(!file.is_open()) return inst;

    string line;
    bool coord_section = false, demand_section = false, time_section = false;

    map<int, pair<double, double>> coords;
    map<int, int> demands;
    map<int, int> appear_times;

    auto trim = [](string s) {
        size_t first = s.find_first_not_of(" \t\r\n");
        if (first == string::npos) return string();
        size_t last = s.find_last_not_of(" \t\r\n");
        return s.substr(first, last - first + 1);
    };

    auto value_after_colon = [&](const string& s) {
        size_t pos = s.find(':');
        if (pos == string::npos) return string();
        return trim(s.substr(pos + 1));
    };

    while(getline(file, line)) {
        line = trim(line);
        if(line.empty()) continue;

        if(line.rfind("NAME:", 0) == 0) {
            inst.name = value_after_colon(line);
            continue;
        }
        if(line.rfind("NUM_VEHICLES:", 0) == 0) {
            inst.num_vehicles = stoi(value_after_colon(line));
            continue;
        }
        if(line.rfind("CAPACITIES:", 0) == 0 && line.rfind("NUM_CAPACITIES", 0) != 0) {
            inst.capacity = stoi(value_after_colon(line));
            continue;
        }

        if(line == "LOCATION_COORD_SECTION" || line == "NODE_COORD_SECTION") {
            coord_section = true;
            demand_section = false;
            time_section = false;
            continue;
        }
        if(line == "DEMAND_SECTION") {
            coord_section = false;
            demand_section = true;
            time_section = false;
            continue;
        }
        if(line == "TIME_AVAIL_SECTION") {
            coord_section = false;
            demand_section = false;
            time_section = true;
            continue;
        }

        if(line.find("_SECTION") != string::npos || line == "EOF") {
            if (line != "LOCATION_COORD_SECTION" && line != "NODE_COORD_SECTION" &&
                line != "DEMAND_SECTION" && line != "TIME_AVAIL_SECTION") {
                coord_section = false;
                demand_section = false;
                time_section = false;
            }
        }

        stringstream ss(line);
        if(coord_section) {
            int id;
            double x, y;
            if(ss >> id >> x >> y) {
                coords[id] = {x, y};
            }
        } else if(demand_section) {
            int id, demand;
            if(ss >> id >> demand) {
                demands[id] = abs(demand);
            }
        } else if(time_section) {
            int id, appearance_time;
            if(ss >> id >> appearance_time) {
                appear_times[id] = appearance_time;
            }
        }
    }

    map<int, bool> ids;
    for (const auto& kv : coords) ids[kv.first] = true;
    for (const auto& kv : demands) ids[kv.first] = true;
    for (const auto& kv : appear_times) ids[kv.first] = true;

    inst.nodes.clear();
    for (const auto& kv : ids) {
        int id = kv.first;
        double x = 0.0, y = 0.0;
        int demand = 0;
        int appearance_time = 0;

        auto c = coords.find(id);
        if (c != coords.end()) {
            x = c->second.first;
            y = c->second.second;
        }

        auto d = demands.find(id);
        if (d != demands.end()) demand = d->second;

        auto t = appear_times.find(id);
        if (t != appear_times.end()) appearance_time = t->second;

        if (id == 0) appearance_time = 0;
        inst.nodes.push_back({id, x, y, demand, appearance_time});
    }

    return inst;
}

void run_simulation_on_directory(Config cfg) {
    string folder_path = "./DVRP";
    if (!fs::exists(folder_path)) {
        cout << "[LOI] Khong tim thay thu muc " << folder_path << endl;
        return;
    }

    cout << "\n" << left << setw(25) << "Ten File (.dat)" << setw(12) << "Diem" << setw(15) << "Ket qua (Cost)" << endl;
    cout << string(55, '-') << endl;

    for (const auto& entry : fs::directory_iterator(folder_path)) {
        if (entry.path().extension() == ".dat") {
            Instance inst = load_instance(entry.path().string());
            if (inst.nodes.empty()) continue;

            ACSDVRP solver(inst, cfg, 42);
            double result = solver.run_simulation();

            cout << left << setw(25) << entry.path().filename().string() 
                 << setw(12) << inst.nodes.size() 
                 << setw(15) << fixed << setprecision(2) << result << endl;
        }
    }
}

// Option 1: Sweep over nts values (10..50 step 5) with fixed gamma_r=0.3
void run_experiment_nts(Config base_cfg) {
    string folder_path = "./DVRP";
    if (!fs::exists(folder_path)) {
        cout << "[LOI] Khong tim thay thu muc " << folder_path << endl;
        return;
    }

    vector<int> nts_values;
    for (int v = 10; v <= 50; v += 5) nts_values.push_back(v);
    const int repeats = 3;

    cout << "\n" << left << setw(25) << "Ten File" << setw(8) << "nts" << setw(12) << "T/nts" << setw(12) << "t_ls" << setw(12) << "Min" << setw(12) << "Max" << setw(12) << "Avg" << endl;
    cout << string(90, '-') << endl;

    for (const auto& entry : fs::directory_iterator(folder_path)) {
        if (entry.path().extension() == ".dat") {
            Instance inst = load_instance(entry.path().string());
            if (inst.nodes.empty()) continue;

            for (int nts : nts_values) {
                Config cfg = base_cfg;
                cfg.nts = nts;
                cfg.gamma_r = 0.3; // fixed per spec

                double T_div = cfg.T / (double)cfg.nts;
                double t_ls = cfg.T / (6.0 * cfg.nts);

                double minv = numeric_limits<double>::infinity();
                double maxv = -numeric_limits<double>::infinity();
                double sum = 0.0;

                for (int r = 0; r < repeats; ++r) {
                    int seed = 100 + r;
                    ACSDVRP solver(inst, cfg, seed);
                    double res = solver.run_simulation();
                    if (res < minv) minv = res;
                    if (res > maxv) maxv = res;
                    sum += res;
                }
                double avg = sum / repeats;

                cout << left << setw(25) << entry.path().filename().string()
                     << setw(8) << nts
                     << setw(12) << fixed << setprecision(2) << T_div
                     << setw(12) << fixed << setprecision(2) << t_ls
                     << setw(12) << fixed << setprecision(2) << minv
                     << setw(12) << fixed << setprecision(2) << maxv
                     << setw(12) << fixed << setprecision(2) << avg << endl;
            }
        }
    }
}

// Option 2: Sweep gamma_r from 0.1..1.0 step 0.1 with nts fixed at 25
void run_experiment_gamma(Config base_cfg) {
    string folder_path = "./DVRP";
    if (!fs::exists(folder_path)) {
        cout << "[LOI] Khong tim thay thu muc " << folder_path << endl;
        return;
    }

    vector<double> gammas;
    for (int i = 1; i <= 10; ++i) gammas.push_back(0.1 * i);
    const int repeats = 3;

    cout << "\n" << left << setw(25) << "Ten File" << setw(12) << "gamma_r" << setw(12) << "Min" << setw(12) << "Max" << setw(12) << "Avg" << endl;
    cout << string(75, '-') << endl;

    for (const auto& entry : fs::directory_iterator(folder_path)) {
        if (entry.path().extension() == ".dat") {
            Instance inst = load_instance(entry.path().string());
            if (inst.nodes.empty()) continue;

            for (double g : gammas) {
                Config cfg = base_cfg;
                cfg.nts = 25; // fixed per spec
                cfg.gamma_r = g;

                double minv = numeric_limits<double>::infinity();
                double maxv = -numeric_limits<double>::infinity();
                double sum = 0.0;

                for (int r = 0; r < repeats; ++r) {
                    int seed = 200 + r;
                    ACSDVRP solver(inst, cfg, seed);
                    double res = solver.run_simulation();
                    if (res < minv) minv = res;
                    if (res > maxv) maxv = res;
                    sum += res;
                }
                double avg = sum / repeats;

                cout << left << setw(25) << entry.path().filename().string()
                     << setw(12) << fixed << setprecision(2) << g
                     << setw(12) << fixed << setprecision(2) << minv
                     << setw(12) << fixed << setprecision(2) << maxv
                     << setw(12) << fixed << setprecision(2) << avg << endl;
            }
        }
    }
}

// Hàm chạy bộ kiểm thử mini để xác minh công thức toán học
void run_mini_test(Config cfg) {
    Instance inst = load_instance("./DVRP/mini_test.dat");
    if (inst.nodes.empty()) {
        cout << "Khong tai duoc file ./DVRP/mini_test.dat" << endl;
        return;
    }
    
    cfg.nts = 10;
    cfg.T_co = 750.0;
    
    cout << "\n>>> KET QUA KIEM THU MINI (Slice-by-Slice Results):" << endl;
    cout << string(80, '=') << endl;
    
    ACSDVRP solver(inst, cfg, 42);
    solver.set_debug(true);
    double result = solver.run_simulation();
    
    cout << string(80, '=') << endl;
    cout << "TONG CHI PHI: " << fixed << setprecision(2) << result << endl;
}

int main() {
    Config current_cfg;
    current_cfg.q0 = 0.9;
    int choice;
    
    while (true) {
        cout << "\n================= QUAN LY KIEM THU THUC TE ACS-DVRP =================" << endl;
        cout << "1. HIEU CHUAN THAM SO nts (10..50 step5)" << endl;
        cout << "2. TINH CHINH THAM SO gamma_r (0.1..1.0 step0.1)" << endl;
        cout << "3. KIEM THU MINI (xac minh cong thuc toan hoc)" << endl;
        cout << "0. Thoat" << endl;
        cout << "Lua chon: ";
        cin >> choice;

        if (choice == 0) break;

        switch (choice) {
            case 1:
                run_experiment_nts(current_cfg);
                break;
            case 2:
                run_experiment_gamma(current_cfg);
                break;
            case 3:
                run_mini_test(current_cfg);
                break;
            default:
                cout << "Lua chon khong hop le." << endl;
        }
    }
    return 0;
}