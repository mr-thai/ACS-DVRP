#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <random>
#include <limits>
#include <fstream>
#include <string>
#include <filesystem>
#include <sstream>
#include <ctime>

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
    double T_ac = 0.0;  // thời gian cam kết trước
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
    mt19937 rng;

    vector<Vehicle> fleet;
    vector<int> committed; 
    vector<int> pending;   

public:
    ACSDVRP(const Instance& inst, Config config, int seed = 42) 
        : nodes(inst.nodes), capacity_inst(inst.capacity), cfg(config), rng(seed) {
        
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

        uniform_real_distribution<double> dist_q(0.0, 1.0);
        if (dist_q(rng) <= cfg.q0) {
            int best_node = allowed[0];
            double max_val = -1.0;
            for (int j : allowed) {
                double eta_val = eta[curr][j];
                double val = (beta == 1.0) ? (tau[curr][j] * eta_val) : (tau[curr][j] * pow(eta_val, beta));
                if (val > max_val) {
                    max_val = val;
                    best_node = j;
                }
            }
            return best_node;
        } else {
            vector<double> probs;
            double total = 0;
            for (int j : allowed) {
                double eta_val = eta[curr][j];
                double p = (beta == 1.0) ? (tau[curr][j] * eta_val) : (tau[curr][j] * pow(eta_val, beta));
                probs.push_back(p);
                total += p;
            }
            double r = dist_q(rng) * total;
            double current_sum = 0;
            for (size_t i = 0; i < allowed.size(); ++i) {
                current_sum += probs[i];
                if (current_sum >= r) return allowed[i];
            }
        }
        return allowed.back();
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

        committed.clear();
        pending.clear();

        for (int slice = 0; slice < cfg.nts; ++slice) {
            double current_time = slice * slice_duration;
            
            for (size_t i = 1; i < nodes.size(); ++i) {
                bool already_in = false;
                for(int c : committed) if(c == (int)i) already_in = true;
                for(int p : pending) if(p == (int)i) already_in = true;

                if (!already_in && nodes[i].appearance_time <= current_time && nodes[i].appearance_time < cfg.T_co) {
                    pending.push_back(i);
                }
            }

            if (pending.empty()) continue;

            vector<vector<int>> best_slice_tours;
            double best_slice_cost = 1e18;

            int acs_iters = 200;
            vector<int> start_nodes_snapshot;
            for (auto& v : fleet) start_nodes_snapshot.push_back(v.last_node);

            // Thực hiện các vòng lặp ACS trong lát cắt
            for (int iter = 0; iter < acs_iters; ++iter) { 
                // Giữ vị trí hiện tại của xe từ lát cắt trước; chỉ làm mới tải trọng
                for (size_t i = 0; i < fleet.size(); ++i) {
                    fleet[i].last_node = start_nodes_snapshot[i];
                    fleet[i].remaining_cap = capacity_inst;
                }
                
                vector<vector<int>> iteration_tours;
                vector<int> unserved = pending;

                for (auto& v : fleet) {
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
                        v.last_node = curr;
                        v.remaining_cap = cap;
                        local_search(tour);
                        iteration_tours.push_back(tour);
                    }
                }
                
                double current_iter_cost = 0;
                for (auto& t : iteration_tours) {
                    int prev = 0;
                    for (int n : t) { current_iter_cost += dist[prev][n]; prev = n; }
                    current_iter_cost += dist[prev][0];
                }

                if (current_iter_cost < best_slice_cost && current_iter_cost > 0) {
                    best_slice_cost = current_iter_cost;
                    best_slice_tours = iteration_tours;
                }

                if (!best_slice_tours.empty()) {
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

            for(int p : pending) committed.push_back(p);
            pending.clear();
            current_best_total_cost = best_slice_cost;

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
    bool coord_section = false, demand_section = false;

    while(getline(file, line)) {
        if(line.empty()) continue;

        if(line.find("NAME") != string::npos) inst.name = line.substr(line.find(":") + 1);
        else if(line.find("NUM_VEHICLES") != string::npos) inst.num_vehicles = stoi(line.substr(line.find(":") + 1));
        else if(line.find("CAPACITIES") != string::npos) inst.capacity = stoi(line.substr(line.find(":") + 1));
        else if(line.find("LOCATION_COORD_SECTION") != string::npos || line.find("NODE_COORD_SECTION") != string::npos) { 
            coord_section = true; demand_section = false; continue; 
        }
        else if(line.find("DEMAND_SECTION") != string::npos) { 
            coord_section = false; demand_section = true; continue; 
        }
        else if(line.find("DEPOT") != string::npos || line.find("EOF") != string::npos) {
            coord_section = false; demand_section = false;
        }

        stringstream ss(line);
        if(coord_section) {
            int id; double x, y;
            if(ss >> id >> x >> y) {
                inst.nodes.push_back({id, x, y, 0, (id == 0) ? 0 : (rand() % 1200)});
            }
        } else if(demand_section) {
            int id, demand;
            if(ss >> id >> demand) {
                for(auto &n : inst.nodes) if(n.id == id) n.demand = abs(demand);
            }
        }
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

int main() {
    srand(time(0));
    Config current_cfg;
    current_cfg.q0 = 0.9;
    int choice;
    
    while (true) {
        cout << "\n================= QUAN LY KIEM THU THUC TE ACS-DVRP =================" << endl;
        cout << "1. HIEU CHUAN THAM SO nts (10..50 step5)" << endl;
        cout << "2. TINH CHINH THAM SO gamma_r (0.1..1.0 step0.1)" << endl;
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
            default:
                cout << "Lua chon khong hop le." << endl;
        }
    }
    return 0;
}