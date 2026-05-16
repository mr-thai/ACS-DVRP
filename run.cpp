#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>
#include "data-model.cpp"

using namespace std;

Instance load_instance(string path);

/*
input: Config cfg (tham số cấu hình)
output: In kết quả chík chi phí của từng file .dat
Chạy mô phỏng ACS-DVRP trên tất cả các file .dat trong thư mục DVRP và in kết quả
*/
void run_simulation_on_directory(Config cfg) {
    string folder_path = "./DVRP";
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

/*
input: Config base_cfg (tham số cấu hình cơ sở)
output: In bảng kết quả thủ nghiệm với các giá trị nts khác nhau
Thủ nghiệm đến hạn lát cắt (nts): kiểm tra min, max, avg chi phí qua các lần lặp
*/
void run_experiment_nts(Config base_cfg) {
    string folder_path = "./DVRP";
    vector<int> nts_values;
    for (int v = 10; v <= 50; v += 10) nts_values.push_back(v);
    const int repeats = 5;

    cout << "\n" << left << setw(25) << "Ten File" << setw(8) << "nts" << setw(12) << "T/nts" << setw(12) << "t_ls" << setw(12) << "Min" << setw(12) << "Max" << setw(12) << "Avg" << endl;
    cout << string(90, '-') << endl;

    for (const auto& entry : fs::directory_iterator(folder_path)) {
        if (entry.path().extension() == ".dat") {
            Instance inst = load_instance(entry.path().string());
            if (inst.nodes.empty()) continue;

            for (int nts : nts_values) {
                Config cfg = base_cfg;
                cfg.nts = nts;
                cfg.gamma_r = 0.3;

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

/*
input: Config base_cfg (tham số cấu hình cơ sở)
output: In bảng kết quả thủ nghiệm với các giá trị gamma_r khác nhau
Thủ nghiệm hệ số bảo tồn pheromone (gamma_r): kiểm tra ảnh hưởng đến chi phí
*/
void run_experiment_gamma(Config base_cfg) {
    string folder_path = "./DVRP";
    if (!fs::exists(folder_path)) {
        cout << "[LOI] Khong tim thay thu muc " << folder_path << endl;
        return;
    }

    vector<double> gammas;
    for (int i = 1; i <= 10; ++i) gammas.push_back(0.1 * i);
    const int repeats = 5;

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

/*
input: Config cfg (tham số cấu hình)
output: In kết quả chi tiết quá trình chạy mô phỏng và tổng chi phí
Chạy một bài kiểm tra nhỏ với debug mode bật để kiểm tra cải tiến và debug
*/
void run_mini_test(Config cfg) {
    Instance inst = load_instance("./DVRP/mini_test.dat");
    if (inst.nodes.empty()) {
        cout << "Khong tai duoc file ./DVRP/mini_test.dat" << endl;
        return;
    }

    cfg.nts = 10;
    cfg.T_co = 750.0;

    cout << "\n ket qua:" << endl;

    ACSDVRP solver(inst, cfg, 42);
    solver.set_debug(true);
    double result = solver.run_simulation();

    cout << string(80, '=') << endl;
    cout << "TONG CHI PHI: " << fixed << setprecision(2) << result << endl;
}

/*
input: Config base_cfg (tham số cấu hình cơ sở)
output: In bảng kết quả chi phí GRASP cho từng file .dat
Chạy thuật toán GRASP-DVRP trên tất cả các file .dat và so sánh với ACS
*/
void run_experiment_grasp(Config base_cfg) {
    string folder_path = "./DVRP";
    if (!fs::exists(folder_path)) {
        cout << "[LOI] Khong tim thay thu muc " << folder_path << endl;
        return;
    }

    const int repeats = 5;

    cout << "\n" << left << setw(25) << "Ten File (.dat)"
         << setw(12) << "Min"
         << setw(12) << "Max"
         << setw(12) << "Avg" << endl;
    cout << string(61, '-') << endl;

    for (const auto& entry : fs::directory_iterator(folder_path)) {
        if (entry.path().extension() == ".dat") {
            Instance inst = load_instance(entry.path().string());
            if (inst.nodes.empty()) continue;

            double minv = numeric_limits<double>::infinity();
            double maxv = -numeric_limits<double>::infinity();
            double sum = 0.0;

            for (int r = 0; r < repeats; ++r) {
                GRASPDVRP solver(inst, base_cfg);
                double result = solver.run_simulation();
                if (result < minv) minv = result;
                if (result > maxv) maxv = result;
                sum += result;
            }

            double avg = sum / repeats;

            cout << left << setw(25) << entry.path().filename().string()
                 << setw(12) << fixed << setprecision(2) << minv
                 << setw(12) << fixed << setprecision(2) << maxv
                 << setw(12) << fixed << setprecision(2) << avg << endl;
        }
    }
}

/*
input: Config cfg (tham số cấu hình)
output: In tổng chi phí giải pháp GRASP
Chạy mô phỏng GRASP-DVRP trên file mini_test.dat để so sánh với ACS
*/
void run_mini_test_grasp(Config cfg) {
    Instance inst = load_instance("./DVRP/mini_test.dat");
    if (inst.nodes.empty()) {
        cout << "Khong tai duoc file ./DVRP/mini_test.dat" << endl;
        return;
    }

    cfg.nts = 10;
    cfg.T_co = 750.0;

    cout << "\n ket qua:" << endl;

    GRASPDVRP solver(inst, cfg);
    double result = solver.run_simulation();

    cout << "tong chi phi: " << fixed << setprecision(2) << result << endl;
}