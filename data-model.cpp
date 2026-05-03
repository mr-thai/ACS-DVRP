#ifndef DATA_MODEL_CPP_INCLUDED
#define DATA_MODEL_CPP_INCLUDED

#include <string>
#include <vector>

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
    bool is_idling = false;
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
    double delta_g = 0.75; // GRASP parameter (RCL width)
};

#endif // DATA_MODEL_CPP_INCLUDED