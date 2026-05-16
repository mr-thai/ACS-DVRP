#ifndef DATA_MODEL_CPP_INCLUDED
#define DATA_MODEL_CPP_INCLUDED

#include <string>
#include <vector>
using namespace std;

/*
Các trường chính:
- id: mã định danh nút
- x, y: tọa độ địa lý Descartes
- demand: nhu cầu hàng hóa cần giao (đơn vị)
- appearance_time: thời điểm đơn hàng xuất hiện/đặt hàng (giây)
*/
struct Node {
    int id;
    double x, y;
    int demand;
    int appearance_time; 
};

/*
- name: tên bài toán/file dữ liệu
- capacity: dung tích xe (đơn vị hàng hóa)
- num_vehicles: số lượng xe trong đoàn xe
- nodes: danh sách tất cả các nút (depot + khách hàng)
*/
struct Instance {
    string name;
    int capacity;
    int num_vehicles;
    vector<Node> nodes;
};

/*
- id: mã định danh phương tiện
- last_node: nút cuối cùng mà xe đã/đang phục vụ
- remaining_cap: dung tích còn lại của xe (sau khi đã giao một phần)
- available_at: thời điểm xe có sẵn để tiếp tục phục vụ
- is_idling: cờ đánh dấu xe đang rảnh/chờ (không có đơn hàng pending)
*/
struct Vehicle {
    int id;
    int last_node;
    int remaining_cap;
    double available_at;
    bool is_idling = false;
};

/*
- q0: xác suất lựa chọn thiên lệch trong quy tắc ACS (0.0 - 1.0)
- beta: hệ số ảnh hưởng của heuristic (eta) trong ACS
- rho: hệ số bay hơi pheromone cac lần update (0.0 - 1.0)
- gamma_r: hệ số bảo tồn pheromone giữa các lát cắt thời gian
- m: số lượng kiến (agent) trong ACS [không sử dụng trong bản hiện tại]
- nts: số lượng lát cắt thời gian (time slices)
- T: tổng thời gian mô phỏng (giây)
- T_co: thời điểm deadline cuối cùng để nhận đơn hàng mới (giây)
- T_ac: thời gian tính trước (anticipation) thêm vào deadline lát cắt
- delta_g: chiều rộng Restricted Candidate List (RCL) trong GRASP
*/
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
    double delta_g = 0.75; 
};

#endif 