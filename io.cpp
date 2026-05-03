#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string>

// Bộ phân tích file .dat chuyên dụng
Instance load_instance(string path) {
    Instance inst;
    inst.capacity = 100;
    inst.num_vehicles = 50;
    ifstream file(path);
    if (!file.is_open()) return inst;

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

    while (getline(file, line)) {
        line = trim(line);
        if (line.empty()) continue;

        if (line.rfind("NAME:", 0) == 0) {
            inst.name = value_after_colon(line);
            continue;
        }
        if (line.rfind("NUM_VEHICLES:", 0) == 0) {
            inst.num_vehicles = stoi(value_after_colon(line));
            continue;
        }
        if (line.rfind("CAPACITIES:", 0) == 0 && line.rfind("NUM_CAPACITIES", 0) != 0) {
            inst.capacity = stoi(value_after_colon(line));
            continue;
        }

        if (line == "LOCATION_COORD_SECTION" || line == "NODE_COORD_SECTION") {
            coord_section = true;
            demand_section = false;
            time_section = false;
            continue;
        }
        if (line == "DEMAND_SECTION") {
            coord_section = false;
            demand_section = true;
            time_section = false;
            continue;
        }
        if (line == "TIME_AVAIL_SECTION") {
            coord_section = false;
            demand_section = false;
            time_section = true;
            continue;
        }

        if (line.find("_SECTION") != string::npos || line == "EOF") {
            if (line != "LOCATION_COORD_SECTION" && line != "NODE_COORD_SECTION" &&
                line != "DEMAND_SECTION" && line != "TIME_AVAIL_SECTION") {
                coord_section = false;
                demand_section = false;
                time_section = false;
            }
        }

        stringstream ss(line);
        if (coord_section) {
            int id;
            double x, y;
            if (ss >> id >> x >> y) {
                coords[id] = {x, y};
            }
        } else if (demand_section) {
            int id, demand;
            if (ss >> id >> demand) {
                demands[id] = abs(demand);
            }
        } else if (time_section) {
            int id, appearance_time;
            if (ss >> id >> appearance_time) {
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