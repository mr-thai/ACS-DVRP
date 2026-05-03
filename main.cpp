#include <filesystem>
#include <iostream>

using namespace std;
namespace fs = std::filesystem;

#include "data-model.cpp"
#include "acsdvrp.cpp"
#include "io.cpp"
#include "graspdvrp.cpp"
#include "run.cpp"
#include <ctime>

int main() {
    Config current_cfg;
    current_cfg.q0 = 0.9;
    srand((unsigned)time(NULL));
    int choice;

    while (true) {
        cout << "1. test nts (10..50 step5)" << endl;
        cout << "2. test gamma-r (0.1..1.0 step0.1)" << endl;
        cout << "3. kiem thu mini" << endl;
        cout << "4. test GRASP-DVRP" << endl;
        cout << "5. kiem thu mini GRASP-DVRP" << endl;
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
            case 4:
                run_experiment_grasp(current_cfg);
                break;
            case 5:
                run_mini_test_grasp(current_cfg);
                break;

        }
    }
    return 0;
}