# Giải quyết bài toán DVRP (Dynamic Vehicle Routing Problem) bằng ACS và GRASP

Dự án này cung cấp các thuật toán để mô phỏng và giải quyết bài toán Định tuyến Xe động (Dynamic Vehicle Routing Problem - DVRP). Hệ thống cài đặt hai thuật toán chính:
1. **ACS-DVRP**: Dựa trên Hệ thống Kiến (Ant Colony System) tối ưu hóa cho bài toán DVRP.
2. **GRASP-DVRP**: Thuật toán tìm kiếm cục bộ ngẫu nhiên tham lam (Greedy Randomized Adaptive Search Procedure) áp dụng cho DVRP.

Dự án được viết bằng C++ và đi kèm với bài báo học thuật tham khảo liên quan đến giải pháp ACS cho DVRP.

## Tính năng chính

- Cung cấp giao diện dòng lệnh (CLI) dễ sử dụng để chạy các bài kiểm thử khác nhau.
- Mô phỏng và thử nghiệm hàng loạt với nhiều bộ dữ liệu đầu vào (các files `.dat` trong thư mục `DVRP`).
- Hỗ trợ đánh giá tác động của các tham số cấu hình tới kết quả như: giới hạn lát cắt thời gian (`nts`) và hệ số bảo tồn pheromone (`gamma-r`).
- Chạy các tập kiểm thử nhỏ (mini_test) để kiểm tra luồng thuật toán một cách trực quan qua các log hiển thị màn hình (debug mode).

## Cấu trúc dự án

- `main.cpp`: Điểm vào (entry point) của chương trình, chứa menu điều hướng người dùng.
- `run.cpp`: Chứa các hàm kịch bản để chạy các thử nghiệm, thực hiện đọc dữ liệu, lặp lại các quá trình mô phỏng và tính toán thống kê (Min, Max, Avg) cho các thuật toán.
- `acsdvrp.cpp`: Mã nguồn triển khai chi tiết thuật toán Ant Colony System.
- `graspdvrp.cpp`: Mã nguồn triển khai thuật toán GRASP.
- `data-model.cpp` & `io.cpp`: Định nghĩa các thực thể/cấu trúc dữ liệu và xử lý thao tác I/O đọc ghi file.
- `DVRP/`: Thư mục chứa các file dữ liệu mẫu (`.dat`) được sử dụng làm đầu vào để chạy mô phỏng.
- `3.Ant_colony_system_for_a_dynamic_vehicle.pdf`: Tài liệu tham khảo của thuật toán nghiên cứu.
- `DVRP.exe`: Tập tin thực thi sẵn cho hệ điều hành Windows.

## Yêu cầu hệ thống

- Trình biên dịch C++ hỗ trợ tối thiểu phiên bản **C++17** (do mã nguồn có sử dụng thư viện `<filesystem>`).
- Hệ điều hành: Windows / Linux / macOS.

## Hướng dẫn cài đặt và Biên dịch

Nếu bạn đang sử dụng hệ điều hành Windows, bạn có thể chạy trực tiếp tệp `DVRP.exe`.

Nếu bạn ở trên hệ điều hành khác hoặc muốn tự biên dịch lại từ mã nguồn, hãy làm theo các bước sau:

1. Clone hoặc tải toàn bộ mã nguồn về máy.
2. Mở terminal/command prompt tại thư mục gốc của dự án.
3. Chạy lệnh biên dịch (sử dụng trình biên dịch như `g++`):
   ```bash
   g++ main.cpp -o dvrp -std=c++17
   ```
   *(Lưu ý: Bạn chỉ cần truyền `main.cpp` vào lệnh biên dịch, vì các tệp mã nguồn khác đã được `#include` trực tiếp bên trong `main.cpp`)*
4. Chạy tệp thực thi vừa được tạo ra:
   - Trên Linux/macOS: `./dvrp`
   - Trên Windows: `dvrp.exe`

## Hướng dẫn sử dụng

Khi chạy chương trình, màn hình terminal sẽ hiển thị một menu các kịch bản có sẵn. Nhập một con số tương ứng để chọn kịch bản thực thi:

```text
1. test nts (10..50 step5)
2. test gamma-r (0.1..1.0 step0.1)
3. kiem thu mini
4. test GRASP-DVRP
5. kiem thu mini GRASP-DVRP
Lua chon: 
```

- **Lựa chọn 1**: Chạy thực nghiệm thay đổi số lát cắt thời gian `nts` (từ 10 đến 50) cho thuật toán ACS-DVRP trên tất cả các tệp dữ liệu.
- **Lựa chọn 2**: Chạy thực nghiệm thay đổi hệ số `gamma-r` cho ACS-DVRP trên tất cả các tệp dữ liệu.
- **Lựa chọn 3**: Chạy kịch bản thu nhỏ dùng `mini_test.dat` với ACS-DVRP (có bật hiển thị chi tiết log gỡ lỗi/debug).
- **Lựa chọn 4**: Chạy thực nghiệm đối chiếu sử dụng thuật toán GRASP-DVRP trên tất cả các tệp dữ liệu.
- **Lựa chọn 5**: Chạy kịch bản thu nhỏ dùng `mini_test.dat` với GRASP-DVRP.
- **Lựa chọn 0**: Thoát chương trình.

*Lưu ý: Đối với các bài thử nghiệm tổng quát (1, 2 và 4), chương trình sẽ tự động duyệt qua tất cả các file có đuôi `.dat` trong thư mục `DVRP/`, lặp lại mô phỏng nhiều lần cho mỗi file để đảm bảo tính khách quan và sau đó hiển thị bảng tổng kết lên màn hình.*
