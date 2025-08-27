# ARM Robot 3 DOF (Degrees of freedom) 

## Mục tiêu
Tài liệu này mô tả robot tay được thiết kế trong Onshape với chức năng **gắp đồ vật** (pick-and-place task).

## Phạm vi công việc
- Phân tích thiết kế cơ khí: khớp, cấu trúc, kích thước.
- Xác định end-effector (ngàm/kẹp) để gắp đồ.
- Lập kế hoạch điều khiển và mô phỏng chuyển động (ví dụ: ROS, kinematic, điều khiển tương tác).
- Đánh giá tải trọng, giới hạn vận hành, và biên độ hoạt động.

## Hệ thống cấu thành
- **Khung chính (Base)**: trụ của arm dùng Servo Motor MG996R 
- **Khớp 1 (Joint 1)**: wirst của tay nối tiếp là phần khuỷu tay dùng Servo Motor MG996R 
- **Khớp 2 (Joint 2)**: đóng vai trò là vai tay, nối thẳng đến gripper dùng Servo Motor MG996R 

- **End effector**: khả năng cầm nắm
- **Cảm biến** : camera astra

## Điều khiển và mô phỏng
- **Chiều vận hành**: (Manual control / PLC / ROS / script…)
- **Mô phỏng chuyển động**: có hoặc không, công cụ (Onshape Motion, ROS MoveIt…).
- **Thông số kỹ thuật**: tốc độ tối đa, tải trọng, thời gian vòng quay...

## Hướng dẫn sử dụng
1. Mở bản vẽ trong Onshape. (https://cad.onshape.com/documents/07728d6c31f1d10c2b5ab525/w/f2fad39e8d74a8209da09490/e/5085b4801f4a67a905f3d6da)
2. Xem từng thành phần (khớp, ngàm...).
3. Kiểm tra giới hạn vận hành (góc, tải trọng).
4. Nếu có tích hợp điều khiển (script hoặc mô phỏng), hướng dẫn chạy thử.
