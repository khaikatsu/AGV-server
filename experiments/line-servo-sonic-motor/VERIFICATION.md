# Verification

| Kiểm tra | Trạng thái | Bằng chứng |
|---|---|---|
| PlatformIO build | PASS | `platformio run -d experiments/line-servo-sonic-motor`: SUCCESS, RAM 6.7%, Flash 21.4% |
| Workspace validate | PASS | `tools/workspace.ps1 -Action Validate`: workspace và firmware guards PASS |
| Firmware regression tests | PASS | `tools/workspace.ps1 -Action Test`: 42/42 test cases passed |
| Firmware chính build | PASS | `tools/workspace.ps1 -Action Build`: esp32dev SUCCESS |
| BOOT bật/tắt motor | NOT_RUN | Code đã build; cần nhấn nút thật và đo/quan sát PWM sau từng lần nhấn |
| Mapping `100/110` chỉ trái, `001/011` chỉ phải | PASS | Người dùng thử trên xe và xác nhận mapping mới hoạt động đúng; mapping chạy bên phải ở `100/110` trước đó đã FAIL |
| Line và motor thật | NOT_RUN | Cần xe, driver motor và đường line thật |
| Servo quét 0–180 | NOT_RUN | Cần servo và nguồn 5 V thật |
| HC-SR04 đo theo góc | NOT_RUN | Cần cảm biến và mạch chia áp ECHO thật |

