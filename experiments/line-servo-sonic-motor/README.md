# Test line + motor + servo + HC-SR04

Project test độc lập, không sửa firmware chính trong `NCKHAGV/`.

## Hành vi

- Khi bật nguồn, motor giữ STOP. Mỗi lần nhấn nút BOOT trên ESP32 sẽ đổi trạng thái motor: lần một bật, lần hai tắt, lần ba bật lại. Servo và HC-SR04 vẫn quét khi motor bị tắt. Không giữ BOOT trong lúc reset hoặc cấp nguồn.
- Tín hiệu của ba cảm biến line hiện được đọc trực tiếp, không đảo; mức `1` được xem là line đen và mức `0` là nền trắng. `INVERT_LINE_INPUTS` trong `src/TestConfig.h` đang đặt là `false`.
- Chỉ sensor giữa có line: hai bên cùng chạy thẳng. Theo hướng lắp thực tế đã quan sát, `100/110` chỉ chạy cụm bánh trái để cua phải về line; `001/011` chỉ chạy cụm bánh phải để cua trái về line.
- Mất cả ba tín hiệu line hoặc mẫu không rõ: motor dừng ngay; thấy line lại thì tự chạy tiếp.
- Servo lập tức về 90°, chờ 800 ms, về 0°, rồi quét liên tục 0–180–0.
- HC-SR04 đo ở mỗi góc servo. ECHO timeout không làm treo vòng dò line.

## Chân theo `NCKHAGV/src/Config.h`

| Khối | Chân ESP32 |
|---|---|
| Line trái / giữa / phải | GPIO39 / GPIO34 / GPIO35 |
| Motor trái ENA / IN1 / IN2 | GPIO14 / GPIO27 / GPIO26 |
| Motor phải ENB / IN3 / IN4 | GPIO25 / GPIO33 / GPIO32 |
| Servo 1 signal | GPIO15 |
| HC-SR04 TRIG / ECHO | GPIO12 / GPIO13 |

Servo và HC-SR04 dùng nguồn 5 V phù hợp, nhưng phải nối chung GND với ESP32. ECHO của HC-SR04 phải qua mạch chia áp trước GPIO13. Với điện trở 1 kΩ: ECHO → 1 kΩ → GPIO13 → 2 kΩ (hai con 1 kΩ nối tiếp) → GND.

Mặc định code chọn **L298N**. Nếu xe dùng BTS7960, đổi `DRIVER` trong `src/TestConfig.h` thành `Config::MotorDriver::BTS7960`. Nếu một bánh quay ngược, đổi biến `LEFT_MOTOR_INVERTED` hoặc `RIGHT_MOTOR_INVERTED` tương ứng thành `true`.

Hai cụm bánh có thể lệch lực. `LEFT_PWM_TRIM = -20` đang giảm bên trái 20 PWM vì bên trái mạnh hơn; chỉnh giá trị này gần `0` hơn nếu xe lại lệch sang trái, hoặc âm thêm nếu xe vẫn lệch sang phải.

## Test lần đầu

1. Kê bánh xe khỏi mặt đất.
2. Đặt cảm biến giữa lên line đen, hai cảm biến ngoài ở nền trắng.
3. Upload rồi mở monitor 115200 baud.
4. Kiểm tra Serial hiện `pattern=010` khi chỉ cảm biến giữa nằm trên line, rồi kiểm tra hai bên bánh quay tới.
5. Đặt xe xuống đường line và thử ở tốc độ thấp mặc định.

Lệnh Serial:

- `s`: dừng motor, đưa servo về 90° rồi tắt xung servo.
- Nút `BOOT`: bật/tắt quyền chạy motor; sau lệnh `s` phải bấm BOOT để bật lại.
- `g`: khởi động lại quá trình quét servo từ 90°, không tự mở khóa motor.
- `?`: in hướng dẫn.

Build:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -d experiments\line-servo-sonic-motor
```

Hồ sơ tích hợp về sau: [AGV-PROFILE-001](../../docs/requirements/2026-09-30-line-servo-sonic-profile.md).

