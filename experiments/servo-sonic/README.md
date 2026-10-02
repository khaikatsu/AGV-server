# Test MG90S quét HC-SR04

Project PlatformIO riêng. Nó đọc các chân servo và siêu âm từ `NCKHAGV/src/Config.h`; không sửa firmware AGV hoặc bài test RFID. Nút BOOT trên ESP32 dùng GPIO0 chỉ trong bài test này. Không dùng motor. Sau khi khởi động, Servo 1/GPIO15 tự quét từ lệnh 0° đến 180° và ngược lại, đồng thời đo khoảng cách ở mỗi bước 15°. Góc lệnh chưa phải góc vật lý đã hiệu chuẩn. Phải bảo đảm gá HC-SR04 không chạm vật gì trong toàn dải quét trước khi cấp nguồn.

## Dây

| Bộ phận | Nối với |
| --- | --- |
| MG90S cam/vàng (signal), chọn Servo 1 | ESP32 GPIO15 |
| MG90S cam/vàng (signal), chọn Servo 2 | ESP32 GPIO17 |
| MG90S cam/vàng (signal), chọn Servo 3 | ESP32 GPIO21 |
| MG90S đỏ (V+) | Nguồn 5 V ngoài đủ dòng cho servo |
| MG90S nâu/đen (GND) | GND nguồn servo; nối chung GND ESP32 |
| HC-SR04 VCC | 5 V theo module thật |
| HC-SR04 GND | GND chung |
| HC-SR04 TRIG | ESP32 GPIO12 |
| HC-SR04 ECHO | Qua chia áp xuống khoảng 3 V rồi vào ESP32 GPIO13 |

Với HC-SR04 xuất ECHO 5 V, dùng điện trở 2,2 kΩ từ ECHO đến nút GPIO13 và 3,3 kΩ từ nút GPIO13 xuống GND; mức HIGH vào ESP32 khoảng 3,0 V. Không nối ECHO 5 V trực tiếp vào ESP32. GPIO12 và GPIO15 là chân boot strap của ESP32; nếu board không khởi động, kiểm tra ngoại vi có kéo sai mức lúc reset. Không cấp servo từ GPIO hoặc chân 3V3 ESP32; dùng nguồn 5 V ngoài và GND chung.

## Chạy

Đóng Serial Monitor trước khi upload. Mở đúng thư mục `experiments/servo-sonic` bằng VS Code/PlatformIO. Nếu terminal không nhận `pio`, dùng đường dẫn đầy đủ:

```powershell
& "C:\Users\ACER\.platformio\penv\Scripts\pio.exe" run -t upload --upload-port COMx
& "C:\Users\ACER\.platformio\penv\Scripts\pio.exe" device monitor -b 115200 --port COMx
```

Thay `COMx` bằng cổng thật. Trong Serial Monitor gửi:

```text
1 / 2 / 3 = chọn chân servo tương ứng GPIO15 / 17 / 21; mặc định Servo 1
c         = đưa servo về 90° và giữ
g         = bắt đầu quét 0–180° liên tục, đo khoảng cách theo góc
0         = dừng quét, đưa servo về 0° và giữ
t         = test HC-SR04 riêng mỗi giây; servo về 90° rồi tắt
s         = ngừng quét, về 90°, chờ 800 ms rồi tắt xung servo
?         = hiện hướng dẫn
```

Không cần gửi `g` sau khi reset: code tự dùng Servo 1/GPIO15 và bắt đầu quét, đồng thời in khoảng cách. Nếu giữ nút BOOT lúc khởi động, tự động quét bị chặn; thả nút rồi gửi `g` để chạy.

Nút **BOOT** trên board cũng làm như lệnh `0`: dừng quét và đưa servo về 0°. Bấm sau khi ESP32 đã khởi động; giữ BOOT khi reset có thể đưa ESP32 vào chế độ nạp. Nút được chống dội 30 ms. Nếu board không có nút BOOT, dùng lệnh Serial `0`.

Ví dụ: servo signal nối GPIO17 thì gửi `2`, sau đó `c` để kiểm tra tâm, rồi `g` để quét. Nếu bất kỳ góc nào chạm cơ khí, ngắt nguồn servo ngay và chỉnh gá trước khi thử lại. `0` giữ servo ở vị trí 0° bằng PWM. Khi gửi `s`, servo về 90°, giữ xung 800 ms để có thời gian di chuyển rồi mới tắt xung; sau đó servo không còn lực giữ vị trí. Muốn đổi Servo 1/2/3, gửi `s`, đợi dòng `SERVO DA VE 90 DO VA DA TAT XUNG`, rồi mới gửi số mới.

Serial in `Goc ... do | ... cm`. `ECHO timeout` nghĩa là không đo được, không phải khoảng cách 0 cm hay không có vật cản. Mỗi lần đo có thể chờ tối đa 25 ms; nút BOOT sẽ được xử lý sau lần đo đang diễn ra. Bài test này chỉ chạy servo và siêu âm, chưa dùng làm vòng điều khiển AGV. PWM thử: 50 Hz, 1000–2000 µs, dải lệnh 0–180° và chờ 350 ms mỗi bước; cần hiệu chuẩn theo MG90S/gá thực tế. Nếu MG90S/gá không chịu được dải đầy đủ, tăng `kMinAngle` và giảm `kMaxAngle` trước khi chạy thực tế.

Nếu quét luôn báo timeout, gửi `t` để tách servo khỏi phép thử. `TRIG_HIGH=1` xác nhận ESP32 đã đặt GPIO12 lên HIGH trong lúc phát xung. `ECHO_TRUOC=0 ECHO_SAU=0 TIMEOUT` nghĩa là GPIO13 không thấy phản hồi; kiểm tra nguồn HC-SR04, thứ tự TRIG/ECHO, cầu chia áp, GND hoặc cảm biến. Nếu ECHO luôn bằng 1, kiểm tra ECHO bị nối lên nguồn hoặc cầu chia áp sai. Phép đọc GPIO chỉ quan sát phía ESP32, không đo được điện áp thực tế trên module.
