# Test riêng RFID + line + động cơ

Project PlatformIO riêng, không sửa firmware trong `NCKHAGV`. Chỉ đọc bảng GPIO từ `NCKHAGV/src/Config.h`; giữ nguyên vị trí thư mục này trong workspace khi build. Muốn chuyển sang máy khác, mang theo file Config.h ở cùng cấu trúc đường dẫn. Không dùng Wi-Fi/MQTT, servo, encoder hay siêu âm trong bài test này.

Hành vi mặc định theo mô tả: thẻ 1 → bánh trái tiến, bánh phải lùi (quay tại chỗ sang phải) → cảm biến LEFT rời line ổn định rồi bắt lại line ổn định → chuyển về dò line. Hướng quay phải là hướng vật lý của xe, chưa thể khẳng định đó là rẽ trái theo sơ đồ đường. Muốn quay trái, đổi `CARD_1_PIVOT_RIGHT = false` trong [TestConfig.h](src/TestConfig.h); điều kiện tìm line vẫn là cảm biến LEFT.

## Cách chạy

1. Mở **chính thư mục này** bằng VS Code/PlatformIO, build và upload môi trường `esp32dev` vào đúng ESP32. Upload thay chương trình đang nằm trên board; source firmware chính trên máy vẫn giữ nguyên.
2. Mở Serial Monitor **115200 baud**, reset board để xem hướng dẫn. Boot luôn dừng motor.
3. Chọn driver: gửi `a` nếu **L298N**, hoặc `b` nếu **BTS7960** đấu đúng adapter bên dưới. Mỗi lần chọn driver đều dừng motor.
4. Kê bánh khỏi mặt đất; gửi `f` để test tiến 400 ms, `r` để quay phải 400 ms, `l` để quay trái 400 ms. Gửi `s` để dừng sớm. Nếu chiều bánh sai, sửa `LEFT_FORWARD_INVERTED`/`RIGHT_FORWARD_INVERTED` trong TestConfig.h rồi build/upload lại.
5. Đặt từng cảm biến lên vạch và nền, nhìn `raw LCR`. Gửi `0` nếu cảm biến ra LOW khi gặp vạch, hoặc `1` nếu ra HIGH. `line=100` nghĩa LEFT đang thấy vạch. Ba cảm biến cần cùng cực tính.
6. Gửi `u`, đưa thẻ muốn làm **thẻ 1** vào RC522. Serial in UID và lưu mapping trong RAM. Nếu vừa quét thẻ, nhấc khỏi đầu đọc rồi đưa lại. Có thể điền UID thật vào `CARD_1_UID` để giữ mapping qua reset; không dùng UID ví dụ.
7. Đưa xe lên line rồi gửi `g`: xe bắt đầu dò line. Quét thẻ 1 ở vị trí cua. Trong mỗi lượt `g`, thẻ 1 chỉ kích hoạt quay **một lần**. Thẻ khác chỉ in UID. Gửi `s` rồi `g` để bắt đầu lượt mới; nhấc thẻ ra rồi quét lại.

Không tự chạy nếu chưa chọn driver, cực tính line và UID. Reset làm mất driver/cực tính đã chọn và UID học trong RAM. `u` luôn dừng xe trước khi học thẻ.

## GPIO và dây

GPIO được đọc trực tiếp từ Config.h chính thức:

| Bộ phận | GPIO |
| --- | --- |
| Motor trái ENA / IN1 / IN2 | 14 / 27 / 26 |
| Motor phải ENB / IN3 / IN4 | 25 / 33 / 32 |
| Line LEFT / CENTER / RIGHT | 39 / 34 / 35 |
| RC522 SCK / MISO / MOSI / SS / RST | 18 / 19 / 23 / 5 / 22 |

L298N: ENA/ENB nhận PWM, IN nhận hướng; tháo jumper enable theo board thật. BTS7960: dùng một module cho mỗi phía; ENA/ENB nối chung R_EN và L_EN của module tương ứng; IN1/IN3 → RPWM; IN2/IN4 → LPWM. Không chọn adapter BTS7960 cho dây đấu kiểu L298N.

RC522 cấp 3,3 V. Cảm biến line phải xuất mức phù hợp ESP32; GPIO39/34/35 không có pull nội. Nguồn motor riêng đủ dòng, chung GND. Thử bánh kê trước khi chạy trên đường, có cách ngắt nguồn motor: bài test này không có chức năng dừng theo vật cản.

## Tham số test và hành vi dừng

PWM tiến 100/255, sửa lái 45/255, quay 95/255; thời gian quay tối thiểu 200 ms, timeout 5000 ms, line ổn định 30 ms. Đây là giá trị khởi đầu để chỉnh trên bàn thử, chưa phải hiệu chuẩn xe. Chỉnh trong TestConfig.h.

Quay theo trạng thái và `millis()`, không dùng vòng `while` chờ line. LEFT phải mất vạch liên tục 30 ms, sau đó thấy vạch liên tục 30 ms, và đã quay ít nhất 200 ms. Nếu LEFT không rời vạch hoặc không tìm được vạch mới trong 5 giây, PWM về 0 và giữ lỗi đến lệnh `g` mới. Serial `s` được kiểm tra mỗi loop; thời gian phản ứng thực tế còn phụ thuộc SPI/Serial/scheduler và chưa đo.

Khi dò line: 010 tiến; 100/110 sửa trái; 001/011 sửa phải; **111 tiến** để qua vùng giao line có thẻ (lựa chọn riêng của bài test); 000 và 101 dừng giữ lỗi. Khi đang quay, 000 được phép để tìm nhánh mới. Khi LEFT bắt được vạch, xe bắt đầu sửa trái theo line rồi đi thẳng khi CENTER thấy vạch. Không có bước tiến tới tâm giao lộ trước quay: cần đặt thẻ/đầu đọc ở vị trí trục quay phù hợp.

## Lệnh build và kiểm tra

Từ thư mục này:

```powershell
pio run -e esp32dev
pio test -e native
pio run -e esp32dev -t upload --upload-port COMx
pio device monitor -b 115200 --port COMx
```

Thay COMx bằng cổng thật. Native test gọi đúng TestLogic.cpp được dùng trong firmware test, không gọi bản sao thuật toán. Kết quả build/unit và giới hạn phần cứng được ghi ở [VERIFICATION.md](VERIFICATION.md).
