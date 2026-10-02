# Chẩn đoán RC522

Project riêng chỉ chẩn đoán RC522. Không sửa firmware AGV hoặc bài test RFID đang dùng. Chân được lấy từ `NCKHAGV/src/Config.h`.

Đóng Serial Monitor, mở thư mục này bằng PlatformIO rồi upload. Mở monitor ở 115200 baud. Lần đầu sẽ in `VersionReg` tám lần và trạng thái anten; mỗi giây in `WUPA`, sau đó in `ATQA` và `UID` nếu thẻ đáp lại. Đặt một thẻ 13,56 MHz sát mặt anten trong vài giây.

```powershell
& "C:\Users\ACER\.platformio\penv\Scripts\pio.exe" run -t upload --upload-port COMx
& "C:\Users\ACER\.platformio\penv\Scripts\pio.exe" device monitor -b 115200 --port COMx
```

Thay `COMx` bằng cổng thật.

* `VersionReg` lúc đúng lúc sai: nghi nguồn hoặc đường SPI tiếp xúc kém.
* `CANH BAO: antenna tu OFF`: chương trình không yêu cầu tắt anten. Nó sẽ bật lại và in kết quả đọc thanh ghi. Nếu lặp lại, cần kiểm tra nguồn 3,3 V ngay tại RC522 khi đang chạy, chân RST và module. Giá trị `RST_GPIO` là mức logic tại GPIO22 của ESP32, không thay cho phép đo điện áp tại chân module.
* `VersionReg` luôn `82`, anten `ON`, WUPA luôn timeout với thẻ 13,56 MHz chắc chắn hoạt động: nghi phần RF/anten hoặc IC RC522. Mã `82` riêng lẻ chưa chứng minh nguyên nhân.
* WUPA thành công nhưng không có UID: nghi va chạm nhiều thẻ, chất lượng RF hoặc phần đọc UID.
* Có UID: phần cứng đọc được, khi đó quay lại bài test cấp quyền.

Biên dịch chỉ kiểm tra phần mềm. Kết quả vật lý cần xem Serial và thẻ thật.
