# Test RFID độc lập

Đây là project riêng, không sửa firmware trong `NCKHAGV`. Code dùng đúng chân RC522 trong bảng chân chính thức:

```text
SCK  -> GPIO18
MISO -> GPIO19
MOSI -> GPIO23
SS   -> GPIO5
RST  -> GPIO22
```

RC522 dùng nguồn 3.3V. Mở Serial Monitor ở `115200` baud.

Lệnh:

```text
1  : quét và lưu thẻ vừa quét thành THE 1
s  : chế độ đọc thẻ, không thay quyền
p  : xem UID THE 1 đang lưu
c  : xóa quyền THE 1
?  : xem lại hướng dẫn
```

Sau khi gửi `1`, đưa thẻ vào RC522. UID được lưu vào bộ nhớ NVS của ESP32 nên reset vẫn còn. Khi quét lại, Serial báo `THE 1 HOP LE`; thẻ khác báo `THE KHONG DUOC CAP QUYEN`.

Build/upload từ thư mục này:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -d experiments\rfid-only
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -d experiments\rfid-only -t upload --upload-port COMx
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device monitor -b 115200 --port COMx
```

Thay `COMx` bằng cổng ESP32 thật. Bộ test này chỉ đọc RFID và báo quyền, chưa điều khiển motor hoặc line.
