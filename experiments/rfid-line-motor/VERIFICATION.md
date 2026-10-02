# Kết quả kiểm tra bài test riêng

Ngày: 2026-09-28. Không upload hoặc chạy motor trên phần cứng trong phiên tạo code.

| Kiểm tra | Trạng thái | Bằng chứng |
| --- | --- | --- |
| Native logic bài test | PASS | 10/10 Unity tests trên TestLogic.cpp thực tế; [log](logs/native.txt) |
| Build esp32dev bài test | PASS | espressif32 6.13.0, Arduino core 2.0.17, MFRC522 1.4.12; RAM 21.760 byte, flash 286.953 byte; [log](logs/build.txt) |
| Validate workspace | PASS | Workspace và firmware guards exit 0; [log](logs/validate.txt) |
| Regression firmware chính | PASS | 42/42 native tests và build esp32dev exit 0; [native](logs/regression-native.txt), [build](logs/regression-build.txt) |
| Bản bàn giao firmware chính khớp source | PASS | export_firmware.py --check xác nhận nội dung/hash không đổi; [log](logs/delivery.txt). Không ghi lại tài liệu chính vì không sửa source chính. |
| RFID/chiều motor/cực tính line trên xe | NOT_RUN | Chưa có board, dây và số đo thực tế |
| Rẽ đúng nhánh và tiếp tục theo line | NOT_RUN | Cần UID thật, vị trí thẻ, hình học cua và hiệu chỉnh PWM/timing |

Unit test không xác nhận khả năng đọc thẻ khi xe chạy, góc quay, driver, mức điện hay thời gian dừng vật lý. Log host được lưu riêng trong thư mục logs của bài test.

SHA-256 của `.pio/build/esp32dev/firmware.bin` bài test:
`5E416E9EB8725D4E9ED8F3E49B29BB1053AB0E239FCF9012AC4EEC7F1C6F38DD`.

Lần thử đầu bị sandbox chặn file khóa cache PlatformIO ngoài workspace (BLOCKED môi trường). Chạy lại với quyền dùng cache được duyệt tự động đã cho kết quả PASS ở trên; không phải lỗi firmware.
