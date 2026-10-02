# Kết quả kiểm tra

Ngày 2026-09-30. Không upload hay cho servo chạy trên board trong phiên tạo code.

| Kiểm tra | Trạng thái | Bằng chứng |
| --- | --- | --- |
| Build ESP32 bài test, tự quét và đo sau boot | PASS | `pio run --project-dir experiments/servo-sonic -e esp32dev`: exit 0, RAM 21.600 byte, flash 278.345 byte |
| Validate workspace | PASS | `tools/workspace.ps1 -Action Validate`: workspace và firmware guards PASS |
| Regression firmware chính | PASS | `-Action Test`: 42/42 Unity; `-Action Build`: exit 0; `export_firmware.py --check`: source/tài liệu chính khớp |
| Servo/siêu âm trên bàn thử | NOT_RUN | Chưa có kết nối, nguồn, phép đo và log từ xe |

Build không xác nhận chiều quay, góc cơ khí, mức ECHO hay khoảng cách thực tế.
