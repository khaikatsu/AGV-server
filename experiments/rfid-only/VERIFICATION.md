# Kết quả kiểm tra RFID độc lập

- **PASS — Build:** `platformio run -d experiments\rfid-only`, ngày 2026-10-02.
- **PASS — Workspace validate:** `tools\workspace.ps1 -Action Validate`, ngày 2026-10-02.
- **NOT_RUN — Đọc UID bằng RC522 thật:** cần nạp firmware và quét thẻ trên phần cứng.
- **NOT_RUN — Lưu quyền THẺ 1 qua reset:** cần gửi lệnh `1`, quét thẻ, reset ESP32 rồi quét lại.

Kết quả build chỉ xác nhận mã nguồn biên dịch được; chưa chứng minh module RC522, nguồn và dây nối đang hoạt động.
