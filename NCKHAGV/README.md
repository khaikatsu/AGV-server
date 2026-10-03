# Firmware ESP32 AGV

## Cấu trúc source

```text
NCKHAGV/
  platformio.ini
  src/
    main.cpp
    Config.h
    Types.h
    core/
      AGVController.h / .cpp
      ControlLogic.h / .cpp
      CommandCodec.h / .cpp
    motion/
      MotorModule.h / .cpp
      MovementModule.h / .cpp
      LineFollowModule.h / .cpp
    sensors/
      SensorModule.h / .cpp
      EncoderModule.h / .cpp
      RFIDModule.h / .cpp
      CargoModule.h / .cpp
      BatteryModule.h / .cpp
    actuators/
      ServoModule.h / .cpp
      BuzzerModule.h
    network/
      WiFiModule.h / .cpp
      MQTTModule.h / .cpp
  test/test_control/test_main.cpp
```

[main.cpp](src/main.cpp) chỉ khởi tạo và cập nhật controller. Mỗi module có header khai báo API và source triển khai đặt cùng nhau trong nhóm chức năng; còi nhỏ nên triển khai ngay trong header.

[Config.h](src/Config.h) là nguồn GPIO/cấu hình duy nhất; [Types.h](src/Types.h) chứa kiểu dùng chung. Hai header này giữ nguyên vị trí để các project thử nghiệm độc lập tiếp tục dùng cấu hình hiện tại.

PlatformIO tự biên dịch source trong các thư mục con của `src/`. Thư viện bên ngoài vẫn được quản lý bằng `lib_deps` trong [platformio.ini](platformio.ini); các module của xe không phải thư viện tải thêm.

Các lệnh build/test và quy trình kiểm chứng nằm trong [hướng dẫn phát triển](../docs/05-development.md). Việc chia thư mục không thay đổi thuật toán, GPIO hay giao thức và không bao gồm nạp firmware lên xe.
