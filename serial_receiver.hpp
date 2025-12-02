#ifndef SERIAL_RECEIVER_HPP
#define SERIAL_RECEIVER_HPP

#include <string>
// 使用正确的头文件路径
#include <libserial/SerialPort.h>
#include "serial_comm/message.hpp"

namespace serial_comm {

class SerialReceiver {
public:
    SerialReceiver(const std::string& port_name, uint32_t baud_rate = 115200);
    ~SerialReceiver();
    
    bool open();
    void close();
    bool is_open() const;
    bool read_message(SerialMessage& message);
    void set_timeout(uint32_t timeout_ms);
    
private:
    std::string port_name_;
    uint32_t baud_rate_;
    LibSerial::SerialPort serial_port_;  // 使用正确的命名空间
    
    bool verify_crc16(const uint8_t* data, size_t length, uint16_t crc) const;
    uint16_t calculate_crc16(const uint8_t* data, size_t length) const;
    uint32_t bytes_to_uint32_le(const uint8_t* bytes) const;
    uint16_t bytes_to_uint16_le(const uint8_t* bytes) const;
    void uint32_to_bytes_le(uint32_t value, uint8_t* bytes) const;
    void uint16_to_bytes_le(uint16_t value, uint8_t* bytes) const;
};

} // namespace serial_comm

#endif // SERIAL_RECEIVER_HPP
