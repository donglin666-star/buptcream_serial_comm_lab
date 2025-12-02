#include "serial_comm/serial_receiver.hpp"
#include <spdlog/spdlog.h>
#include <cstring>

namespace serial_comm {

// ==================== 构造函数和析构函数 ====================
SerialReceiver::SerialReceiver(const std::string& port_name, uint32_t baud_rate)
    : port_name_(port_name), baud_rate_(baud_rate) {
    SPDLOG_INFO("创建串口接收器: 端口={}, 波特率={}", port_name, baud_rate);
}

SerialReceiver::~SerialReceiver() {
    if (is_open()) {
        close();
    }
}

// ==================== 串口操作函数 ====================
bool SerialReceiver::open() {
    try {
        // 使用LibSerial的正确方式
        serial_port_.Open(port_name_);
        serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);  // 可以根据baud_rate_调整
        
        // 设置其他参数
        serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
        serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        
        SPDLOG_INFO("成功打开串口: {}", port_name_);
        return true;
    } catch (const LibSerial::OpenFailed& e) {
        SPDLOG_ERROR("打开串口失败 {}: {}", port_name_, e.what());
        return false;
    } catch (const std::exception& e) {
        SPDLOG_ERROR("串口操作异常 {}: {}", port_name_, e.what());
        return false;
    }
}

void SerialReceiver::close() {
    if (serial_port_.IsOpen()) {
        serial_port_.Close();
        SPDLOG_INFO("关闭串口: {}", port_name_);
    }
}

bool SerialReceiver::is_open() const {
    return serial_port_.IsOpen();
}

void SerialReceiver::set_timeout(uint32_t timeout_ms) {
    // LibSerial的超时设置方式
    if (is_open()) {
        serial_port_.SetTimeout(LibSerial::Timeout::TIMEOUT_READ, timeout_ms);
    }
}

// ==================== CRC计算函数 ====================
uint16_t SerialReceiver::calculate_crc16(const uint8_t* data, size_t length) const {
    uint16_t crc = 0x0000;
    
    for (size_t i = 0; i < length; i++) {
        crc ^= (static_cast<uint16_t>(data[i]) << 8);
        
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

bool SerialReceiver::verify_crc16(const uint8_t* data, size_t length, uint16_t crc) const {
    uint16_t calculated_crc = calculate_crc16(data, length);
    return calculated_crc == crc;
}

// ==================== 字节序转换函数 ====================
uint32_t SerialReceiver::bytes_to_uint32_le(const uint8_t* bytes) const {
    return static_cast<uint32_t>(bytes[0]) |
           (static_cast<uint32_t>(bytes[1]) << 8) |
           (static_cast<uint32_t>(bytes[2]) << 16) |
           (static_cast<uint32_t>(bytes[3]) << 24);
}

uint16_t SerialReceiver::bytes_to_uint16_le(const uint8_t* bytes) const {
    return static_cast<uint16_t>(bytes[0]) |
           (static_cast<uint16_t>(bytes[1]) << 8);
}

void SerialReceiver::uint32_to_bytes_le(uint32_t value, uint8_t* bytes) const {
    bytes[0] = static_cast<uint8_t>(value & 0xFF);
    bytes[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    bytes[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    bytes[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

void SerialReceiver::uint16_to_bytes_le(uint16_t value, uint8_t* bytes) const {
    bytes[0] = static_cast<uint8_t>(value & 0xFF);
    bytes[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

// ==================== 读取报文函数 ====================
bool SerialReceiver::read_message(SerialMessage& message) {
    if (!is_open()) {
        SPDLOG_ERROR("串口未打开，无法读取数据");
        return false;
    }
    
    constexpr size_t SERIAL_MSG_SIZE = packed_sizeof<SerialMessage>();
    constexpr size_t CRC_DATA_SIZE = packed_sizeof<uint8_t>() + packed_sizeof<uint32_t>() + packed_sizeof<ImuMessage>();
    
    uint8_t buffer[SERIAL_MSG_SIZE];
    
    try {
        // 使用LibSerial读取数据
        serial_port_.Read(buffer, SERIAL_MSG_SIZE);
    } catch (const LibSerial::ReadTimeout& e) {
        SPDLOG_WARN("读取超时: {}", e.what());
        return false;
    } catch (const std::exception& e) {
        SPDLOG_ERROR("读取串口数据异常: {}", e.what());
        return false;
    }
    
    // 检查报文头
    if (buffer[0] != SERIAL_MSG_HEAD) {
        SPDLOG_WARN("报文头错误: 期望 0x{:02X}({}), 实际 0x{:02X}", 
                    SERIAL_MSG_HEAD, SERIAL_MSG_HEAD, buffer[0]);
        return false;
    }
    
    // 解析ID（小端序）
    message.id = bytes_to_uint32_le(&buffer[1]);
    
    // 解析IMU数据（直接内存拷贝）
    std::memcpy(&message.data, &buffer[5], sizeof(ImuMessage));
    
    // 解析CRC16（小端序）
    size_t crc_offset = 1 + 4 + sizeof(ImuMessage);
    message.crc16 = bytes_to_uint16_le(&buffer[crc_offset]);
    
    // 解析尾字节
    message.tail = buffer[SERIAL_MSG_SIZE - 1];
    
    // 检查报文尾
    if (message.tail != SERIAL_MSG_TAIL) {
        SPDLOG_WARN("报文尾错误: 期望 0x{:02X}({}), 实际 0x{:02X}", 
                    SERIAL_MSG_TAIL, SERIAL_MSG_TAIL, message.tail);
        return false;
    }
    
    // 验证CRC（校验范围：head + id + data）
    if (!verify_crc16(buffer, CRC_DATA_SIZE, message.crc16)) {
        SPDLOG_ERROR("CRC校验失败: ID={}, 计算值=0x{:04X}, 实际值=0x{:04X}", 
                     message.id, calculate_crc16(buffer, CRC_DATA_SIZE), message.crc16);
        return false;
    }
    
    // 设置正确的头尾字节
    message.head = SERIAL_MSG_HEAD;
    
    SPDLOG_DEBUG("成功解析报文: ID={}, CRC=0x{:04X}", message.id, message.crc16);
    return true;
}

} // namespace serial_comm
