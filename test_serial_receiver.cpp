#include "serial_comm/serial_receiver.hpp"
#include "serial_comm/message.hpp"
#include <spdlog/spdlog.h>
#include <iostream>
#include <cstring>

// 测试CRC计算
void test_crc_calculation() {
    std::cout << "=== 测试CRC计算 ===" << std::endl;
    
    // 测试数据（示例）
    uint8_t test_data[] = {0x2B, 0x01, 0x00, 0x00, 0x00};  // head=0x2B, id=1 (小端序)
    
    serial_comm::SerialReceiver receiver("/dev/ttyUSB0");
    uint16_t crc = receiver.calculate_crc16(test_data, sizeof(test_data));
    
    std::cout << "CRC计算结果: 0x" << std::hex << crc << std::dec << std::endl;
    
    // 验证CRC
    bool valid = receiver.verify_crc16(test_data, sizeof(test_data), crc);
    std::cout << "CRC验证结果: " << (valid ? "通过" : "失败") << std::endl;
}

// 测试字节序转换
void test_endian_conversion() {
    std::cout << "\n=== 测试字节序转换 ===" << std::endl;
    
    // 测试小端序：0x01000000 应该等于 1
    uint8_t bytes_le[] = {0x01, 0x00, 0x00, 0x00};
    
    serial_comm::SerialReceiver receiver("/dev/ttyUSB0");
    uint32_t value = receiver.bytes_to_uint32_le(bytes_le);
    
    std::cout << "小端序字节: ";
    for (auto b : bytes_le) {
        printf("%02X ", b);
    }
    std::cout << "-> uint32_t: " << value << std::endl;
    
    // 反向转换测试
    uint8_t output_bytes[4];
    receiver.uint32_to_bytes_le(0x12345678, output_bytes);
    std::cout << "uint32_t: 0x12345678 -> 小端序字节: ";
    for (int i = 0; i < 4; i++) {
        printf("%02X ", output_bytes[i]);
    }
    std::cout << std::endl;
}

// 模拟一个报文进行解析测试
void test_message_parsing() {
    std::cout << "\n=== 测试报文解析 ===" << std::endl;
    
    // 创建一个模拟的SerialMessage
    serial_comm::SerialMessage msg;
    msg.id = 123;
    msg.data.quaternion = {1.0, 0.0, 0.0, 0.0};  // 单位四元数
    msg.data.angular_velocity = {0.1, 0.2, 0.3};
    msg.data.linear_acceleration = {0.0, 0.0, 9.8};
    
    std::cout << "创建测试报文: ID=" << msg.id 
              << ", 四元数=(" << msg.data.quaternion.w << ", " 
              << msg.data.quaternion.x << ", " 
              << msg.data.quaternion.y << ", " 
              << msg.data.quaternion.z << ")" << std::endl;
              
    std::cout << "角速度=(" << msg.data.angular_velocity.x << ", "
              << msg.data.angular_velocity.y << ", "
              << msg.data.angular_velocity.z << ")" << std::endl;
              
    std::cout << "加速度=(" << msg.data.linear_acceleration.x << ", "
              << msg.data.linear_acceleration.y << ", "
              << msg.data.linear_acceleration.z << ")" << std::endl;
}

int main(int argc, char** argv) {
    // 设置日志级别
    spdlog::set_level(spdlog::level::debug);
    
    std::cout << "串口接收器测试程序" << std::endl;
    std::cout << "==================" << std::endl;
    
    // 运行测试
    test_crc_calculation();
    test_endian_conversion();
    test_message_parsing();
    
    // 如果提供了串口参数，尝试实际读取
    if (argc > 1) {
        std::string port_name = argv[1];
        std::cout << "\n=== 测试实际串口读取 ===" << std::endl;
        std::cout << "尝试打开串口: " << port_name << std::endl;
        
        serial_comm::SerialReceiver receiver(port_name);
        
        if (receiver.open()) {
            std::cout << "串口打开成功，开始读取数据（按Ctrl+C退出）..." << std::endl;
            
            int success_count = 0;
            int fail_count = 0;
            
            for (int i = 0; i < 10; i++) {  // 尝试读取10个报文
                serial_comm::SerialMessage msg;
                if (receiver.read_message(msg)) {
                    success_count++;
                    std::cout << "成功读取报文 " << i+1 << ": ID=" << msg.id << std::endl;
                } else {
                    fail_count++;
                    std::cout << "读取报文 " << i+1 << " 失败" << std::endl;
                }
            }
            
            std::cout << "\n统计: 成功=" << success_count 
                      << ", 失败=" << fail_count << std::endl;
        } else {
            std::cout << "串口打开失败" << std::endl;
        }
    } else {
        std::cout << "\n提示: 要测试实际串口，请运行: " << argv[0] << " /dev/ttyUSB0" << std::endl;
    }
    
    return 0;
}
