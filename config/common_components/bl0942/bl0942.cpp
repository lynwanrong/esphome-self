#include "bl0942.h"
#include "esphome/core/log.h"

namespace esphome {
namespace bl0942 {

static const char *TAG = "bl0942";
static const int C_MAX = 20; // 这里的定义保留您的原始设定

void BL0942::setup() {
  ESP_LOGCONFIG(TAG, "Setting up BL0942...");
  // 可以在这里做一些初始化检查，但 UART 已经在父类初始化了
}

void BL0942::update() {
  // 对应您原代码中的 bl0942_timer_callback
  // 发送读取指令 0x58, 0xAA
  uint8_t cmd[2] = {0x58, 0xAA};
  this->write_array(cmd, 2);
}

void BL0942::loop() {
  // 对应您原代码中的 while(true) 接收逻辑
  // ESPHome 的 loop 是非阻塞的，所以我们不能在这里用死循环
  // 而是检查是否有数据到达

  // 这里的逻辑：寻找包头 0x55
  while (this->available() > 0) {
    uint8_t byte;
    this->peek_byte(&byte);

    if (rx_buffer_.empty()) {
      if (byte == 0x55) {
        // 找到包头，放入缓冲区
        this->read_byte(&byte);
        rx_buffer_.push_back(byte);
      } else {
        // 不是包头，丢弃
        this->read_byte(&byte);
      }
    } else {
      // 已经有包头了，继续读取后续数据
      this->read_byte(&byte);
      rx_buffer_.push_back(byte);

      // 检查长度是否足够 (23字节)
      if (rx_buffer_.size() >= 23) {
        // 解析数据
        this->parse_data_(rx_buffer_.data(), 23);
        // 清空缓冲区，准备下一次读取
        rx_buffer_.clear();
      }
    }
  }
}

// 您的核心解析逻辑，完全保留
void BL0942::parse_data_(uint8_t *data, int len) {
  // 再次检查包头和长度 (双重保险)
  if (data[0] != 0x55 || len != 23) {
    ESP_LOGE(TAG, "Invalid frame or length");
    return;
  }

  uint8_t check_sum = 88; // 您的初始校验值
  
  // 校验和计算
  for (int i = 0; i < len - 1; i++) {
    check_sum += data[i];
  }
  check_sum = ~check_sum;

  if (check_sum != data[len - 1]) {
    ESP_LOGE(TAG, "Checksum error: cal=%02X, recv=%02X", check_sum, data[len - 1]);
    return;
  }

  // --- 以下是您的原始计算公式 ---

  // 计算电流
  int32_t c_reg = (data[3] << 16) | (data[2] << 8) | data[1];
  c_reg = (data[3] & 0x80) ? -(16777216 - c_reg) : c_reg;
  double c_value = (C_MAX == 10) ? c_reg * 1.218 / (305978 * 3) : c_reg * 1.218 / 305978;

  // 计算电压
  uint32_t v_reg = (data[6] << 16) | (data[5] << 8) | data[4];
  double v_value = v_reg * 1.218 * 1950.51 / 37734390;

  // 读取功率
  int32_t p_reg = (data[12] << 16) | (data[11] << 8) | data[10];
  p_reg = (data[12] & 0x80) ? -(16777216 - p_reg) : p_reg;
  double p_reg_value = (C_MAX == 10) ? p_reg * 1.218 * 1.218 * 1950.51 / 511610 : p_reg * 1.218 * 1.218 * 1950.51 / 1803870;

  // 计算功率因数
  double p_cal_value = c_value * v_value;
  double p_factor = p_cal_value ? p_reg_value / p_cal_value : 0;

  // --- 更新传感器 ---
  if (this->voltage_sensor_ != nullptr) 
    this->voltage_sensor_->publish_state(v_value);
    
  if (this->current_sensor_ != nullptr) 
    this->current_sensor_->publish_state(c_value);
    
  if (this->power_sensor_ != nullptr) 
    this->power_sensor_->publish_state(p_reg_value);

  if (this->power_factor_sensor_ != nullptr) 
    this->power_factor_sensor_->publish_state(p_factor);
}

}  // namespace bl0942
}  // namespace esphome