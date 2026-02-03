#include "led_driver.h"
#include <string>
#include <map>
#include <utility>
#include <vector>

class LedDriver {
private:
    LedDriver() = default;
    LedDriver(const LedDriver&) = delete;
    LedDriver& operator=(const LedDriver&) = delete;

    static std::map<std::string, std::pair<GPIO_TypeDef*, uint16_t>> map_;

    struct BlinkState {
        uint32_t blink_count;
        uint32_t blink_interval_ms;
        uint32_t pause_duration_ms;
        uint32_t counter;
        bool is_active;
        bool current_state;
    };

    static std::map<std::string, BlinkState> blink_states_;

public:
    static LedDriver& getInstance();
    static void register_(const std::string& name, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
    static void set_(const std::string& name, bool state);
    static void toggle_(const std::string& name);
    static int led_cmd_cb_(size_t argc, const std::vector<std::string>& argv);
};
std::map<std::string, std::pair<GPIO_TypeDef*, uint16_t>> LedDriver::map_;
std::map<std::string, LedDriver::BlinkState> LedDriver::blink_states_;

LedDriver& LedDriver::getInstance() {
    static LedDriver instance;
    return instance;
}

void LedDriver::register_(const std::string& name, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    map_[name] = std::make_pair(GPIOx, GPIO_Pin);
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}
    
void LedDriver::set_(const std::string& name, bool state) {
    auto it = map_.find(name);
    if (it != map_.end()) {
        HAL_GPIO_WritePin(it->second.first, it->second.second,
            state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

void LedDriver::toggle_(const std::string& name) {
    auto it = map_.find(name);
    if (it != map_.end()) {
        HAL_GPIO_TogglePin(it->second.first, it->second.second);
    }
}

int LedDriver::led_cmd_cb_(size_t argc, const std::vector<std::string>& argv) {
    if (argc < 2) {
        printf("Error: Usage: <led_name> <0|1>\r\n");
        printf("0: turn off, 1: turn on\r\n");
        return 0x03;
    }

    const std::string& led_name = argv[0];
    int led_mode = std::stoi(argv[1]);

    printf("LED control - name: %s, mode: %d\r\n",
               led_name.c_str(), led_mode);

    if (led_mode == 0) {
        set_(led_name, false);
    } else if (led_mode == 1) {
        set_(led_name, true);
    } else {
        printf("Error: Invalid mode. Use 0 or 1\r\n");
        return 0x03;
    }

    return 0;
}

// C接口实现
extern "C" {
void register_led(const char* name, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    LedDriver::getInstance().register_(name, GPIOx, GPIO_Pin);
}

void set_led(const char* name, GPIO_PinState state) {
    LedDriver::getInstance().set_(name, state == GPIO_PIN_SET);
}

void toggle_led(const char* name) {
    LedDriver::getInstance().toggle_(name);
}
}
