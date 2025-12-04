#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <robot_msgs/msg/motor_control.hpp>

#include <SDL2/SDL.h>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;


struct DS4State {
    // Buttons
    bool square;
    bool triangle;
    bool circle;
    bool cross;
    bool l1;
    bool r1;
    bool l2; // digital press
    bool r2; // digital press
    bool share;
    bool options;
    bool l3;
    bool r3;
    bool ps;
    bool touchpad;

    // D-Pad
    bool dpad_up;
    bool dpad_down;
    bool dpad_left;
    bool dpad_right;

    // Axes
    int16_t left_x;
    int16_t left_y;
    int16_t right_x;
    int16_t right_y;
    int16_t l2_analog; // 0–32767
    int16_t r2_analog;
};


static void dual_shock4(SDL_GameController *controller, DS4State *_buttons) {
    SDL_GameControllerUpdate();

    _buttons->square = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_X); // DS4 square
    _buttons->cross = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_A); // DS4 cross
    _buttons->circle = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_B); // DS4 circle
    _buttons->triangle = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_Y); // DS4 triangle
    _buttons->l1 = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_LEFTSHOULDER);
    _buttons->r1 = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER);
    _buttons->share = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_BACK);
    _buttons->options = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_START);
    _buttons->l3 = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_LEFTSTICK);
    _buttons->r3 = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_RIGHTSTICK);
    _buttons->ps = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_GUIDE);
    _buttons->touchpad = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_MISC1);

    // D-Pad
    _buttons->dpad_up = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_DPAD_UP);
    _buttons->dpad_down = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_DPAD_DOWN);
    _buttons->dpad_left = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_DPAD_LEFT);
    _buttons->dpad_right = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_DPAD_RIGHT);

    // Axes
    _buttons->left_x = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTX);
    _buttons->left_y = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTY);
    _buttons->right_x = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_RIGHTX);
    _buttons->right_y = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_RIGHTY);
    _buttons->l2_analog = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_TRIGGERLEFT);
    _buttons->r2_analog = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_TRIGGERRIGHT);
}


class CommandNode final : public rclcpp::Node {
public:
    CommandNode()
        : Node("command_node"), count_(0) {
        if (SDL_Init(SDL_INIT_GAMECONTROLLER) != 0) {
            RCLCPP_ERROR(this->get_logger(), "SDL_Init failed: %s", SDL_GetError());
            return;
        }
        for (int i = 0; i < SDL_NumJoysticks(); ++i) {
            if (SDL_IsGameController(i)) {
                controller_ = SDL_GameControllerOpen(i);
                break;
            }
        }

        if (!controller_) {
            RCLCPP_ERROR(this->get_logger(), "No controller found");
            return;
        }

        // SDL_GameControllerRumble(controller_, 0xffff, 0xfff, 20);


publisher_ = this->create_publisher<robot_msgs::msg::MotorControl>("command_node", 10);
        input_thread_ = std::thread(&CommandNode::input_loop, this);
        // timer_ = this->create_wall_timer(
        // 500 ms, [this] { timer_callback(); });
    }

private:
    void timer_callback() {
        auto message = robot_msgs::msg::MotorControl();

        message.left_motor = 1;
        message.right_motor = 2;

        RCLCPP_INFO(this->get_logger(), "Publishing MotorControl: %lu", count_);
        RCLCPP_INFO(this->get_logger(),
                    "Publishing MotorControl: motor_id=%d speed=%d",
                    message.left_motor, message.right_motor);
        // SDL_GameControllerRumble(controller_, 0xFFFF, 0xFFFF, 1000); // max rumble for 1 second
        dual_shock4(controller_, &buttons);
        RCLCPP_INFO(this->get_logger(), "button: %d", this->buttons.left_x);
        publisher_->publish(message);
        count_++;
    }

    static int16_t apply_dead_zone(int16_t value, int16_t deadzone=7000) {
        if (std::abs(value) < deadzone) {
            return 0;
        }

        constexpr int16_t max_val = 32767;
        const int16_t sign = (value > 0) ? 1 : -1;
        const int16_t magnitude = std::abs(value);

        float scaled =
            static_cast<float>(magnitude - deadzone) /
            static_cast<float>(max_val - deadzone);

        return sign * static_cast<int16_t>(scaled * max_val) * -1;
    }

    void input_loop() {
        while (rclcpp::ok()) {
            rclcpp::Rate rate(5);
            auto message = robot_msgs::msg::MotorControl();
            dual_shock4(controller_, &buttons);

            const int16_t ly = apply_dead_zone(buttons.left_y, 7000);
            const int16_t ry = apply_dead_zone(buttons.right_y, 7000);

            message.left_motor  = ly / 128;     // or whatever scaling you want
            message.right_motor = ry / 128;

            publisher_->publish(message);
            if (message.left_motor == 0 && message.right_motor == 0) {
                SDL_GameControllerRumble(controller_, 0xf, 0xffff, 100);
            }
            RCLCPP_INFO(this->get_logger(), "left motor: %d", message.left_motor);
            RCLCPP_INFO(this->get_logger(), "right motor: %d", message.right_motor);


            rate.sleep();
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::thread input_thread_;
    rclcpp::Publisher<robot_msgs::msg::MotorControl>::SharedPtr publisher_;
    size_t count_;
    SDL_GameController *controller_ = nullptr;
    DS4State buttons = {};
};

int main(const int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommandNode>());
    rclcpp::shutdown();
    return 0;
}
