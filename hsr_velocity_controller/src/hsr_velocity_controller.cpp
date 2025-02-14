#include <vector>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "controller_interface/controller_interface.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "urdf/model.h"
#include <pluginlib/class_loader.hpp>
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace hsr_velocity_controller_ns
{
    class HsrVelocityController : public controller_interface::ControllerInterface
    {
    public:
        HsrVelocityController() : controller_interface::ControllerInterface() {}

        controller_interface::CallbackReturn on_init() override
        {
            try
            {
                node_ = std::make_shared<rclcpp::Node>("hsr_velocity_controller");
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("HsrVelocityController"), "Exception: %s", e.what());
                return controller_interface::CallbackReturn::ERROR;
            }

            return controller_interface::CallbackReturn::SUCCESS;
        }

        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            node_->declare_parameter("joints", std::vector<std::string>());
            joint_names_ = node_->get_parameter("joints").as_string_array();

            if (joint_names_.empty())
            {
                RCLCPP_ERROR(node_->get_logger(), "No joints specified in parameter file!");
                return controller_interface::CallbackReturn::ERROR;
            }

            n_joints_ = joint_names_.size();
            commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));

            // Initialize vectors
            old_integrator_.resize(n_joints_, 0.0);
            old_error_.resize(n_joints_, 0.0);
            filtered_vel_.resize(n_joints_, 0.0);
            p_gains_.resize(n_joints_, 0.0);
            i_gains_.resize(n_joints_, 0.0);
            d_gains_.resize(n_joints_, 0.0);
            ff_gains_.resize(n_joints_, 0.0);

            // Create subscriber
            sub_command_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
                "command", 1, std::bind(&HsrVelocityController::commandCB, this, std::placeholders::_1));

            // Create real-time publisher
            pub_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
                node_->create_publisher<std_msgs::msg::Float64MultiArray>("controller_state", 1));

            RCLCPP_INFO(node_->get_logger(), "HsrVelocityController Configured.");
            return controller_interface::CallbackReturn::SUCCESS;
        }

        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            for (size_t i = 0; i < n_joints_; i++)
            {
                joint_handles_.push_back(hardware_interface::CommandInterface(
                    joint_names_[i], hardware_interface::HW_IF_POSITION));
            }
            return controller_interface::CallbackReturn::SUCCESS;
        }

        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & /*previous_state*/) override
        {
            return controller_interface::CallbackReturn::SUCCESS;
        }

        controller_interface::return_type update(
            const rclcpp::Time & /*time*/, const rclcpp::Duration &period) override
        {
            std::vector<double> &commands = *commands_buffer_.readFromRT();
            double dt = period.seconds();

            for (size_t i = 0; i < n_joints_; i++)
            {
                double vel_cmd = commands[i];
                double current_pos;
                
                // Correct way to get joint position
                if (!joint_handles_[i].get_value(current_pos))
                {
                    RCLCPP_ERROR(node_->get_logger(), "Failed to get value for joint %s", joint_names_[i].c_str());
                    continue;
                }

                double error = vel_cmd - filtered_vel_[i];
                double new_integrator = old_integrator_[i] + error * dt;

                double next_pos = current_pos +
                                  vel_cmd * dt * ff_gains_[i] +
                                  error * p_gains_[i] +
                                  new_integrator * i_gains_[i] +
                                  d_gains_[i] / dt * (error - old_error_[i]);

                // Correct way to set joint position
                if (!joint_handles_[i].set_value(next_pos))
                {
                    RCLCPP_ERROR(node_->get_logger(), "Failed to set value for joint %s", joint_names_[i].c_str());
                }

                old_error_[i] = error;
                old_integrator_[i] = new_integrator;
            }

            if (counter % 10 == 0)
            {
                if (pub_ && pub_->trylock())
                {
                    pub_->msg_.data = filtered_vel_;
                    pub_->unlockAndPublish();
                }
            }

            counter++;
            return controller_interface::return_type::OK;
        }

        controller_interface::InterfaceConfiguration command_interface_configuration() const override
        {
            controller_interface::InterfaceConfiguration config;
            config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
            for (const auto &joint_name : joint_names_)
            {
                config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
            }
            return config;
        }

        controller_interface::InterfaceConfiguration state_interface_configuration() const override
        {
            controller_interface::InterfaceConfiguration config;
            config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
            for (const auto &joint_name : joint_names_)
            {
                config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
            }
            return config;
        }

    private:
        void commandCB(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
            if (msg->data.size() != n_joints_)
            {
                RCLCPP_ERROR(node_->get_logger(), "Command size mismatch: %zu (expected %u)",
                             msg->data.size(), n_joints_);
                return;
            }
            commands_buffer_.writeFromNonRT(msg->data);
        }

        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_command_;
        std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> pub_;

        std::vector<std::string> joint_names_;
        std::vector<hardware_interface::CommandInterface> joint_handles_;
        unsigned int n_joints_;
        unsigned int counter = 0;
        std::vector<double> old_integrator_;
        std::vector<double> old_error_;
        std::vector<double> filtered_vel_;
        std::vector<double> p_gains_;
        std::vector<double> i_gains_;
        std::vector<double> d_gains_;
        std::vector<double> ff_gains_;

        realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer_;
    };

} // namespace hsr_velocity_controller_ns

PLUGINLIB_EXPORT_CLASS(hsr_velocity_controller_ns::HsrVelocityController, controller_interface::ControllerInterface)
