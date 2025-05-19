#include <iostream>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mujoco/mujoco.h>

class RRRRobot : public rclcpp::Node
{
public:
  RRRRobot() : Node("rrr_robot")
  {
    // Get the model path
    std::string pkg_path = ament_index_cpp::get_package_share_directory("rrr_challenge");
    std::string model_path = pkg_path + "/include/rrr_challenge/mujoco_files/model/model.xml";
    
    // Load model
    char error[1000] = "Could not load model";
    this->model = mj_loadXML(model_path.c_str(), NULL, error, 1000);
    if (!this->model) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load model: %s", error);
      return;
    }
    
    // Create data
    this->data = mj_makeData(this->model);
    RCLCPP_INFO(this->get_logger(), "Model loaded successfully");
    
    // Create a timer to step the simulation
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&RRRRobot::step_simulation, this));
  }

  ~RRRRobot() 
  {
    if (this->data) mj_deleteData(this->data);
    if (this->model) mj_deleteModel(this->model);
  }

private:
  void step_simulation()
  {
    // Step the simulation
    mj_step(this->model, this->data);
    
    // Print joint positions
    RCLCPP_INFO(this->get_logger(), "Joint positions: %f, %f, %f",
                this->data->qpos[0], this->data->qpos[1], this->data->qpos[2]);
  }

  mjModel* model = nullptr;
  mjData* data = nullptr;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RRRRobot>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}