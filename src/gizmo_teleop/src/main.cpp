#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "gizmo_teleop/joy_node.hpp"
#include "gizmo_teleop/main_window.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int exit_code = 0;
  {
    QApplication app(argc, argv);

    auto node = std::make_shared<JoyNode>();
    MainWindow window(node);
    window.show();

    exit_code = app.exec();
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return exit_code;
}
