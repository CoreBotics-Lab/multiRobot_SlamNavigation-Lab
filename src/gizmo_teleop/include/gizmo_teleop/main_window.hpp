#ifndef GIZMO_TELEOP__MAIN_WINDOW_HPP_
#define GIZMO_TELEOP__MAIN_WINDOW_HPP_

#include <QMainWindow>
#include <QLineEdit>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QLabel>
#include <QFrame>
#include <QTimer>
#include <QSet>
#include <memory>

#include "gizmo_teleop/joy_node.hpp"
#include "gizmo_teleop/joystick_widget.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(std::shared_ptr<JoyNode> node);

protected:
  void keyPressEvent(QKeyEvent * event) override;
  void keyReleaseEvent(QKeyEvent * event) override;
  bool eventFilter(QObject * source, QEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void moveEvent(QMoveEvent * event) override;

private slots:
  void onJoystickMoved(double x, double y);
  void onTopicUpdate();
  void onSpeedUpdate();
  void toggleSettings();
  void hideSettings();
  void onSettingsInteracted();
  void onInvLinChanged();
  void onInvAngChanged();
  void onTwistStampedChanged();
  void onOntopChanged();

private:
  void updateJoystickFromKeys();

  std::shared_ptr<JoyNode> node_;

  QLineEdit * topic_input_{nullptr};
  QPushButton * settings_btn_{nullptr};
  QFrame * settings_panel_{nullptr};
  QCheckBox * inv_lin_cb_{nullptr};
  QCheckBox * inv_ang_cb_{nullptr};
  QCheckBox * twist_stamped_cb_{nullptr};
  QCheckBox * ontop_cb_{nullptr};

  QDoubleSpinBox * linear_input_{nullptr};
  QDoubleSpinBox * angular_input_{nullptr};
  QPushButton * turtle_btn_{nullptr};
  QPushButton * rabbit_btn_{nullptr};

  QLabel * label_{nullptr};
  JoystickWidget * joystick_{nullptr};

  QTimer ros_timer_;
  QTimer settings_timer_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  QSet<int> keys_pressed_;
  double current_x_{0.0};
  double current_y_{0.0};
};

#endif  // GIZMO_TELEOP__MAIN_WINDOW_HPP_
