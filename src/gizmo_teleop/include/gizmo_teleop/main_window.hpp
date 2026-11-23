#ifndef GIZMO_TELEOP__MAIN_WINDOW_HPP_
#define GIZMO_TELEOP__MAIN_WINDOW_HPP_

#include <QMainWindow>
#include <QLineEdit>
#include <QPushButton>
#include <QAbstractSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QLabel>
#include <QFrame>
#include <QTimer>
#include <QSet>
#include <QMouseEvent>
#include <memory>

#include "gizmo_teleop/joy_node.hpp"
#include "gizmo_teleop/joystick_widget.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>

class TopicSpinBox : public QAbstractSpinBox
{
  Q_OBJECT

public:
  explicit TopicSpinBox(QWidget * parent = nullptr)
  : QAbstractSpinBox(parent)
  {
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  }

  QSize sizeHint() const override
  {
    QSize s = QAbstractSpinBox::sizeHint();
    if (s.width() < 240) {
      s.setWidth(240);
    }
    return s;
  }

  void stepBy(int steps) override
  {
    emit stepped(steps);
    if (window()) {
      window()->setFocus();
    }
  }

  void setMultiTopic(bool enable)
  {
    multi_topic_ = enable;
    setButtonSymbols(enable ? UpDownArrows : NoButtons);
  }

  StepEnabled stepEnabled() const override
  {
    if (!multi_topic_) {
      return StepNone;
    }
    return StepEnabled(StepUpEnabled | StepDownEnabled);
  }

  QLineEdit * lineEdit() const { return QAbstractSpinBox::lineEdit(); }
  QString text() const { return lineEdit()->text(); }
  void setText(const QString & text) { lineEdit()->setText(text); }

signals:
  void stepped(int steps);

private:
  bool multi_topic_{true};
};

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(std::shared_ptr<JoyNode> node);
  ~MainWindow() override;

protected:
  void keyPressEvent(QKeyEvent * event) override;
  void keyReleaseEvent(QKeyEvent * event) override;
  bool eventFilter(QObject * source, QEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void moveEvent(QMoveEvent * event) override;
  void changeEvent(QEvent * event) override;

private slots:
  void onJoystickMoved(double x, double y);
  void onTopicUpdate();
  void switchToNextTopic();
  void switchToPrevTopic();
  void onSpeedUpdate();
  void toggleSettings();
  void hideSettings();
  void onSettingsInteracted();
  void onInvLinChanged();
  void onInvAngChanged();
  void onTwistStampedChanged();
  void onOntopChanged();
  void onGlobalInputChanged();

private:
  void updateJoystickFromKeys();
  void initGlobalInput();
  void cleanupGlobalInput();
  void checkGlobalKeys();
  void startTopicEditing();
  void stopTopicEditing();

  std::shared_ptr<JoyNode> node_;

  TopicSpinBox * topic_input_{nullptr};
  bool topic_editing_active_{false};
  int current_topic_idx_{0};
  QPushButton * settings_btn_{nullptr};
  QFrame * settings_panel_{nullptr};
  QCheckBox * inv_lin_cb_{nullptr};
  QCheckBox * inv_ang_cb_{nullptr};
  QCheckBox * twist_stamped_cb_{nullptr};
  QCheckBox * ontop_cb_{nullptr};
  QCheckBox * global_input_cb_{nullptr};

  QDoubleSpinBox * linear_input_{nullptr};
  QDoubleSpinBox * angular_input_{nullptr};
  QPushButton * turtle_btn_{nullptr};
  QPushButton * rabbit_btn_{nullptr};

  QLabel * label_{nullptr};
  QLabel * controls_label_{nullptr};
  JoystickWidget * joystick_{nullptr};

  QTimer ros_timer_;
  QTimer settings_timer_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  QSet<int> keys_pressed_;
  double current_x_{0.0};
  double current_y_{0.0};

  // Background / Inactive OS Input Hook
  bool global_input_enabled_{true};
  void * x11_display_{nullptr};
  void * x11_handle_{nullptr};
  void * p_query_keymap_{nullptr};
  bool was_prev_pressed_{false};
  bool was_next_pressed_{false};

  unsigned char kc_w_{0};
  unsigned char kc_s_{0};
  unsigned char kc_a_{0};
  unsigned char kc_d_{0};
  unsigned char kc_space_{0};
  unsigned char kc_q_{0};
  unsigned char kc_e_{0};
  unsigned char kc_minus_{0};
  unsigned char kc_kp_subtract_{0};
  unsigned char kc_equal_{0};
  unsigned char kc_plus_{0};
  unsigned char kc_kp_add_{0};
};

#endif  // GIZMO_TELEOP__MAIN_WINDOW_HPP_
