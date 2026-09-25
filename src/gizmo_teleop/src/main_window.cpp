#include "gizmo_teleop/main_window.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QEvent>

MainWindow::MainWindow(std::shared_ptr<JoyNode> node)
: node_(node)
{
  executor_.add_node(node_);
  setWindowTitle("ROS 2 Qt6 Joystick (C++)");
  setWindowFlags(windowFlags() | Qt::WindowStaysOnTopHint | Qt::MSWindowsFixedSizeDialogHint);
  setFocusPolicy(Qt::StrongFocus);

  auto * main_widget = new QWidget(this);
  auto * layout = new QVBoxLayout(main_widget);
  layout->setContentsMargins(10, 10, 10, 10);

  // --- Topic Row ---
  auto * topic_layout = new QHBoxLayout();
  auto * topic_label = new QLabel("Topic:", this);
  topic_input_ = new QLineEdit(QString::fromStdString(node_->topic_name_), this);
  
  auto * update_btn = new QPushButton("Update", this);
  update_btn->setFocusPolicy(Qt::NoFocus);
  connect(update_btn, &QPushButton::clicked, this, &MainWindow::onTopicUpdate);

  topic_layout->addWidget(topic_label);
  topic_layout->addWidget(topic_input_);
  topic_layout->addWidget(update_btn);

  settings_btn_ = new QPushButton("⚙️ Settings", this);
  settings_btn_->setFocusPolicy(Qt::NoFocus);
  connect(settings_btn_, &QPushButton::clicked, this, &MainWindow::toggleSettings);
  topic_layout->addWidget(settings_btn_);

  layout->addLayout(topic_layout);

  // --- Settings Panel ---
  settings_panel_ = new QFrame(this, Qt::Tool | Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
  settings_panel_->setStyleSheet("QFrame { border: 1px solid #ccc; border-radius: 4px; background-color: #f9f9f9; }");
  auto * settings_layout = new QVBoxLayout(settings_panel_);
  settings_layout->setContentsMargins(4, 4, 4, 4);

  inv_lin_cb_ = new QCheckBox("Inv Lin", settings_panel_);
  inv_ang_cb_ = new QCheckBox("Inv Ang", settings_panel_);
  twist_stamped_cb_ = new QCheckBox("TwistStamped", settings_panel_);
  twist_stamped_cb_->setChecked(node_->use_twist_stamped_);
  ontop_cb_ = new QCheckBox("Always on Top", settings_panel_);
  ontop_cb_->setChecked(true);

  for (auto * cb : {inv_lin_cb_, inv_ang_cb_, twist_stamped_cb_, ontop_cb_}) {
    connect(cb, &QCheckBox::clicked, this, &MainWindow::onSettingsInteracted);
    settings_layout->addWidget(cb);
  }
  settings_layout->addStretch();

  connect(inv_lin_cb_, &QCheckBox::clicked, this, &MainWindow::onInvLinChanged);
  connect(inv_ang_cb_, &QCheckBox::clicked, this, &MainWindow::onInvAngChanged);
  connect(twist_stamped_cb_, &QCheckBox::clicked, this, &MainWindow::onTwistStampedChanged);
  connect(ontop_cb_, &QCheckBox::clicked, this, &MainWindow::onOntopChanged);

  settings_panel_->setLayout(settings_layout);
  settings_panel_->setVisible(false);

  // --- Speed Limits Row ---
  auto * speed_layout = new QHBoxLayout();
  auto * linear_label = new QLabel("Max Linear:", this);
  linear_input_ = new QDoubleSpinBox(this);
  linear_input_->setRange(0.0, 20.0);
  linear_input_->setSingleStep(0.1);
  linear_input_->setValue(node_->max_linear_);
  linear_input_->setKeyboardTracking(false);
  linear_input_->installEventFilter(this);
  connect(linear_input_, &QDoubleSpinBox::valueChanged, this, &MainWindow::onSpeedUpdate);

  auto * angular_label = new QLabel("Max Angular:", this);
  angular_input_ = new QDoubleSpinBox(this);
  angular_input_->setRange(0.0, 20.0);
  angular_input_->setSingleStep(0.1);
  angular_input_->setValue(node_->max_angular_);
  angular_input_->setKeyboardTracking(false);
  angular_input_->installEventFilter(this);
  connect(angular_input_, &QDoubleSpinBox::valueChanged, this, &MainWindow::onSpeedUpdate);

  speed_layout->addWidget(linear_label);
  speed_layout->addWidget(linear_input_);
  speed_layout->addWidget(angular_label);
  speed_layout->addWidget(angular_input_);
  layout->addLayout(speed_layout);

  // --- Label & Joystick ---
  label_ = new QLabel("Linear X: 0.00 | Angular Z: 0.00", this);
  label_->setAlignment(Qt::AlignCenter);
  layout->addWidget(label_);

  joystick_ = new JoystickWidget(this);
  connect(joystick_, &JoystickWidget::joystickMoved, this, &MainWindow::onJoystickMoved);
  layout->addWidget(joystick_);

  auto * controls_label = new QLabel("Controls: w↑ s↓ a← d→ wd↗ ds↘ sa↙ aw↖  (space: stop)", this);
  controls_label->setStyleSheet("color: gray; font-size: 11px;");
  layout->addWidget(controls_label);

  setCentralWidget(main_widget);
  setFixedSize(sizeHint());

  // Node callbacks
  node_->gui_update_topic_cb = [this](const std::string & topic) {
    topic_input_->setText(QString::fromStdString(topic));
  };
  node_->gui_update_linear_cb = [this](double val) {
    if (std::abs(linear_input_->value() - val) > 1e-4) {
      linear_input_->blockSignals(true);
      linear_input_->setValue(val);
      linear_input_->blockSignals(false);
      onJoystickMoved(current_x_, current_y_);
    }
  };
  node_->gui_update_angular_cb = [this](double val) {
    if (std::abs(angular_input_->value() - val) > 1e-4) {
      angular_input_->blockSignals(true);
      angular_input_->setValue(val);
      angular_input_->blockSignals(false);
      onJoystickMoved(current_x_, current_y_);
    }
  };
  node_->gui_update_twist_stamped_cb = [this](bool enabled) {
    twist_stamped_cb_->blockSignals(true);
    twist_stamped_cb_->setChecked(enabled);
    twist_stamped_cb_->blockSignals(false);
  };

  // Timers
  connect(&ros_timer_, &QTimer::timeout, [this]() {
    if (!rclcpp::ok()) {
      qApp->quit();
      return;
    }
    executor_.spin_some();
  });
  ros_timer_.start(10);  // 100Hz spin

  settings_timer_.setSingleShot(true);
  connect(&settings_timer_, &QTimer::timeout, this, &MainWindow::hideSettings);

  setFocus();
}

void MainWindow::onJoystickMoved(double x, double y)
{
  if (keys_pressed_.contains(Qt::Key_Space)) {
    x = 0.0;
    y = 0.0;
    joystick_->blockSignals(true);
    joystick_->setNormalizedPosition(0.0, 0.0);
    joystick_->blockSignals(false);
  }
  current_x_ = x;
  current_y_ = y;
  node_->updateTwist(x, y);

  label_->setText(QString("Linear X: %1 | Angular Z: %2")
    .arg(node_->twist_msg_.linear.x, 0, 'f', 2)
    .arg(node_->twist_msg_.angular.z, 0, 'f', 2));
}

void MainWindow::onTopicUpdate()
{
  std::string new_topic = topic_input_->text().trimmed().toStdString();
  node_->set_parameters({rclcpp::Parameter("topic_name", new_topic)});
  setFocus();
}

bool MainWindow::eventFilter(QObject * source, QEvent * event)
{
  if (event->type() == QEvent::KeyPress) {
    auto * k = static_cast<QKeyEvent *>(event);
    if (k->key() == Qt::Key_W || k->key() == Qt::Key_A || k->key() == Qt::Key_S ||
        k->key() == Qt::Key_D || k->key() == Qt::Key_Space) {
      keyPressEvent(k);
      return true;
    }
  } else if (event->type() == QEvent::KeyRelease) {
    auto * k = static_cast<QKeyEvent *>(event);
    if (k->key() == Qt::Key_W || k->key() == Qt::Key_A || k->key() == Qt::Key_S ||
        k->key() == Qt::Key_D || k->key() == Qt::Key_Space) {
      keyReleaseEvent(k);
      return true;
    }
  }
  return QMainWindow::eventFilter(source, event);
}

void MainWindow::onSpeedUpdate()
{
  node_->set_parameters({
    rclcpp::Parameter("max_linear", linear_input_->value()),
    rclcpp::Parameter("max_angular", angular_input_->value())
  });
}

void MainWindow::toggleSettings()
{
  if (settings_panel_->isVisible()) {
    hideSettings();
  } else {
    QRect geo = geometry();
    settings_panel_->move(geo.right() + 5, geo.top());
    settings_panel_->show();
    settings_timer_.start(3000);
    setFocus();
  }
}

void MainWindow::hideSettings()
{
  if (!settings_panel_->isVisible()) return;
  settings_panel_->hide();
  settings_timer_.stop();
  setFocus();
}

void MainWindow::onSettingsInteracted()
{
  settings_timer_.start(3000);
  setFocus();
}

void MainWindow::onInvLinChanged()
{
  node_->invert_linear_ = inv_lin_cb_->isChecked();
  onJoystickMoved(current_x_, current_y_);
}

void MainWindow::onInvAngChanged()
{
  node_->invert_angular_ = inv_ang_cb_->isChecked();
  onJoystickMoved(current_x_, current_y_);
}

void MainWindow::onTwistStampedChanged()
{
  bool enabled = twist_stamped_cb_->isChecked();
  node_->set_parameters({rclcpp::Parameter("twistStamped", enabled)});
}

void MainWindow::onOntopChanged()
{
  setWindowFlag(Qt::WindowStaysOnTopHint, ontop_cb_->isChecked());
  show();

  settings_panel_->setWindowFlag(Qt::WindowStaysOnTopHint, ontop_cb_->isChecked());
  if (settings_panel_->isVisible()) {
    settings_panel_->show();
  }
  setFocus();
}



void MainWindow::keyPressEvent(QKeyEvent * event)
{
  if (event->isAutoRepeat()) return;
  int key = event->key();
  if (key == Qt::Key_W || key == Qt::Key_A || key == Qt::Key_S || key == Qt::Key_D || key == Qt::Key_Space) {
    keys_pressed_.insert(key);
    updateJoystickFromKeys();
  } else {
    QMainWindow::keyPressEvent(event);
  }
}

void MainWindow::keyReleaseEvent(QKeyEvent * event)
{
  if (event->isAutoRepeat()) return;
  int key = event->key();
  if (key == Qt::Key_W || key == Qt::Key_A || key == Qt::Key_S || key == Qt::Key_D || key == Qt::Key_Space) {
    keys_pressed_.remove(key);
    updateJoystickFromKeys();
  } else {
    QMainWindow::keyReleaseEvent(event);
  }
}

void MainWindow::updateJoystickFromKeys()
{
  double x = 0.0, y = 0.0;

  if (keys_pressed_.contains(Qt::Key_Space)) {
    joystick_->setEStop(true);
  } else {
    joystick_->setEStop(false);
    if (keys_pressed_.contains(Qt::Key_W)) y += 1.0;
    if (keys_pressed_.contains(Qt::Key_S)) y -= 1.0;
    if (keys_pressed_.contains(Qt::Key_A)) x -= 1.0;
    if (keys_pressed_.contains(Qt::Key_D)) x += 1.0;

    double mag = std::sqrt(x * x + y * y);
    if (mag > 0.0) {
      x /= mag;
      y /= mag;
    }
  }

  joystick_->setNormalizedPosition(x, y);
}

void MainWindow::mousePressEvent(QMouseEvent * event)
{
  if (settings_panel_->isVisible()) {
    hideSettings();
  }
  QMainWindow::mousePressEvent(event);
}

void MainWindow::moveEvent(QMoveEvent * event)
{
  QMainWindow::moveEvent(event);
  if (settings_panel_->isVisible()) {
    QRect geo = geometry();
    settings_panel_->move(geo.right() + 5, geo.top());
  }
}
