#include "gizmo_teleop/main_window.hpp"
#include <QApplication>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QEvent>

#ifdef __linux__
#include <dlfcn.h>
typedef void * (*XOpenDisplay_fn)(const char *);
typedef int (*XCloseDisplay_fn)(void *);
typedef int (*XQueryKeymap_fn)(void *, char[32]);
typedef unsigned char (*XKeysymToKeycode_fn)(void *, unsigned long);

#ifndef XK_w
#define XK_w 0x0077
#define XK_s 0x0073
#define XK_a 0x0061
#define XK_d 0x0064
#define XK_space 0x0020
#define XK_q 0x0071
#define XK_e 0x0065
#define XK_minus 0x002d
#define XK_plus 0x002b
#define XK_equal 0x003d
#define XK_KP_Subtract 0xffad
#define XK_KP_Add 0xffab
#endif
#elif defined(_WIN32)
#include <windows.h>
#endif

MainWindow::MainWindow(std::shared_ptr<JoyNode> node)
: node_(node)
{
  executor_.add_node(node_);
  initGlobalInput();
  setWindowTitle("ROS 2 Qt6 Joystick (C++)");
  setWindowFlags(windowFlags() | Qt::WindowStaysOnTopHint | Qt::MSWindowsFixedSizeDialogHint);
  setFocusPolicy(Qt::StrongFocus);
  qApp->installEventFilter(this);

  auto * main_widget = new QWidget(this);
  auto * layout = new QVBoxLayout(main_widget);
  layout->setContentsMargins(10, 10, 10, 10);

  // --- Topic Row ---
  auto * topic_layout = new QHBoxLayout();
  auto * topic_label = new QLabel("Topic:", this);
  bool multi_topic = node_->topic_list_.size() > 1;
  topic_input_ = new TopicSpinBox(this);
  topic_input_->setMultiTopic(multi_topic);
  topic_input_->setText(QString::fromStdString(node_->topic_name_));
  if (multi_topic) {
    topic_input_->setToolTip("Robot cmd_vel topic (Use arrows, Q/E, or -/+ to switch)");
  } else {
    topic_input_->setToolTip("Robot cmd_vel topic");
  }
  connect(topic_input_, &TopicSpinBox::stepped, this, [this](int steps) {
    if (steps > 0) {
      switchToNextTopic();
    } else if (steps < 0) {
      switchToPrevTopic();
    }
  });
  connect(topic_input_->lineEdit(), &QLineEdit::returnPressed, this, &MainWindow::stopTopicEditing);
  connect(qApp, &QApplication::focusChanged, this, [this](QWidget * old, QWidget * now) {
    bool was_in_topic = (topic_input_ && (old == topic_input_ || old == topic_input_->lineEdit()));
    bool is_in_topic = (topic_input_ && (now == topic_input_ || now == topic_input_->lineEdit()));
    if (!was_in_topic && is_in_topic) {
      startTopicEditing();
    } else if (was_in_topic && !is_in_topic) {
      stopTopicEditing();
    }
  });

  auto * update_btn = new QPushButton("Update", this);
  update_btn->setFocusPolicy(Qt::NoFocus);
  connect(update_btn, &QPushButton::clicked, this, &MainWindow::onTopicUpdate);

  topic_layout->addWidget(topic_label);
  topic_layout->addWidget(topic_input_, 1);
  topic_layout->addWidget(update_btn);

  settings_btn_ = new QPushButton("⚙️ Settings", this);
  settings_btn_->setFocusPolicy(Qt::NoFocus);
  connect(settings_btn_, &QPushButton::clicked, this, &MainWindow::toggleSettings);
  topic_layout->addWidget(settings_btn_);

  layout->addLayout(topic_layout);

  // Initialize current topic index in topic_list_
  current_topic_idx_ = 0;
  for (size_t i = 0; i < node_->topic_list_.size(); ++i) {
    if (node_->topic_list_[i] == node_->topic_name_) {
      current_topic_idx_ = static_cast<int>(i);
      break;
    }
  }

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
  global_input_cb_ = new QCheckBox("Global Keys", settings_panel_);
  global_input_cb_->setChecked(true);

  for (auto * cb : {inv_lin_cb_, inv_ang_cb_, twist_stamped_cb_, ontop_cb_, global_input_cb_}) {
    connect(cb, &QCheckBox::clicked, this, &MainWindow::onSettingsInteracted);
    settings_layout->addWidget(cb);
  }
  settings_layout->addStretch();

  connect(inv_lin_cb_, &QCheckBox::clicked, this, &MainWindow::onInvLinChanged);
  connect(inv_ang_cb_, &QCheckBox::clicked, this, &MainWindow::onInvAngChanged);
  connect(twist_stamped_cb_, &QCheckBox::clicked, this, &MainWindow::onTwistStampedChanged);
  connect(ontop_cb_, &QCheckBox::clicked, this, &MainWindow::onOntopChanged);
  connect(global_input_cb_, &QCheckBox::clicked, this, &MainWindow::onGlobalInputChanged);

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

  QString controls_text = (node_->topic_list_.size() > 1)
    ? "Controls: w↑ s↓ a← d→  (space: stop, q/e or -/+: switch robot)"
    : "Controls: w↑ s↓ a← d→  (space: stop)";
  controls_label_ = new QLabel(controls_text, this);
  controls_label_->setStyleSheet("color: #666; font-size: 23px;");
  layout->addWidget(controls_label_);

  setCentralWidget(main_widget);
  QSize win_size = sizeHint();
  if (win_size.width() < 670) {
    win_size.setWidth(670);
  }
  setFixedSize(win_size);

  // Node callbacks
  node_->gui_update_topic_cb = [this](const std::string & topic) {
    topic_input_->setText(QString::fromStdString(topic));
    for (size_t i = 0; i < node_->topic_list_.size(); ++i) {
      if (node_->topic_list_[i] == topic) {
        current_topic_idx_ = static_cast<int>(i);
        break;
      }
    }
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
    if (global_input_enabled_) {
      checkGlobalKeys();
    }
  });
  ros_timer_.start(10);  // 100Hz spin

  settings_timer_.setSingleShot(true);
  connect(&settings_timer_, &QTimer::timeout, this, &MainWindow::hideSettings);

  setFocus();
}

MainWindow::~MainWindow()
{
  if (qApp) {
    qApp->removeEventFilter(this);
  }
  cleanupGlobalInput();
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
  QString raw_text = topic_input_->text().trimmed();
  if (raw_text.contains(',')) {
    QStringList parts = raw_text.split(',', Qt::SkipEmptyParts);
    std::vector<std::string> new_list;
    for (const auto & part : parts) {
      std::string t = part.trimmed().toStdString();
      if (!t.empty()) {
        new_list.push_back(t);
      }
    }
    if (!new_list.empty()) {
      node_->topic_list_ = new_list;
      for (const auto & t : new_list) {
        if (node_->use_twist_stamped_) {
          node_->getTwistStampedPublisher(t);
        } else {
          node_->getTwistPublisher(t);
        }
      }
      current_topic_idx_ = 0;
      bool multi = node_->topic_list_.size() > 1;
      topic_input_->setMultiTopic(multi);
      if (controls_label_) {
        controls_label_->setText(multi
          ? "Controls: w↑ s↓ a← d→  (space: stop, q/e or -/+: switch robot)"
          : "Controls: w↑ s↓ a← d→  (space: stop)");
      }
      std::string first_topic = new_list[0];
      topic_input_->setText(QString::fromStdString(first_topic));
      node_->set_parameters({rclcpp::Parameter("topic_name", first_topic)});
      topic_editing_active_ = false;
      if (joystick_) {
        joystick_->setEnabled(true);
      }
      setFocus();
      return;
    }
  }

  std::string new_topic = raw_text.toStdString();
  if (new_topic.empty()) {
    new_topic = node_->topic_name_;
    topic_input_->setText(QString::fromStdString(new_topic));
  } else {
    if (node_->use_twist_stamped_) {
      node_->getTwistStampedPublisher(new_topic);
    } else {
      node_->getTwistPublisher(new_topic);
    }
  }
  for (size_t i = 0; i < node_->topic_list_.size(); ++i) {
    if (node_->topic_list_[i] == new_topic) {
      current_topic_idx_ = static_cast<int>(i);
      break;
    }
  }
  node_->set_parameters({rclcpp::Parameter("topic_name", new_topic)});
  topic_editing_active_ = false;
  if (joystick_) {
    joystick_->setEnabled(true);
  }
  setFocus();
}

void MainWindow::startTopicEditing()
{
  if (topic_editing_active_) return;
  topic_editing_active_ = true;
  if (joystick_) {
    joystick_->setEnabled(false);
    joystick_->setNormalizedPosition(0.0, 0.0);
  }
  node_->updateTwist(0.0, 0.0);
  node_->publishTwist();
  keys_pressed_.clear();
}

void MainWindow::stopTopicEditing()
{
  if (!topic_editing_active_) return;
  topic_editing_active_ = false;
  onTopicUpdate();
  if (topic_input_) {
    topic_input_->clearFocus();
    if (topic_input_->lineEdit()) {
      topic_input_->lineEdit()->clearFocus();
    }
  }
  if (joystick_) {
    joystick_->setEnabled(true);
  }
  setFocus();
}

void MainWindow::switchToNextTopic()
{
  if (node_->topic_list_.size() <= 1) return;
  current_topic_idx_ = (current_topic_idx_ + 1) % static_cast<int>(node_->topic_list_.size());
  std::string new_topic = node_->topic_list_[current_topic_idx_];
  topic_input_->setText(QString::fromStdString(new_topic));
  node_->set_parameters({rclcpp::Parameter("topic_name", new_topic)});
}

void MainWindow::switchToPrevTopic()
{
  if (node_->topic_list_.size() <= 1) return;
  int n = static_cast<int>(node_->topic_list_.size());
  current_topic_idx_ = (current_topic_idx_ - 1 + n) % n;
  std::string new_topic = node_->topic_list_[current_topic_idx_];
  topic_input_->setText(QString::fromStdString(new_topic));
  node_->set_parameters({rclcpp::Parameter("topic_name", new_topic)});
}

bool MainWindow::eventFilter(QObject * source, QEvent * event)
{
  if (event->type() == QEvent::MouseButtonPress) {
    if (topic_editing_active_ && topic_input_) {
      QWidget * clicked = qobject_cast<QWidget *>(source);
      if (clicked && clicked != topic_input_ && !topic_input_->isAncestorOf(clicked)) {
        stopTopicEditing();
        setFocus();
      }
    }
  }

  if (topic_editing_active_) {
    if (event->type() == QEvent::KeyPress || event->type() == QEvent::KeyRelease) {
      auto * k = static_cast<QKeyEvent *>(event);
      if (k->key() == Qt::Key_Return || k->key() == Qt::Key_Enter) {
        stopTopicEditing();
        setFocus();
        return true;
      }
      return false;
    }
  }

  if (event->type() == QEvent::KeyPress) {
    auto * k = static_cast<QKeyEvent *>(event);
    if (!k->isAutoRepeat()) {
      int key = k->key();
      if (key == Qt::Key_W || key == Qt::Key_A || key == Qt::Key_S ||
          key == Qt::Key_D || key == Qt::Key_Space ||
          key == Qt::Key_Q || key == Qt::Key_E ||
          key == Qt::Key_Minus || key == Qt::Key_Plus || key == Qt::Key_Equal) {
        keyPressEvent(k);
        return true;
      }
    }
  } else if (event->type() == QEvent::KeyRelease) {
    auto * k = static_cast<QKeyEvent *>(event);
    if (!k->isAutoRepeat()) {
      int key = k->key();
      if (key == Qt::Key_W || key == Qt::Key_A || key == Qt::Key_S ||
          key == Qt::Key_D || key == Qt::Key_Space) {
        keyReleaseEvent(k);
        return true;
      }
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
  if (!global_input_enabled_) {
    int key = event->key();
    if (key == Qt::Key_W || key == Qt::Key_A || key == Qt::Key_S || key == Qt::Key_D || key == Qt::Key_Space) {
      keys_pressed_.insert(key);
      updateJoystickFromKeys();
    } else if (key == Qt::Key_Q || key == Qt::Key_Minus) {
      switchToPrevTopic();
    } else if (key == Qt::Key_E || key == Qt::Key_Plus || key == Qt::Key_Equal) {
      switchToNextTopic();
    }
  }
  QMainWindow::keyPressEvent(event);
}

void MainWindow::keyReleaseEvent(QKeyEvent * event)
{
  if (event->isAutoRepeat()) return;
  if (!global_input_enabled_) {
    int key = event->key();
    if (key == Qt::Key_W || key == Qt::Key_A || key == Qt::Key_S || key == Qt::Key_D || key == Qt::Key_Space) {
      keys_pressed_.remove(key);
      updateJoystickFromKeys();
    }
  }
  QMainWindow::keyReleaseEvent(event);
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

void MainWindow::changeEvent(QEvent * event)
{
  if (event->type() == QEvent::ActivationChange) {
    if (global_input_enabled_) {
      checkGlobalKeys();
    }
  }
  QMainWindow::changeEvent(event);
}

void MainWindow::onGlobalInputChanged()
{
  global_input_enabled_ = global_input_cb_->isChecked();
  if (!global_input_enabled_) {
    if (!keys_pressed_.isEmpty()) {
      keys_pressed_.clear();
      updateJoystickFromKeys();
    }
  }
}

void MainWindow::initGlobalInput()
{
#ifdef __linux__
  x11_handle_ = dlopen("libX11.so.6", RTLD_LAZY);
  if (!x11_handle_) {
    x11_handle_ = dlopen("libX11.so", RTLD_LAZY);
  }
  if (!x11_handle_) {
    return;
  }

  auto pXOpenDisplay = reinterpret_cast<XOpenDisplay_fn>(dlsym(x11_handle_, "XOpenDisplay"));
  auto pXKeysymToKeycode = reinterpret_cast<XKeysymToKeycode_fn>(dlsym(x11_handle_, "XKeysymToKeycode"));
  p_query_keymap_ = dlsym(x11_handle_, "XQueryKeymap");

  if (!pXOpenDisplay || !pXKeysymToKeycode || !p_query_keymap_) {
    dlclose(x11_handle_);
    x11_handle_ = nullptr;
    p_query_keymap_ = nullptr;
    return;
  }

  x11_display_ = pXOpenDisplay(nullptr);
  if (!x11_display_) {
    dlclose(x11_handle_);
    x11_handle_ = nullptr;
    p_query_keymap_ = nullptr;
    return;
  }

  kc_w_ = pXKeysymToKeycode(x11_display_, XK_w);
  kc_s_ = pXKeysymToKeycode(x11_display_, XK_s);
  kc_a_ = pXKeysymToKeycode(x11_display_, XK_a);
  kc_d_ = pXKeysymToKeycode(x11_display_, XK_d);
  kc_space_ = pXKeysymToKeycode(x11_display_, XK_space);
  kc_q_ = pXKeysymToKeycode(x11_display_, XK_q);
  kc_e_ = pXKeysymToKeycode(x11_display_, XK_e);
  kc_minus_ = pXKeysymToKeycode(x11_display_, XK_minus);
  kc_kp_subtract_ = pXKeysymToKeycode(x11_display_, XK_KP_Subtract);
  kc_equal_ = pXKeysymToKeycode(x11_display_, XK_equal);
  kc_plus_ = pXKeysymToKeycode(x11_display_, XK_plus);
  kc_kp_add_ = pXKeysymToKeycode(x11_display_, XK_KP_Add);
#endif
}

void MainWindow::cleanupGlobalInput()
{
#ifdef __linux__
  if (x11_handle_) {
    if (x11_display_) {
      auto pXCloseDisplay = reinterpret_cast<XCloseDisplay_fn>(dlsym(x11_handle_, "XCloseDisplay"));
      if (pXCloseDisplay) {
        pXCloseDisplay(x11_display_);
      }
      x11_display_ = nullptr;
    }
    dlclose(x11_handle_);
    x11_handle_ = nullptr;
    p_query_keymap_ = nullptr;
  }
#endif
}

void MainWindow::checkGlobalKeys()
{
  if (topic_editing_active_) {
    return;
  }
  if (topic_input_ && (topic_input_->hasFocus() || (topic_input_->lineEdit() && topic_input_->lineEdit()->hasFocus()))) {
    return;
  }
  if (joystick_ && !joystick_->isEnabled()) {
    return;
  }
#ifdef __linux__
  if (!x11_handle_ || !x11_display_ || !p_query_keymap_) {
    return;
  }

  char keys[32];
  reinterpret_cast<XQueryKeymap_fn>(p_query_keymap_)(x11_display_, keys);

  auto is_down = [&](unsigned char kc) -> bool {
    if (kc == 0) return false;
    return (keys[kc / 8] & (1 << (kc % 8))) != 0;
  };

  bool changed = false;
  auto sync_key = [&](int qt_key, bool down) {
    if (down) {
      if (!keys_pressed_.contains(qt_key)) {
        keys_pressed_.insert(qt_key);
        changed = true;
      }
    } else {
      if (keys_pressed_.contains(qt_key)) {
        keys_pressed_.remove(qt_key);
        changed = true;
      }
    }
  };

  sync_key(Qt::Key_W, is_down(kc_w_));
  sync_key(Qt::Key_S, is_down(kc_s_));
  sync_key(Qt::Key_A, is_down(kc_a_));
  sync_key(Qt::Key_D, is_down(kc_d_));
  sync_key(Qt::Key_Space, is_down(kc_space_));

  if (changed) {
    updateJoystickFromKeys();
  }

  // Previous topic: Q, main keyboard minus, or numpad minus
  bool prev_down = is_down(kc_q_) || is_down(kc_minus_) || is_down(kc_kp_subtract_);
  if (prev_down && !was_prev_pressed_) {
    switchToPrevTopic();
  }
  was_prev_pressed_ = prev_down;

  // Next topic: E, main keyboard equal/plus, or numpad plus
  bool next_down = is_down(kc_e_) || is_down(kc_equal_) || is_down(kc_plus_) || is_down(kc_kp_add_);
  if (next_down && !was_next_pressed_) {
    switchToNextTopic();
  }
  was_next_pressed_ = next_down;

#elif defined(_WIN32)
  auto is_down = [](int vkey) -> bool {
    return (GetAsyncKeyState(vkey) & 0x8000) != 0;
  };

  bool changed = false;
  auto sync_key = [&](int qt_key, bool down) {
    if (down) {
      if (!keys_pressed_.contains(qt_key)) {
        keys_pressed_.insert(qt_key);
        changed = true;
      }
    } else {
      if (keys_pressed_.contains(qt_key)) {
        keys_pressed_.remove(qt_key);
        changed = true;
      }
    }
  };

  sync_key(Qt::Key_W, is_down('W'));
  sync_key(Qt::Key_S, is_down('S'));
  sync_key(Qt::Key_A, is_down('A'));
  sync_key(Qt::Key_D, is_down('D'));
  sync_key(Qt::Key_Space, is_down(VK_SPACE));

  if (changed) {
    updateJoystickFromKeys();
  }

  bool prev_down = is_down('Q') || is_down(VK_OEM_MINUS) || is_down(VK_SUBTRACT);
  if (prev_down && !was_prev_pressed_) {
    switchToPrevTopic();
  }
  was_prev_pressed_ = prev_down;

  bool next_down = is_down('E') || is_down(VK_OEM_PLUS) || is_down(VK_ADD);
  if (next_down && !was_next_pressed_) {
    switchToNextTopic();
  }
  was_next_pressed_ = next_down;
#endif
}
