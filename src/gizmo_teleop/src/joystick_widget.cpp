#include "gizmo_teleop/joystick_widget.hpp"
#include <QPen>
#include <QBrush>
#include <QColor>

JoystickWidget::JoystickWidget(QWidget * parent)
: QWidget(parent)
{
  setMinimumSize(400, 400);
}

void JoystickWidget::paintEvent(QPaintEvent * /*event*/)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  // Draw background boundary ring
  painter.setPen(QPen(QColor(100, 100, 100), 3));
  painter.setBrush(QBrush(QColor(220, 220, 220)));
  painter.drawEllipse(center_, max_distance_, max_distance_);

  // Draw draggable puck (radius = 25px)
  painter.setPen(QPen(QColor(20, 20, 20), 2));
  if (e_stop_active_) {
    painter.setBrush(QBrush(QColor(255, 50, 50)));  // Red for E-Stop
  } else {
    painter.setBrush(QBrush(QColor(50, 150, 255))); // Blue
  }
  painter.drawEllipse(puck_pos_, 25, 25);
}

void JoystickWidget::resizeEvent(QResizeEvent * event)
{
  center_ = QPointF(width() / 2.0, height() / 2.0);
  puck_pos_ = center_;
  max_distance_ = std::min(width(), height()) / 2.0 - 30.0;
  QWidget::resizeEvent(event);
}

void JoystickWidget::mousePressEvent(QMouseEvent * event)
{
  pressed_ = true;
  updatePuck(event->position());
}

void JoystickWidget::mouseMoveEvent(QMouseEvent * event)
{
  if (pressed_) {
    updatePuck(event->position());
  }
}

void JoystickWidget::mouseReleaseEvent(QMouseEvent * event)
{
  (void)event;
  pressed_ = false;
  puck_pos_ = center_;
  update();
  emit joystickMoved(0.0, 0.0);
}

void JoystickWidget::updatePuck(const QPointF & pos)
{
  double dx = pos.x() - center_.x();
  double dy = pos.y() - center_.y();
  double distance = std::sqrt(dx * dx + dy * dy);

  if (distance > max_distance_) {
    dx = dx * max_distance_ / distance;
    dy = dy * max_distance_ / distance;
  }

  puck_pos_ = QPointF(center_.x() + dx, center_.y() + dy);
  update();

  double norm_x = dx / max_distance_;
  double norm_y = -dy / max_distance_;  // Invert Y so up is positive
  emit joystickMoved(norm_x, norm_y);
}

void JoystickWidget::setNormalizedPosition(double norm_x, double norm_y)
{
  pressed_ = false;
  double dx = norm_x * max_distance_;
  double dy = -norm_y * max_distance_;
  puck_pos_ = QPointF(center_.x() + dx, center_.y() + dy);
  update();
  emit joystickMoved(norm_x, norm_y);
}

void JoystickWidget::setEStop(bool active)
{
  if (e_stop_active_ != active) {
    e_stop_active_ = active;
    update();
  }
}
