#ifndef GIZMO_TELEOP__JOYSTICK_WIDGET_HPP_
#define GIZMO_TELEOP__JOYSTICK_WIDGET_HPP_

#include <QWidget>
#include <QPointF>
#include <QPainter>
#include <QMouseEvent>
#include <cmath>

class JoystickWidget : public QWidget
{
  Q_OBJECT

public:
  explicit JoystickWidget(QWidget * parent = nullptr);
  void setNormalizedPosition(double norm_x, double norm_y);
  void setEStop(bool active);

signals:
  void joystickMoved(double norm_x, double norm_y);

protected:
  void paintEvent(QPaintEvent * event) override;
  void resizeEvent(QResizeEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void mouseReleaseEvent(QMouseEvent * event) override;

private:
  void updatePuck(const QPointF & pos);

  double max_distance_{100.0};
  QPointF puck_pos_{125.0, 125.0};
  QPointF center_{125.0, 125.0};
  bool pressed_{false};
  bool e_stop_active_{false};
};

#endif  // GIZMO_TELEOP__JOYSTICK_WIDGET_HPP_
