#ifndef ROBOT_SIMPLE_COMMANDER__SPEED_PANEL_HPP_
#define ROBOT_SIMPLE_COMMANDER__SPEED_PANEL_HPP_

#include <memory>

#include <QBasicTimer>
#include <QLabel>
#include <QSlider>
#include <QTimerEvent>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"

namespace robot_simple_commander
{

class SpeedPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit SpeedPanel(QWidget * parent = nullptr);
  ~SpeedPanel() override;

  void onInitialize() override;

protected:
  void timerEvent(QTimerEvent * event) override;

private Q_SLOTS:
  void onSliderChanged(int ticks);
  void onSliderReleased();

private:
  void apply(double vx);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::AsyncParametersClient> controller_;
  std::shared_ptr<rclcpp::AsyncParametersClient> smoother_;
  QBasicTimer timer_;
  QLabel * value_{nullptr};
  QLabel * status_{nullptr};
  QSlider * slider_{nullptr};
};

}  // namespace robot_simple_commander

#endif  // ROBOT_SIMPLE_COMMANDER__SPEED_PANEL_HPP_
