#ifndef ROBOT_SIMPLE_COMMANDER__FOLLOW_PANEL_HPP_
#define ROBOT_SIMPLE_COMMANDER__FOLLOW_PANEL_HPP_

#include <memory>

#include <QBasicTimer>
#include <QLabel>
#include <QPushButton>
#include <QString>
#include <QTimerEvent>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace robot_simple_commander
{

class FollowPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit FollowPanel(QWidget * parent = nullptr);
  ~FollowPanel() override;

  void onInitialize() override;

protected:
  void timerEvent(QTimerEvent * event) override;

private Q_SLOTS:
  void onStart();
  void onReturn();
  void onStop();

private:
  void callTrigger(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client);
  QPushButton * makeButton(const QString & text, const QString & style);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr return_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  QBasicTimer timer_;
  QLabel * status_{nullptr};
  QPushButton * start_btn_{nullptr};
  QPushButton * return_btn_{nullptr};
  QPushButton * stop_btn_{nullptr};
};

}  // namespace robot_simple_commander

#endif  // ROBOT_SIMPLE_COMMANDER__FOLLOW_PANEL_HPP_
