#include "robot_simple_commander/follow_panel.hpp"

#include <QVBoxLayout>

#include <chrono>
#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace robot_simple_commander
{

namespace
{
const char * kStartStyle =
  "QPushButton { background-color: #2f9e44; color: white; font-size: 16px;"
  " font-weight: 600; padding: 8px; border: none; border-radius: 8px; }"
  "QPushButton:hover { background-color: #37b24d; }"
  "QPushButton:pressed { background-color: #2b8a3e; }";
const char * kReturnStyle =
  "QPushButton { background-color: #1c7ed6; color: white; font-size: 16px;"
  " font-weight: 600; padding: 8px; border: none; border-radius: 8px; }"
  "QPushButton:hover { background-color: #228be6; }"
  "QPushButton:pressed { background-color: #1864ab; }";
const char * kStopStyle =
  "QPushButton { background-color: #e03131; color: white; font-size: 16px;"
  " font-weight: 600; padding: 8px; border: none; border-radius: 8px; }"
  "QPushButton:hover { background-color: #f03e3e; }"
  "QPushButton:pressed { background-color: #c92a2a; }";
}  // namespace

FollowPanel::FollowPanel(QWidget * parent)
: Panel(parent)
{
  auto * title = new QLabel(QString::fromUtf8("轨迹跟随"));
  title->setStyleSheet(
    "QLabel { font-size: 18px; font-weight: 600; padding-bottom: 2px; }");
  auto * hint = new QLabel(QString::fromUtf8(
    "绿点是起点，橙点是终点。\n"
    "跟完点「回到起点」，车沿原路开回去。"));
  hint->setWordWrap(true);
  hint->setStyleSheet("QLabel { color: #adb5bd; font-size: 12px; }");

  status_ = new QLabel(QString::fromUtf8("正在等跟随节点…"));
  status_->setWordWrap(true);
  status_->setStyleSheet(
    "QLabel { font-size: 14px; padding: 8px; background: #212529;"
    " border-radius: 8px; }");

  start_btn_ = makeButton(QString::fromUtf8("开始"), kStartStyle);
  return_btn_ = makeButton(QString::fromUtf8("回到起点"), kReturnStyle);
  stop_btn_ = makeButton(QString::fromUtf8("停止"), kStopStyle);
  start_btn_->setToolTip(QString::fromUtf8("从轨迹起点开始跟随"));
  return_btn_->setToolTip(
    QString::fromUtf8("沿录制路线开回起点，并转到录制时的朝向"));
  stop_btn_->setToolTip(QString::fromUtf8("立刻停下，程序不退出"));

  auto * layout = new QVBoxLayout;
  layout->setContentsMargins(12, 12, 12, 12);
  layout->setSpacing(8);
  layout->addWidget(title);
  layout->addWidget(hint);
  layout->addWidget(status_);
  layout->addSpacing(4);
  layout->addWidget(start_btn_);
  layout->addWidget(return_btn_);
  layout->addWidget(stop_btn_);
  layout->addStretch();
  setLayout(layout);

  connect(start_btn_, &QPushButton::clicked, this, &FollowPanel::onStart);
  connect(return_btn_, &QPushButton::clicked, this, &FollowPanel::onReturn);
  connect(stop_btn_, &QPushButton::clicked, this, &FollowPanel::onStop);
}

QPushButton * FollowPanel::makeButton(const QString & text, const QString & style)
{
  auto * button = new QPushButton(text);
  button->setMinimumHeight(44);
  button->setStyleSheet(style);
  button->setCursor(Qt::PointingHandCursor);
  return button;
}

FollowPanel::~FollowPanel()
{
  timer_.stop();
}

void FollowPanel::onInitialize()
{
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--remap", "__node:=follow_rviz_panel"});
  node_ = std::make_shared<rclcpp::Node>("_", options);

  start_client_ = node_->create_client<std_srvs::srv::Trigger>("/follow_start");
  return_client_ = node_->create_client<std_srvs::srv::Trigger>("/follow_return");
  stop_client_ = node_->create_client<std_srvs::srv::Trigger>("/follow_stop");
  status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/follow_ui_status",
    rclcpp::QoS(1).transient_local(),
    [this](std_msgs::msg::String::ConstSharedPtr msg) {
      status_->setText(QString::fromStdString(msg->data));
    });

  timer_.start(100, this);
}

void FollowPanel::timerEvent(QTimerEvent * /*event*/)
{
  if (node_) {
    rclcpp::spin_some(node_);
  }
}

void FollowPanel::callTrigger(
  const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client)
{
  if (!client->wait_for_service(std::chrono::milliseconds(300))) {
    status_->setText(QString::fromUtf8("跟随节点还没起来"));
    return;
  }
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  client->async_send_request(
    req,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      const auto resp = future.get();
      if (!resp->success) {
        status_->setText(QString::fromStdString(resp->message));
      }
    });
}

void FollowPanel::onStart()
{
  callTrigger(start_client_);
}

void FollowPanel::onReturn()
{
  callTrigger(return_client_);
}

void FollowPanel::onStop()
{
  callTrigger(stop_client_);
}

}  // namespace robot_simple_commander

PLUGINLIB_EXPORT_CLASS(
  robot_simple_commander::FollowPanel, rviz_common::Panel)
