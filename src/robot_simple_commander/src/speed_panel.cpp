#include "robot_simple_commander/speed_panel.hpp"

#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace robot_simple_commander
{

namespace
{
constexpr int kMinTick = 4;    // 0.20 m/s
constexpr int kMaxTick = 20;   // 1.00 m/s
constexpr double kTurnMax = 1.20;

double tickToSpeed(int ticks)
{
  return static_cast<double>(ticks) / 20.0;
}
}  // namespace

SpeedPanel::SpeedPanel(QWidget * parent)
: Panel(parent)
{
  value_ = new QLabel(QString::fromUtf8("1.00 m/s"));
  status_ = new QLabel(QString::fromUtf8("松手后生效。只改跟随速度，建图仍用手柄。"));
  status_->setWordWrap(true);
  slider_ = new QSlider(Qt::Horizontal);
  slider_->setRange(kMinTick, kMaxTick);
  slider_->setValue(kMaxTick);
  slider_->setTickInterval(2);
  slider_->setTickPosition(QSlider::TicksBelow);
  slider_->setToolTip(QString::fromUtf8("前进速度上限，0.20–1.00 m/s"));

  auto * layout = new QVBoxLayout;
  layout->setContentsMargins(12, 12, 12, 12);
  layout->addWidget(new QLabel(QString::fromUtf8("前进速度上限")));
  layout->addWidget(value_);
  layout->addWidget(slider_);
  layout->addWidget(status_);
  layout->addStretch();
  setLayout(layout);

  connect(slider_, &QSlider::valueChanged, this, &SpeedPanel::onSliderChanged);
  connect(slider_, &QSlider::sliderReleased, this, &SpeedPanel::onSliderReleased);
}

SpeedPanel::~SpeedPanel()
{
  timer_.stop();
}

void SpeedPanel::onInitialize()
{
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--remap", "__node:=speed_rviz_panel"});
  node_ = std::make_shared<rclcpp::Node>("_", options);
  controller_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "controller_server");
  smoother_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "velocity_smoother");
  timer_.start(100, this);
}

void SpeedPanel::timerEvent(QTimerEvent * /*event*/)
{
  if (node_) {
    rclcpp::spin_some(node_);
  }
}

void SpeedPanel::onSliderChanged(int ticks)
{
  value_->setText(QString::number(tickToSpeed(ticks), 'f', 2) + tr(" m/s"));
}

void SpeedPanel::onSliderReleased()
{
  apply(tickToSpeed(slider_->value()));
}

void SpeedPanel::apply(double vx)
{
  if (!controller_ || !controller_->service_is_ready()) {
    status_->setText(QString::fromUtf8(
      "导航还没起来。建图用手柄；跟随起来后再拖一次。"));
    return;
  }

  controller_->set_parameters(
    {rclcpp::Parameter("FollowPath.vx_max", vx)},
    [this, vx](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
      const auto results = future.get();
      const bool ok = !results.empty() && results.front().successful;
      if (!ok) {
        status_->setText(QString::fromUtf8("没写进去。确认现在开的是跟随，不是建图。"));
        return;
      }
      status_->setText(
        QString::fromUtf8("已设为 %1 m/s。这是上限，空旷处才会到。")
        .arg(vx, 0, 'f', 2));
    });

  if (smoother_ && smoother_->service_is_ready()) {
    smoother_->set_parameters(
      {rclcpp::Parameter(
          "max_velocity", std::vector<double>{vx, 0.0, kTurnMax})});
  }
}

}  // namespace robot_simple_commander

PLUGINLIB_EXPORT_CLASS(
  robot_simple_commander::SpeedPanel,
  rviz_common::Panel)
