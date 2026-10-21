#include "robot_simple_commander/map_save_panel.hpp"

#include <QHBoxLayout>
#include <QDir>
#include <QFileInfo>
#include <QSizePolicy>

#include "pluginlib/class_list_macros.hpp"

namespace robot_simple_commander
{

namespace
{
const char * kDefaultSuffix = "/RoboGait_ws/src/robot_slam_mapping/maps/today";
}  // namespace

MapSavePanel::MapSavePanel(QWidget * parent)
: Panel(parent)
{
  path_ = new QLineEdit(QDir::homePath() + kDefaultSuffix);
  save_btn_ = new QPushButton(QString::fromUtf8("保存"));
  proc_ = new QProcess(this);

  auto * row = new QHBoxLayout;
  row->setContentsMargins(4, 4, 4, 4);
  row->addWidget(path_, 1);
  row->addWidget(save_btn_);
  setLayout(row);
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);

  connect(save_btn_, &QPushButton::clicked, this, &MapSavePanel::onSave);
  connect(
    proc_,
    QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
    this, &MapSavePanel::onSaveFinished);
}

void MapSavePanel::save(rviz_common::Config config) const
{
  Panel::save(config);
  config.mapSetValue("map_path", path_->text());
}

void MapSavePanel::load(const rviz_common::Config & config)
{
  Panel::load(config);
  QString stored;
  if (config.mapGetString("map_path", &stored) && !stored.isEmpty()) {
    path_->setText(stored);
  }
}

QString MapSavePanel::mapPrefix() const
{
  QString prefix = path_->text().trimmed();
  if (prefix.endsWith(".yaml") || prefix.endsWith(".pgm") || prefix.endsWith(".png")) {
    const int dot = prefix.lastIndexOf('.');
    if (dot > 0) {
      prefix = prefix.left(dot);
    }
  }
  return prefix;
}

void MapSavePanel::onSave()
{
  const QString prefix = mapPrefix();
  if (prefix.isEmpty() || !QDir().mkpath(QFileInfo(prefix).absolutePath())) {
    save_btn_->setText(QString::fromUtf8("失败"));
    return;
  }

  path_->setText(prefix);
  save_btn_->setEnabled(false);
  save_btn_->setText(QString::fromUtf8("保存中"));
  proc_->start(
    "ros2",
    {"run", "nav2_map_server", "map_saver_cli", "-f", prefix});
  if (!proc_->waitForStarted(3000)) {
    save_btn_->setEnabled(true);
    save_btn_->setText(QString::fromUtf8("失败"));
  }
}

void MapSavePanel::onSaveFinished(int exit_code, QProcess::ExitStatus status)
{
  save_btn_->setEnabled(true);
  const bool ok = status == QProcess::NormalExit && exit_code == 0;
  save_btn_->setText(ok ? QString::fromUtf8("已保存") : QString::fromUtf8("失败"));
}

}  // namespace robot_simple_commander

PLUGINLIB_EXPORT_CLASS(
  robot_simple_commander::MapSavePanel,
  rviz_common::Panel)
