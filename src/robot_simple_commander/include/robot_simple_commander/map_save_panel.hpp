#ifndef ROBOT_SIMPLE_COMMANDER__MAP_SAVE_PANEL_HPP_
#define ROBOT_SIMPLE_COMMANDER__MAP_SAVE_PANEL_HPP_

#include <QLineEdit>
#include <QProcess>
#include <QPushButton>

#include "rviz_common/config.hpp"
#include "rviz_common/panel.hpp"

namespace robot_simple_commander
{

class MapSavePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MapSavePanel(QWidget * parent = nullptr);

  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  void onSave();
  void onSaveFinished(int exit_code, QProcess::ExitStatus status);

private:
  QString mapPrefix() const;

  QLineEdit * path_{nullptr};
  QPushButton * save_btn_{nullptr};
  QProcess * proc_{nullptr};
};

}  // namespace robot_simple_commander

#endif  // ROBOT_SIMPLE_COMMANDER__MAP_SAVE_PANEL_HPP_
