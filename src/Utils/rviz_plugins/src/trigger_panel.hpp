#ifndef RVIZ_TRIGGER_PANEL_H
#define RVIZ_TRIGGER_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QLabel>
#include <QPushButton>

namespace rviz
{
class RvizTriggerPanel : public rviz::Panel
{
  Q_OBJECT
public:
  explicit RvizTriggerPanel(QWidget *parent = 0);
  ~RvizTriggerPanel() override;

  void onInitialize() override;

protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_trigger_;

  QLabel *label_;
  QPushButton *button_;

private Q_SLOTS:
  void buttonActivated();
};
} // namespace rviz

#endif // RVIZ_TRIGGER_PANEL_H
