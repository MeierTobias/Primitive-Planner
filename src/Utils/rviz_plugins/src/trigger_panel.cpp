#include "trigger_panel.hpp"
#include <QVBoxLayout>
#include <geometry_msgs/PoseStamped.h>

namespace rviz
{

RvizTriggerPanel::RvizTriggerPanel(QWidget *parent)
  : Panel(parent)
{
  // Create a label and a button, displayed vertically (the V in VBox means vertical)
  const auto layout = new QVBoxLayout(this);
  // Create a button and a label for the button
  label_ = new QLabel("[no data]");
  button_ = new QPushButton("GO!");
  // Add those elements to the GUI layout
  layout->addWidget(label_);
  layout->addWidget(button_);

  // Connect the event of when the button is released to our callback,
  // so pressing the button results in the buttonActivated callback being called.
  QObject::connect(button_, &QPushButton::released, this, &RvizTriggerPanel::buttonActivated);
}

RvizTriggerPanel::~RvizTriggerPanel() = default;

void RvizTriggerPanel::onInitialize()
{
  pub_trigger_ = nh_.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 1);
}

// When the widget's button is pressed, this callback is triggered,
// and then we publish a new message on our topic.
void RvizTriggerPanel::buttonActivated()
{
  geometry_msgs::PoseStamped message;
  pub_trigger_.publish(message);
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(rviz::RvizTriggerPanel, rviz::Panel)
