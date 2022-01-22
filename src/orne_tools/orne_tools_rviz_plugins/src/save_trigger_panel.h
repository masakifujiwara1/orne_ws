#ifndef STATE_TRIGGER_PANEL_H
#define STATE_TRIGGER_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

class QPushButton;

namespace orne_tools_rviz_plugins
{

class SaveTriggerPanel: public rviz::Panel
{
Q_OBJECT
public:
  SaveTriggerPanel( QWidget* parent = 0 );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

public Q_SLOTS:
  void pushSaveWaypoints();
    
protected:
  ros::NodeHandle nh_;
  ros::ServiceClient save_client_;
  QPushButton *save_button_;
};

} // end namespace orne_rviz_plugins

#endif // STATE_TRIGGER_PANEL_H
