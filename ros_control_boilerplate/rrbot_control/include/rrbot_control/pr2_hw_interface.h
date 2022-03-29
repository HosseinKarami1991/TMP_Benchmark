
#ifndef RRBOT_CONTROL__RRBOT_HW_INTERFACE_H
#define RRBOT_CONTROL__RRBOT_HW_INTERFACE_H
#include <sensor_msgs/JointState.h>
#include <ros_control_boilerplate/generic_hw_interface.h>

namespace rrbot_control
{

/// \brief Hardware interface for a robot
class RRBotHWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  RRBotHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration &elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration &elapsed_time);

  /** \breif Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration &period);
 void psoeCb(const sensor_msgs::JointStatePtr & msg);
private:
 ros::Publisher pubcommand;
 ros::Subscriber subjointpos;
};  // class

}  // namespace

#endif
