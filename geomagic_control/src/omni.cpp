#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduVector.h>
#include <HL/hl.h>
#include <geometry_msgs/PoseStamped.h>
#include <pthread.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include <string.h>
#include <tf/transform_listener.h>
#include <sstream>
#include <tapi_lib/tapi_lib.hpp>
#include "tapi_geomagic_control/OmniFeedback.h"

int calibrationStyle;

struct OmniState
{
  hduVector3Dd position;  // 3x1 vector of position
  hduVector3Dd velocity;  // 3x1 vector of velocity
  hduVector3Dd inp_vel1;  // 3x1 history of velocity used for filtering velocity estimate
  hduVector3Dd inp_vel2;
  hduVector3Dd inp_vel3;
  hduVector3Dd out_vel1;
  hduVector3Dd out_vel2;
  hduVector3Dd out_vel3;
  hduVector3Dd pos_hist1;  // 3x1 history of position used for 2nd order backward difference estimate of velocity
  hduVector3Dd pos_hist2;
  hduVector3Dd rot;
  hduVector3Dd joints;
  hduVector3Dd force;  // 3 element double vector force[0], force[1], force[2]
  double thetas[7];
  int buttons[2];
  int buttons_prev[2];
  bool lock;
  hduVector3Dd lock_pos;
};

class PhantomROS
{
public:
  ros::NodeHandle n;
  ros::Publisher joint_pub;
  ros::Publisher *button1_pub, *button2_pub;
  ros::Publisher *pose_pub;
  ros::Subscriber haptic_sub;
  Tapi::Publisher *tpub;
  tf::TransformListener listener;
  geometry_msgs::PoseStamped pose;

  std::string omni_name;

  OmniState *state;

  void init(OmniState *s)
  {
    ros::param::param(std::string("~device_name"), omni_name, std::string("Geomagic"));

    // Publish joint states for robot_state_publisher, and anyone else who wants them.
    ROS_INFO("Omni name: %s", omni_name.c_str());
    std::ostringstream joint_topic;
    joint_topic << omni_name << "_joint_states";
    joint_pub = n.advertise<sensor_msgs::JointState>(joint_topic.str(), 1);

    tpub = new Tapi::Publisher(&n, "Geomagic Touch");
    button1_pub = tpub->AddFeature<std_msgs::Bool>("Grey button", 10);
    button2_pub = tpub->AddFeature<std_msgs::Bool>("White Button", 10);
    pose_pub = tpub->AddFeature<geometry_msgs::PoseStamped>("Pose for iiwa", 1);

    // Subscribe to NAME_force_feedback.
    std::ostringstream force_feedback_topic;
    force_feedback_topic << omni_name << "_force_feedback";
    haptic_sub = n.subscribe(force_feedback_topic.str(), 100, &PhantomROS::force_callback, this);

    state = s;
    state->buttons[0] = 0;
    state->buttons[1] = 0;
    state->buttons_prev[0] = 0;
    state->buttons_prev[1] = 0;
    hduVector3Dd zeros(0, 0, 0);
    state->velocity = zeros;
    state->inp_vel1 = zeros;   // 3x1 history of velocity
    state->inp_vel2 = zeros;   // 3x1 history of velocity
    state->inp_vel3 = zeros;   // 3x1 history of velocity
    state->out_vel1 = zeros;   // 3x1 history of velocity
    state->out_vel2 = zeros;   // 3x1 history of velocity
    state->out_vel3 = zeros;   // 3x1 history of velocity
    state->pos_hist1 = zeros;  // 3x1 history of position
    state->pos_hist2 = zeros;  // 3x1 history of position
    state->lock = false;
    state->lock_pos = zeros;
  }

  /*******************************************************************************
   ROS node callback.
   *******************************************************************************/
  void force_callback(const tapi_geomagic_control::OmniFeedbackConstPtr &omnifeed)
  {
    // Some people might not like this extra damping, but it helps to stabilize the overall force feedback. It isn't
    // like we are getting direct impedance matching from the omni anyway
    state->force[0] = omnifeed->force.x - 0.001 * state->velocity[0];
    state->force[1] = omnifeed->force.y - 0.001 * state->velocity[1];
    state->force[2] = omnifeed->force.z - 0.001 * state->velocity[2];

    state->lock_pos[0] = omnifeed->position.x;
    state->lock_pos[1] = omnifeed->position.y;
    state->lock_pos[2] = omnifeed->position.z;
  }

  void publish_omni_state()
  {
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(6);
    joint_state.position.resize(6);
    joint_state.name[0] = "waist";
    joint_state.position[0] = -state->thetas[1];
    joint_state.name[1] = "shoulder";
    joint_state.position[1] = state->thetas[2];
    joint_state.name[2] = "elbow";
    joint_state.position[2] = state->thetas[3];
    joint_state.name[3] = "yaw";
    joint_state.position[3] = -state->thetas[4] + M_PI;
    joint_state.name[4] = "pitch";
    joint_state.position[4] = -state->thetas[5] - 3 * M_PI / 4;
    joint_state.name[5] = "roll";
    joint_state.position[5] = -(-state->thetas[6] - M_PI);
    joint_pub.publish(joint_state);

    if ((state->buttons[0] != state->buttons_prev[0]) or (state->buttons[1] != state->buttons_prev[1]))
    {
      if ((state->buttons[0] == state->buttons[1]) and (state->buttons[0] == 1))
        state->lock = !(state->lock);
      state->buttons_prev[0] = state->buttons[0];
      state->buttons_prev[1] = state->buttons[1];
      std_msgs::Bool button1, button2;
      button1.data = state->buttons[0];
      button1_pub->publish(button1);
      button2.data = state->buttons[1];
      button2_pub->publish(button2);
    }

    // Publish pose for iiwa
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform("/base", "/stylus", ros::Time(0), transform);
      pose.header.seq++;
      pose.header.stamp = ros::Time::now();
      pose.pose.position.x = 2.5 * transform.getOrigin().y() + 0.25;
      pose.pose.position.y = (-2.5) * transform.getOrigin().x();
      pose.pose.position.z = 2.5 * transform.getOrigin().z() + 0.1;
      tf::Quaternion orient, orient_after;
      orient.setX(transform.getRotation().x());
      orient.setY(transform.getRotation().y());
      orient.setZ(transform.getRotation().z());
      orient.setW(transform.getRotation().w());
      tf::Quaternion rot = tf::createQuaternionFromYaw((-90.0) * M_PI / 180.0);
      orient_after = orient * rot;
      pose.pose.orientation.x = orient_after.x();
      pose.pose.orientation.y = orient_after.y();
      pose.pose.orientation.z = orient_after.z();
      pose.pose.orientation.w = orient_after.w();
      pose_pub->publish(pose);
    }
    catch (tf::TransformException ex)
    {
      ROS_INFO("Transformation error");
    }
  }
};

HDCallbackCode HDCALLBACK omni_state_callback(void *pUserData)
{
  OmniState *omni_state = static_cast<OmniState *>(pUserData);
  if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE)
  {
    ROS_DEBUG("Updating calibration...");
    hdUpdateCalibration(calibrationStyle);
  }
  hdBeginFrame(hdGetCurrentDevice());
  // Get angles, set forces
  hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, omni_state->rot);
  hdGetDoublev(HD_CURRENT_POSITION, omni_state->position);
  hdGetDoublev(HD_CURRENT_JOINT_ANGLES, omni_state->joints);
  // BUGFIX: (because x is calibrated as 35 Degree instead of 0 degree)
  omni_state->rot[0] -= 35.0 * M_PI / 180.0;

  hduVector3Dd vel_buff(0, 0, 0);
  vel_buff = (omni_state->position * 3 - 4 * omni_state->pos_hist1 + omni_state->pos_hist2) /
             0.002;  // mm/s, 2nd order backward dif
  omni_state->velocity =
      (.2196 * (vel_buff + omni_state->inp_vel3) + .6588 * (omni_state->inp_vel1 + omni_state->inp_vel2)) / 1000.0 -
      (-2.7488 * omni_state->out_vel1 + 2.5282 * omni_state->out_vel2 -
       0.7776 * omni_state->out_vel3);  // cutoff freq of 20 Hz
  omni_state->pos_hist2 = omni_state->pos_hist1;
  omni_state->pos_hist1 = omni_state->position;
  omni_state->inp_vel3 = omni_state->inp_vel2;
  omni_state->inp_vel2 = omni_state->inp_vel1;
  omni_state->inp_vel1 = vel_buff;
  omni_state->out_vel3 = omni_state->out_vel2;
  omni_state->out_vel2 = omni_state->out_vel1;
  omni_state->out_vel1 = omni_state->velocity;
  if (omni_state->lock == true)
    omni_state->force = 0.04 * (omni_state->lock_pos - omni_state->position) - 0.001 * omni_state->velocity;

  hdSetDoublev(HD_CURRENT_FORCE, omni_state->force);

  // Get buttons
  int nButtons = 0;
  hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
  omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
  omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

  hdEndFrame(hdGetCurrentDevice());

  HDErrorInfo error;
  if (HD_DEVICE_ERROR(error = hdGetError()))
  {
    hduPrintError(stderr, &error, "Error during main scheduler callback");
    if (hduIsSchedulerError(&error))
      return HD_CALLBACK_DONE;
  }

  double t[7] = { 0.,
                  omni_state->joints[0],
                  omni_state->joints[1],
                  omni_state->joints[2] - omni_state->joints[1],
                  omni_state->rot[0],
                  omni_state->rot[1],
                  omni_state->rot[2] };
  for (int i = 0; i < 7; i++)
    omni_state->thetas[i] = t[i];
  return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
 Automatic Calibration of Phantom Device - No character inputs
 *******************************************************************************/
void HHD_Auto_Calibration()
{
  int supportedCalibrationStyles;
  HDErrorInfo error;

  hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
  if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET)
  {
    calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
    ROS_INFO("HD_CALIBRATION_ENCODER_RESE..");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL)
  {
    calibrationStyle = HD_CALIBRATION_INKWELL;
    ROS_INFO("HD_CALIBRATION_INKWELL..");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_AUTO)
  {
    calibrationStyle = HD_CALIBRATION_AUTO;
    ROS_INFO("HD_CALIBRATION_AUTO..");
  }
  if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET)
  {
    do
    {
      hdUpdateCalibration(calibrationStyle);
      ROS_INFO("Calibrating.. (put stylus in well)");
      if (HD_DEVICE_ERROR(error = hdGetError()))
      {
        hduPrintError(stderr, &error, "Reset encoders reset failed.");
        break;
      }
    } while (hdCheckCalibration() != HD_CALIBRATION_OK);
    ROS_INFO("Calibration complete.");
  }
  if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
    ROS_INFO("Please place the device into the inkwell for calibration.");
}

void *ros_publish(void *ptr)
{
  PhantomROS *omni_ros = (PhantomROS *)ptr;
  int publish_rate;
  omni_ros->n.param(std::string("publish_rate"), publish_rate, 100);
  ROS_INFO("Publish rate set to %d", publish_rate);
  ros::Rate loop_rate(publish_rate);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  while (ros::ok())
  {
    omni_ros->publish_omni_state();
    loop_rate.sleep();
  }
  return NULL;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tapi_geomagic_control");
  ros::NodeHandle nh("~");
  std::string device_name = "";
  nh.getParam("device_name", device_name);
  ROS_INFO("Device name: %s", device_name.c_str());
  ////////////////////////////////////////////////////////////////
  // Init Phantom
  ////////////////////////////////////////////////////////////////
  HDErrorInfo error;
  HHD hHD;
  hHD = hdInitDevice(device_name.c_str());  // use ros param and set in launch file
  if (HD_DEVICE_ERROR(error = hdGetError()))
  {
    // hduPrintError(stderr, &error, "Failed to initialize haptic device");
    ROS_ERROR("Failed to initialize haptic device");  //: %s", &error);
    return -1;
  }

  ROS_INFO("Found %s.", hdGetString(HD_DEVICE_MODEL_TYPE));
  hdEnable(HD_FORCE_OUTPUT);
  hdStartScheduler();
  if (HD_DEVICE_ERROR(error = hdGetError()))
  {
    ROS_ERROR("Failed to start the scheduler");  //, &error);
    return -1;
  }
  HHD_Auto_Calibration();

  ////////////////////////////////////////////////////////////////
  // Init ROS
  ////////////////////////////////////////////////////////////////

  OmniState state;
  PhantomROS omni_ros;
  omni_ros.init(&state);
  hdScheduleAsynchronous(omni_state_callback, &state, HD_MAX_SCHEDULER_PRIORITY);
  sleep(2);

  ////////////////////////////////////////////////////////////////
  // Loop and publish
  ////////////////////////////////////////////////////////////////
  pthread_t publish_thread;
  pthread_create(&publish_thread, NULL, ros_publish, (void *)&omni_ros);
  pthread_join(publish_thread, NULL);

  ROS_INFO("Ending Session....");
  hdStopScheduler();
  hdDisableDevice(hHD);

  return 0;
}
