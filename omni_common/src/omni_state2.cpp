#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>
#define BT_EULER_DEFAULT_ZYX
#include <bullet/LinearMath/btMatrix3x3.h>

#include "omni_msgs/OmniButtonEvent.h"
#include "omni_msgs/OmniFeedback.h"
#include "omni_msgs/OmniState.h"
#include <pthread.h>

#include <tf/transform_broadcaster.h>

#include <time.h>
#include <sys/time.h>

float prev_time;
int calibrationStyle;

struct timeval t_aux;

struct OmniState {
  hduVector3Dd position;  //3x1 vector of position
  hduVector3Dd velocity;  //3x1 vector of velocity
  hduVector3Dd inp_vel1;  //3x1 history of velocity used for filtering velocity estimate
  hduVector3Dd inp_vel2;
  hduVector3Dd inp_vel3;
  hduVector3Dd out_vel1;
  hduVector3Dd out_vel2;
  hduVector3Dd out_vel3;
  hduVector3Dd pos_hist1; //3x1 history of position used for 2nd order backward difference estimate of velocity
  hduVector3Dd pos_hist2;
  hduQuaternion rot;
  hduVector3Dd joints;
  hduVector3Dd force;   //3 element double vector force[0], force[1], force[2]
  float thetas[7];
  int buttons[2];
  int buttons_prev[2];
  bool lock;
  bool close_gripper;
  hduVector3Dd lock_pos;
  double units_ratio;
  hduMatrix omni_mx;
};

class PhantomROS {

public:
  ros::NodeHandle n;
  ros::Publisher state_publisher;
  ros::Publisher pose_publisher;
  ros::Publisher button_publisher;
  ros::Publisher joint_publisher;
  ros::Subscriber haptic_sub;
  std::string omni_name, ref_frame, units,type;
  std::string link_names[7];

  OmniState *state;

  void init(OmniState *s) {
    ros::param::param(std::string("~omni_name"), omni_name, std::string("phantom"));
    ros::param::param(std::string("~reference_frame"), ref_frame, std::string("/map"));
    ros::param::param(std::string("~units"), units, std::string("mm"));
    ros::param::param(std::string("~type"), type, std::string(""));
    
    //Publish button state on NAME/button
    std::ostringstream stream1;
    stream1 << omni_name << "/button";
    std::string button_topic = std::string(stream1.str());
    button_publisher = n.advertise<omni_msgs::OmniButtonEvent>(button_topic.c_str(), 100);
    
    //Publish on NAME/state
    std::ostringstream stream2;
    stream2 << omni_name << "/state";
    std::string state_topic_name = std::string(stream2.str());
    state_publisher = n.advertise<omni_msgs::OmniState>(state_topic_name.c_str(), 1);
    
    //Subscribe to NAME/force_feedback
    std::ostringstream stream3;
    stream3 << omni_name << "/force_feedback";
    std::string force_feedback_topic = std::string(stream3.str());
    haptic_sub = n.subscribe(force_feedback_topic.c_str(), 1, &PhantomROS::force_callback, this);
    
    //Publish on NAME/pose
    std::ostringstream stream4;
    stream4 << omni_name << "/pose";
    std::string pose_topic_name = std::string(stream4.str());
    pose_publisher = n.advertise<geometry_msgs::PoseStamped>(pose_topic_name.c_str(), 1);
    
    //Publish on NAME/joint_states
    std::ostringstream stream5;
    stream5 << omni_name << "/joint_states";
    std::string joint_topic_name = std::string(stream5.str());
    joint_publisher = n.advertise<sensor_msgs::JointState>(joint_topic_name.c_str(), 1);


	// Link names
    std::ostringstream stream_base;
    stream_base << type << "base";
    std::string base = std::string(stream_base.str());
    
    std::ostringstream stream_torso;
    stream_torso << type << "torso";
    std::string torso = std::string(stream_torso.str());
    
    std::ostringstream stream_upper_arm;
    stream_upper_arm << type << "upper_arm";
    std::string upper_arm = std::string(stream_upper_arm.str());
    
    std::ostringstream stream_lower_arm;
    stream_lower_arm << type << "lower_arm";
    std::string lower_arm = std::string(stream_lower_arm.str());
    
    std::ostringstream stream_wrist;
    stream_wrist << type << "wrist";
    std::string wrist = std::string(stream_wrist.str());
    
    std::ostringstream stream_tip;
    stream_tip << type << "tip";
    std::string tip = std::string(stream_tip.str());
    
    std::ostringstream stream_stylus;
    stream_stylus << type << "stylus";
    std::string stylus = std::string(stream_stylus.str());
    
    link_names[0] = base.c_str();
    link_names[1] = torso.c_str();
    link_names[2] = upper_arm.c_str();
    link_names[3] = lower_arm.c_str();
    link_names[4] = wrist.c_str();
    link_names[5] = tip.c_str();
    link_names[6] = stylus.c_str();
    
    state = s;
    state->buttons[0] = 0;
    state->buttons[1] = 0;
    state->buttons_prev[0] = 0;
    state->buttons_prev[1] = 0;
    hduVector3Dd zeros(0, 0, 0);
    state->velocity = zeros;
    state->inp_vel1 = zeros;  //3x1 history of velocity
    state->inp_vel2 = zeros;  //3x1 history of velocity
    state->inp_vel3 = zeros;  //3x1 history of velocity
    state->out_vel1 = zeros;  //3x1 history of velocity
    state->out_vel2 = zeros;  //3x1 history of velocity
    state->out_vel3 = zeros;  //3x1 history of velocity
    state->pos_hist1 = zeros; //3x1 history of position
    state->pos_hist2 = zeros; //3x1 history of position
    state->lock = false;
    state->close_gripper = false;
    state->lock_pos = zeros;
    if (!units.compare("mm"))
      state->units_ratio = 1.0;
    else if (!units.compare("cm"))
      state->units_ratio = 10.0;
    else if (!units.compare("dm"))
      state->units_ratio = 100.0;
    else if (!units.compare("m"))
      state->units_ratio = 1000.0;
    else
    {
      state->units_ratio = 1.0;
      ROS_WARN("Unknown units [%s] unsing [mm]", units.c_str());
      units = "mm";
    }
    ROS_INFO("PHaNTOM position given in [%s], ratio [%.1f]", units.c_str(), state->units_ratio);
  }

  /*******************************************************************************
   ROS node callback.
   *******************************************************************************/
  void force_callback(const omni_msgs::OmniFeedbackConstPtr& omnifeed) {
    ////////////////////Some people might not like this extra damping, but it
    ////////////////////helps to stabilize the overall force feedback. It isn't
    ////////////////////like we are getting direct impedance matching from the
    ////////////////////omni anyway
    state->force[0] = omnifeed->force.x - 0.001 * state->velocity[0];
    state->force[1] = omnifeed->force.y - 0.001 * state->velocity[1];
    state->force[2] = omnifeed->force.z - 0.001 * state->velocity[2];

    state->lock_pos[0] = omnifeed->position.x;
    state->lock_pos[1] = omnifeed->position.y;
    state->lock_pos[2] = omnifeed->position.z;
  }

  void publish_omni_state() {
    // Build the state msg
    omni_msgs::OmniState state_msg;
    // Locked
    state_msg.locked = state->lock;
    state_msg.close_gripper = state->close_gripper;
    
    // Notice that we are changing Y <---> Z and inverting the Z-force_feedback
    // Position
    state_msg.pose.position.x = state->position[0];
    state_msg.pose.position.y = -state->position[2];
    state_msg.pose.position.z = state->position[1];
    // Orientation

	tf::Transform transform;
	transform.setOrigin(tf::Vector3(state->omni_mx[3][0], state->omni_mx[3][1], state->omni_mx[3][2]));
	transform.getBasis().setValue(state->omni_mx[0][0], state->omni_mx[1][0], state->omni_mx[2][0],
                                      state->omni_mx[0][1], state->omni_mx[1][1], state->omni_mx[2][1], 
                                      state->omni_mx[0][2], state->omni_mx[1][2], state->omni_mx[2][2]);
     
    //~ // Rotation with grips                                 
    //~ state_msg.pose.orientation.x = transform.getRotation()[0];
    //~ state_msg.pose.orientation.y = -transform.getRotation()[2];
    //~ state_msg.pose.orientation.z = transform.getRotation()[1];
    //~ state_msg.pose.orientation.w = transform.getRotation()[3];
  
	// Rotation with simulated HUG
    state_msg.pose.orientation.x = transform.getRotation()[1];
    state_msg.pose.orientation.y = transform.getRotation()[0];
    state_msg.pose.orientation.z = -transform.getRotation()[2];
    state_msg.pose.orientation.w = transform.getRotation()[3];
    
    // Velocity
    state_msg.velocity.x = state->velocity[0];
    state_msg.velocity.y = state->velocity[1];
    state_msg.velocity.z = state->velocity[2];
  
    // TODO: Append Current to the state msg
    state_msg.header.stamp = ros::Time::now();
    state_publisher.publish(state_msg);
    
    // Joint names
    std::ostringstream stream_waist;
    stream_waist << type << "waist";
    std::string waist = std::string(stream_waist.str());
    
    std::ostringstream stream_shoulder;
    stream_shoulder << type << "shoulder";
    std::string shoulder = std::string(stream_shoulder.str());
    
    std::ostringstream stream_elbow;
    stream_elbow << type << "elbow";
    std::string elbow = std::string(stream_elbow.str());
    
    std::ostringstream stream_yaw;
    stream_yaw << type << "yaw";
    std::string yaw = std::string(stream_yaw.str());
    
    std::ostringstream stream_pitch;
    stream_pitch << type << "pitch";
    std::string pitch = std::string(stream_pitch.str());
    
    std::ostringstream stream_roll;
    stream_roll << type << "roll";
    std::string roll = std::string(stream_roll.str());
    
    // Publish the JointState msg
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(6);
    joint_state.position.resize(6);
    joint_state.name[0] = waist.c_str();
    joint_state.position[0] = -state->thetas[1];
    joint_state.name[1] = shoulder.c_str();
    joint_state.position[1] = state->thetas[2];
    joint_state.name[2] = elbow.c_str();
    joint_state.position[2] = state->thetas[3];
    joint_state.name[3] = yaw.c_str();
    joint_state.position[3] = -state->thetas[4] + M_PI;
    joint_state.name[4] = pitch.c_str();
    joint_state.position[4] = -state->thetas[5] - 3*M_PI/4;
    joint_state.name[5] = roll.c_str();
    joint_state.position[5] = state->thetas[6] + M_PI;
    joint_publisher.publish(joint_state);
    
    // Build the pose msg
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = state_msg.header;
    pose_msg.header.frame_id = link_names[0].c_str();
    pose_msg.pose = state_msg.pose;
    pose_msg.pose.position.x /= 1000.0;
    pose_msg.pose.position.y /= 1000.0;
    pose_msg.pose.position.z /= 1000.0;
    pose_publisher.publish(pose_msg);

    if ((state->buttons[0] != state->buttons_prev[0])
        or (state->buttons[1] != state->buttons_prev[1])) 
    {
      if (state->buttons[0] == 1) {
        state->close_gripper = !(state->close_gripper);
      }
      if (state->buttons[1] == 1) {
        state->lock = !(state->lock);
      }
      omni_msgs::OmniButtonEvent button_event;
      button_event.grey_button = state->buttons[0];
      button_event.white_button = state->buttons[1];
      state->buttons_prev[0] = state->buttons[0];
      state->buttons_prev[1] = state->buttons[1];
      button_publisher.publish(button_event);
    }
  }
};

double timeval_diff(struct timeval *a, struct timeval *b)
{
	return (double)(a->tv_sec + (double)a->tv_usec/1000000)-
			(double)(b->tv_sec + (double)b->tv_usec/1000000);
}

HDCallbackCode HDCALLBACK omni_state_callback(void *pUserData) 
{
	
  struct timeval t_now;
  double secs;
  
  //~ gettimeofday(&t_now, NULL);
  //~ secs = timeval_diff(&t_now, &t_aux);
  //~ ROS_INFO("%f", secs);
  //~ gettimeofday(&t_aux, NULL);
  
  
  
  OmniState *omni_state = static_cast<OmniState *>(pUserData);
  if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
    ROS_DEBUG("Updating calibration...");
      hdUpdateCalibration(calibrationStyle);
    }
  hdBeginFrame(hdGetCurrentDevice());
  // Get transform and angles
  hduVector3Dd gimbal_angles;

  hdGetDoublev(HD_CURRENT_POSITION, omni_state->position); 
  hdGetDoublev(HD_CURRENT_TRANSFORM, omni_state->omni_mx);
  //~ hdGetDoublev(HD_CURRENT_VELOCITY, omni_state->velocity);
  hdGetDoublev(HD_CURRENT_JOINT_ANGLES, omni_state->joints);
  hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal_angles);
  
    // Velocity estimation
	hduVector3Dd vel_buff(0, 0, 0);
	vel_buff = (omni_state->position * 3 - 4 * omni_state->pos_hist1
      + omni_state->pos_hist2) / 0.002;  //(units)/s, 2nd order backward dif
	omni_state->velocity = (.2196 * (vel_buff + omni_state->inp_vel3)
      + .6588 * (omni_state->inp_vel1 + omni_state->inp_vel2)) / 1000.0
      - (-2.7488 * omni_state->out_vel1 + 2.5282 * omni_state->out_vel2
      - 0.7776 * omni_state->out_vel3);  //cutoff freq of 20 Hz
	omni_state->pos_hist2 = omni_state->pos_hist1;
	omni_state->pos_hist1 = omni_state->position;
	omni_state->inp_vel3 = omni_state->inp_vel2;
	omni_state->inp_vel2 = omni_state->inp_vel1;
	omni_state->inp_vel1 = vel_buff;
	omni_state->out_vel3 = omni_state->out_vel2;
	omni_state->out_vel2 = omni_state->out_vel1;
	omni_state->out_vel1 = omni_state->velocity;

  hduVector3Dd feedback;
  // Notice that we are changing Y <---> Z and inverting the Z-force_feedback
  feedback[0] = omni_state->force[0];
  feedback[1] = omni_state->force[2];
  feedback[2] = -omni_state->force[1];
  hdSetDoublev(HD_CURRENT_FORCE, feedback);
  
  //~ hduVector3Dd zeros(0, 0, 0);
  //~ if (omni_state->lock == true) {
	  //~ ROS_INFO("pos_x = %f", omni_state->position[0]);
	  //~ ROS_INFO("pos_y = %f", omni_state->position[1]);
	  //~ ROS_INFO("pos_z = %f", omni_state->position[2]);
	  
	//~ double sum_pos = omni_state->position[0]
					//~ + omni_state->position[1]
					//~ + omni_state->position[2];
	  //~ 
	//~ double rad_ball = sqrt(pow(omni_state->position[0],2)
						//~ + pow(omni_state->position[1],2)
						//~ + pow(omni_state->position[2],2));
	  
	//~ if (rad_ball < 50){
		//~ omni_state->force[0] = 0.5 * (50 - rad_ball)
			//~ * (omni_state->position[0] / rad_ball);
			//~ - 0.001 * omni_state->velocity[0];
			//~ omni_state->force[1] = 0.5 * (50 - rad_ball)
			//~ * (omni_state->position[1] / rad_ball);
			//~ - 0.001 * omni_state->velocity[1];
			//~ omni_state->force[2] = 0.5 * (50 - rad_ball)
			//~ * (omni_state->position[2] / rad_ball);
			//~ - 0.001 * omni_state->velocity[2];
			
	//~ if (omni_state->position[0]>0) {
        //~ omni_state->force[0] = 2 * (0 - omni_state->position[0]);
	//~ }else{
		//~ omni_state->force = zeros;
	//~ }
  //~ }
  //ROS_INFO("f_x = %f", omni_state->force[0]);
  //ROS_INFO("f_y = %f", omni_state->force[1]);
  //~ //ROS_INFO("f_z = %f", omni_state->force[2]);
  //~ hdSetDoublev(HD_CURRENT_FORCE, omni_state->force);

  //Get buttons
  int nButtons = 0;
  hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
  omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
  omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

  hdEndFrame(hdGetCurrentDevice());

  HDErrorInfo error;
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    hduPrintError(stderr, &error, "Error during main scheduler callback");
    if (hduIsSchedulerError(&error))
      return HD_CALLBACK_DONE;
  }

  float t[7] = { 0., omni_state->joints[0], omni_state->joints[1],
      omni_state->joints[2] - omni_state->joints[1], gimbal_angles[0],
      gimbal_angles[1], gimbal_angles[2] };
  for (int i = 0; i < 7; i++)
    omni_state->thetas[i] = t[i];
  return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
 Automatic Calibration of Phantom Device - No character inputs
 *******************************************************************************/
void HHD_Auto_Calibration() {
  int supportedCalibrationStyles;
  HDErrorInfo error;

  hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
  if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET) {
    calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
    ROS_INFO("HD_CALIBRATION_ENCODER_RESET..");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL) {
    calibrationStyle = HD_CALIBRATION_INKWELL;
    ROS_INFO("HD_CALIBRATION_INKWELL..");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_AUTO) {
    calibrationStyle = HD_CALIBRATION_AUTO;
    ROS_INFO("HD_CALIBRATION_AUTO..");
  }
  if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET) {
    do {
      hdUpdateCalibration(calibrationStyle);
      ROS_INFO("Calibrating.. (put stylus in well)");
      if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Reset encoders reset failed.");
        break;
      }
    } while (hdCheckCalibration() != HD_CALIBRATION_OK);
    ROS_INFO("Calibration complete.");
  }
  while(hdCheckCalibration() != HD_CALIBRATION_OK) {
    usleep(1e6);
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT) 
      ROS_INFO("Please place the device into the inkwell for calibration");
    else if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
      ROS_INFO("Calibration updated successfully");
      hdUpdateCalibration(calibrationStyle);
    }
    else
      ROS_FATAL("Unknown calibration status");
  }
}

void *ros_publish(void *ptr) {
  PhantomROS *omni_ros = (PhantomROS *) ptr;
  int publish_rate;
  ros::param::param(std::string("~publish_rate"), publish_rate, 1000);
  ROS_INFO("Publishing PHaNTOM state at [%d] Hz", publish_rate);
  ros::Rate loop_rate(publish_rate);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  while (ros::ok()) {
    omni_ros->publish_omni_state();
    loop_rate.sleep();
  }
  return NULL;
}

int main(int argc, char** argv) {
  ////////////////////////////////////////////////////////////////
  // Init Phantom
  ////////////////////////////////////////////////////////////////
  HDErrorInfo error;
  HHD hHD;
  hHD = hdInitDevice(HD_DEFAULT_DEVICE);
  gettimeofday(&t_aux, NULL);
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    //hduPrintError(stderr, &error, "Failed to initialize haptic device");
    ROS_ERROR("Failed to initialize haptic device"); //: %s", &error);
    return -1;
  }

  ROS_INFO("Found %s.", hdGetString(HD_DEVICE_MODEL_TYPE));
  hdEnable(HD_FORCE_OUTPUT);
  hdStartScheduler();
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    ROS_ERROR("Failed to start the scheduler"); //, &error);
    return -1;
  }
  HHD_Auto_Calibration();

  ////////////////////////////////////////////////////////////////
  // Init ROS
  ////////////////////////////////////////////////////////////////
  ros::init(argc, argv, "omni_haptic_node");
  OmniState state;
  PhantomROS omni_ros;

  omni_ros.init(&state);
  hdScheduleAsynchronous(omni_state_callback, &state,
      HD_MAX_SCHEDULER_PRIORITY);

  ////////////////////////////////////////////////////////////////
  // Loop and publish
  ////////////////////////////////////////////////////////////////
  pthread_t publish_thread;
  pthread_create(&publish_thread, NULL, ros_publish, (void*) &omni_ros);
  pthread_join(publish_thread, NULL);

  ROS_INFO("Ending Session....");
  hdStopScheduler();
  hdDisableDevice(hHD);

  return 0;
}
