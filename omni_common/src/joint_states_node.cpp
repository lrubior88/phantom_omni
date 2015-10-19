#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

ros::Publisher pub;
sensor_msgs::JointState state_msg;

void initialize(void)
{
	
	std::string nameArr[] = {"left_waist", "left_shoulder", "left_elbow", 
							"left_yaw", "left_pitch", "left_roll",
							"right_waist", "right_shoulder", "right_elbow", 
							"right_yaw", "right_pitch", "right_roll"};
	std::vector<std::string> nameVec(nameArr, nameArr + 12);
	state_msg.name = nameVec;
	
	double posArr[] = {0.0, 0.269, -0.64, 6.285, 1.561, 0.0,
						0.0, 0.269, -0.64, 6.285, 1.561, 0.0};
	std::vector<double> posVec(posArr, posArr + 12);
	state_msg.position = posVec;
	
	return;
}

void left_update(const sensor_msgs::JointState::ConstPtr &omni_states)
{

	sensor_msgs::JointState current_state_msg;
	current_state_msg.header = 		omni_states->header;		

	// Left Phantom
	current_state_msg.name = 		omni_states->name;
	current_state_msg.position =	omni_states->position;
	
	// Right Phantom
	std::vector<std::string>::const_iterator first_name = state_msg.name.begin()+ 6;
	std::vector<std::string>::const_iterator last_name = state_msg.name.end();
	current_state_msg.name.insert(current_state_msg.name.begin()+6, first_name, last_name);

	std::vector<double>::const_iterator first_pos = state_msg.position.begin()+ 6;
	std::vector<double>::const_iterator last_pos = state_msg.position.end();
	current_state_msg.position.insert(current_state_msg.position.begin()+6, first_pos, last_pos);
		
	state_msg = current_state_msg;
	pub.publish(state_msg);
}

void right_update(const sensor_msgs::JointState::ConstPtr &omni_states)
{
	
	sensor_msgs::JointState current_state_msg;
	current_state_msg.header = 		omni_states->header;		

	// Left Phantom
	std::vector<std::string>::const_iterator first_name = state_msg.name.begin();
	std::vector<std::string>::const_iterator last_name = state_msg.name.begin()+ 6;
	current_state_msg.name.insert(current_state_msg.name.begin(), first_name, last_name);

	std::vector<double>::const_iterator first_pos = state_msg.position.begin();
	std::vector<double>::const_iterator last_pos = state_msg.position.begin()+ 6;
	current_state_msg.position.insert(current_state_msg.position.begin(), first_pos, last_pos);

	
	// Right Phantom		
	first_name = omni_states->name.begin();
	last_name = omni_states->name.end();
	current_state_msg.name.insert(current_state_msg.name.begin()+6, first_name, last_name);

	first_pos = omni_states->position.begin();
	last_pos = omni_states->position.end();
	current_state_msg.position.insert(current_state_msg.position.begin()+6, first_pos, last_pos);
		
	state_msg = current_state_msg;
	pub.publish(state_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_states_node");
  ros::NodeHandle node;
  
  initialize();

  pub = node.advertise<sensor_msgs::JointState>("/joint_states", 0);
  ros::Subscriber sub_left = node.subscribe("/left_phantom/joint_states", 0, left_update);
  ros::Subscriber sub_right = node.subscribe("/right_phantom/joint_states", 0, right_update);

  ros::spin();

  return 0;
}
