#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <control_msgs/ControlVector.h>
#include <adaptive_controller/Data.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <stdlib.h> 

class DataAssembler
{
private:
	std::string node_name_;
	std::string est_vel_topic_;
	std::string cmd_vel_topic_;
	std::string control_topic_;
	std::string data_topic_;
	ros::NodeHandle* node_;
	ros::Publisher data_pub_;
	double rate_;

public:
	DataAssembler():
		node_name_("data_assembler"),
		est_vel_topic_("/estimated_vel_stamped"),
		cmd_vel_topic_("/cmd_vel_stamped"),
		control_topic_("/control"),
		rate_(30.0),
		data_topic_("/training_data")
	{

	}

	void start()
	{
		node_ = new ros::NodeHandle("~");
		node_->param("cmd_vel_stamped", cmd_vel_topic_, cmd_vel_topic_);
		node_->param("est_vel_stamped", est_vel_topic_, est_vel_topic_);
		node_->param("control", control_topic_, control_topic_);
		node_->param("training_data", data_topic_, data_topic_);
		node_->param("rate", rate_, rate_);
		data_pub_ = node_->advertise<adaptive_controller::Data>(data_topic_, rate_);
		message_filters::Subscriber<geometry_msgs::TwistStamped> cmd_vel_sub(*node_, cmd_vel_topic_, rate_);
		message_filters::Subscriber<geometry_msgs::TwistStamped> est_vel_sub(*node_, est_vel_topic_, rate_);
		message_filters::Subscriber<control_msgs::ControlVector> control_sub(*node_, control_topic_, rate_);
		typedef	message_filters::sync_policies::ApproximateTime<geometry_msgs::TwistStamped,
		 geometry_msgs::TwistStamped,
		 control_msgs::ControlVector > SyncPolicy;
		message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(rate_), cmd_vel_sub, est_vel_sub, control_sub);
		sync.registerCallback(boost::bind(&DataAssembler::update, this, _1, _2, _3));
		ros::Rate r(rate_);
		while(node_->ok())
		{
			ros::spinOnce();
			r.sleep();
		}
	}

	void update(const geometry_msgs::TwistStampedConstPtr& cmd_vel,
		const geometry_msgs::TwistStampedConstPtr& est_vel,
		const control_msgs::ControlVectorConstPtr& control)
	{
		if(isZero(*cmd_vel)){
			return;
		}
		adaptive_controller::Data data;
		data.cmd_vel = *cmd_vel;
		data.est_vel = *est_vel;
		data.control = *control;
		data_pub_.publish(data);
	}

	bool isZero(const geometry_msgs::TwistStamped& cmd_vel)
	{
		if(cmd_vel.twist.linear.x != 0.0 || 
			cmd_vel.twist.linear.y !=0.0 ||
			cmd_vel.twist.linear.z !=0.0 ||
			cmd_vel.twist.angular.x != 0.0 ||
			cmd_vel.twist.angular.y != 0.0 ||
			cmd_vel.twist.angular.z != 0.0)
			return false;
		return true;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "data_set_collector");
	DataAssembler data_collector;
	data_collector.start();
	return 0;
}