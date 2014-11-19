#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <adaptive_controller/DataSet.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <stdlib.h> 

template <class T>
class BoundedVector
{
private:
	int max_size_;
	std::vector<T> data_;
public:
	int size()
	{
		return data_.size();
	}
	BoundedVector(): max_size_(30)
	{

	}
	
	BoundedVector(int max_size):
		max_size_(max_size)
	{

	}

	~BoundedVector()
	{

	}
	
	bool empty()
	{
		return data_.empty();
	}
	int get_max_size()
	{
		return max_size_;
	}
	void push_back(T elem)
	{
		if(size() == max_size_)
			data_.erase(data_.begin());
		data_.push_back(elem);
	}

	T back()
	{
		return data_.back();
	}
	
	void pop_back()
	{
		data_.pop_back();
	}

	std::vector<T>& get_data()
	{
		return data_;
	} 
};

struct Data{
	std::vector<double> est;
	std::vector<double> cmd;

	Data(std::vector<double> e, std::vector<double> c)
	{
		est = e;
		cmd = c;
	}
};

class DataAssembler
{
private:
	BoundedVector<Data> lin_;
	BoundedVector<Data> ang_;
	std::string node_name_;
	std::string est_vel_topic_;
	std::string cmd_vel_topic_;
	std::string data_set_topic_;
	ros::NodeHandle* node_;
	ros::Publisher data_set_pub_;
	double smoothing_;

public:
	DataAssembler():
		node_name_("data_assembler"),
		est_vel_topic_("/estimated_vel_stamped"),
		cmd_vel_topic_("/cmd_vel_stamped"),
		data_set_topic_("/training_set"),
		smoothing_(2.0/31.0)
	{

	}

	DataAssembler(int max_size):
		node_name_("data_assembler"),
		est_vel_topic_("/estimated_vel_stamped"),
		cmd_vel_topic_("/cmd_vel_stamped"),
		data_set_topic_("/training_set"),
		smoothing_(2.0/(max_size+1)),
		lin_(max_size),
		ang_(max_size)
	{

	}

	void start()
	{
		node_ = new ros::NodeHandle("~");
		node_->param("cmd_vel_stamped", cmd_vel_topic_, cmd_vel_topic_);
		node_->param("est_vel_stamped", est_vel_topic_, est_vel_topic_);
		node_->param("training_set", data_set_topic_, data_set_topic_);
		data_set_pub_ = node_->advertise<adaptive_controller::DataSet>(data_set_topic_, 10);
		message_filters::Subscriber<geometry_msgs::TwistStamped> cmd_vel_sub(*node_, cmd_vel_topic_, 10);
		message_filters::Subscriber<geometry_msgs::TwistStamped> est_vel_sub(*node_, est_vel_topic_, 10);
		typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TwistStamped, geometry_msgs::TwistStamped> SyncPolicy;
		message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), cmd_vel_sub, est_vel_sub);
		sync.registerCallback(boost::bind(&DataAssembler::update, this, _1, _2));
		ros::Rate r(30);
		while(node_->ok())
		{
			ros::spinOnce();
			r.sleep();
		}
	}

	std::vector<double> smooth(const std::vector<double>& x, const std::vector<double>& s_prev)
	{	
		std::vector<double> s = std::vector<double>(x.size(), 0.0);
		for(int i = 0; i < x.size(); ++i)
		{
			s[i] = s_prev[i]*(1 - smoothing_) + x[i]*smoothing_; 
		}
		return s;
	}

	void update(const geometry_msgs::TwistStampedConstPtr& cmd_vel,
		const geometry_msgs::TwistStampedConstPtr& est_vel)
	{
		std::vector<double> est_vel_lin = getLinearVector(est_vel);
		est_vel_lin[0] = sqrt(est_vel_lin[0]*est_vel_lin[0] + est_vel_lin[1]*est_vel_lin[1]);
		est_vel_lin[1] = 0.0;
		est_vel_lin[2] = 0.0;
		std::vector<double> est_vel_ang = getAngularVector(est_vel);
		est_vel_ang[2] = fabs(est_vel_ang[2]);
		std::vector<double> cmd_vel_lin = getLinearVector(cmd_vel);
		cmd_vel_lin[0] = fabs(cmd_vel_lin[0]);
		std::vector<double> cmd_vel_ang = getAngularVector(cmd_vel);
		cmd_vel_ang[2] = fabs(cmd_vel_ang[2]);
		bool zero_ang_flag = false;
		bool zero_lin_flag = false;
		if(!isZeroVector(cmd_vel_lin))
		{
			if(!lin_.empty())
				est_vel_lin = smooth(est_vel_lin, lin_.back().est);
			lin_.push_back(Data(est_vel_lin, cmd_vel_lin));
		}
		else
			zero_lin_flag = true;
		if(!isZeroVector(cmd_vel_ang))
		{
			if(!ang_.empty())
				est_vel_ang = smooth(est_vel_ang, ang_.back().est);
			ang_.push_back(Data(est_vel_ang,cmd_vel_ang));
		}
		else
			zero_ang_flag = true;
		BoundedVector<Data> tmp;
		if(zero_lin_flag && zero_ang_flag)
		{
			return;	
		}
		if(zero_lin_flag)
		{
			data_set_pub_.publish(convertToDataSet(tmp, ang_));
			return;	
		}
		if(zero_ang_flag)
		{
			data_set_pub_.publish(convertToDataSet(lin_, tmp));
			return;	
		}
		data_set_pub_.publish(convertToDataSet(lin_, ang_));
	}

	adaptive_controller::DataSet convertToDataSet(BoundedVector<Data>& lin, BoundedVector<Data>& ang)
	{
		//assume that est_vel_.size() == control_.size()
		adaptive_controller::DataSet data_set;
		data_set.header.stamp = ros::Time::now();
		data_set.lin_row = 3;
		data_set.lin_col = lin.size();
		std::vector<Data> lin_data = lin.get_data();
		for(int i = 0; i < data_set.lin_col; ++i)
			for(int j = 0; j < data_set.lin_row; ++j)
			{
				data_set.est_lin.push_back(lin_data[i].est[j]);
				data_set.cmd_lin.push_back(lin_data[i].cmd[j]);
			}
		data_set.ang_row = 3;
		data_set.ang_col = ang.size();
		std::vector<Data> ang_data = ang.get_data();
		for(int i = 0; i < data_set.ang_col; ++i)
			for(int j = 0; j < data_set.ang_row; ++j)
			{
				data_set.est_ang.push_back(ang_data[i].est[j]);
				data_set.cmd_ang.push_back(ang_data[i].cmd[j]);
			}
		return data_set;
	}

	std::vector<double> getLinearVector(const geometry_msgs::TwistStampedConstPtr& vel)
	{
		std::vector<double> v;
		v.push_back(vel->twist.linear.x);
		v.push_back(vel->twist.linear.y);
		v.push_back(vel->twist.linear.z);
		return v;
	}

	std::vector<double> getAngularVector(const geometry_msgs::TwistStampedConstPtr& vel)
	{
		std::vector<double> v;
		v.push_back(vel->twist.angular.x);
		v.push_back(vel->twist.angular.y);
		v.push_back(vel->twist.angular.z);
		return v;
	}

	std::vector<double> TwistStampedToVector(const geometry_msgs::TwistStampedConstPtr& vel)
	{
		std::vector<double> v;
		v.push_back(vel->twist.linear.x);
		v.push_back(vel->twist.linear.y);
		v.push_back(vel->twist.linear.z);
		v.push_back(vel->twist.angular.x);
		v.push_back(vel->twist.angular.y);
		v.push_back(vel->twist.angular.z);
		return v;
	}

	bool isZeroVector(const std::vector<double>& vel)
	{
		if(vel == std::vector<double>(vel.size(), 0.0))
			return true;
		return false;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "data_set_collector");
	int size = 30;
	ros::param::param<int>("~size", size, 30);
	DataAssembler data_collector(size);
	data_collector.start();
	return 0;
}