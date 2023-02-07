#include <ros/ros.h>
#include <prophesee_event_msgs/Event.h>
#include <prophesee_event_msgs/EventArray.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <iomanip>
using namespace cv;

cv::Mat log_intensity_state_;
cv::Mat ts_array_;

double t_next_publish_;
double t_next_log_intensity_update_;

bool initialised_;

double contrast_threshold_on_user_defined_=0.1;
double contrast_threshold_off_user_defined_=-0.1;

double cutoff_frequency_global_=10*2*M_PI;

double publish_framerate_=60;
double intensity_min_user_defined_=-0.5;
double intensity_max_user_defined_=1.5;
int image_count = 0;

double previous_ts=0;

// std::ofstream myfile;
std::string path_of_events="./new_file.txt";

//ofstream events_output_txt_;
void display() {
  // black and white video
  if (image_count>300)
  {
	  return;
  }
  cv::Mat image;
  cv::exp(log_intensity_state_, image);
  double minVal = 1.4;
  double maxVal = 0.4;
  // minMaxLoc(image, &minVal, &maxVal);
  image = (image - minVal) / (maxVal - minVal);

  cv::Mat img(image.rows, image.cols, CV_64FC1, (char *)image.data);
  img.convertTo(img, CV_8U, 255.0 / 1.0);
  cv::Mat cimg;
  cv::cvtColor(img, cimg, cv::COLOR_GRAY2RGB);

  // save image
  char image_name[100];
  int n;
  n = sprintf(image_name, "./image%05d.png", image_count);
  cv::imwrite(image_name, cimg);

  cv::waitKey(1);
  image_count += 1;
}





void initialise_image_states(const uint32_t& rows, const uint32_t& columns)
{
    //先将上面的参数进行初始化
    log_intensity_state_=cv::Mat::zeros(rows,columns,CV_64FC1);
    ts_array_=cv::Mat::zeros(rows,columns,CV_64FC1);
    t_next_publish_=0.0;
    t_next_log_intensity_update_=0.0;
    initialised_=true;
    //下面那个输出的语句先不要写
}
//上面这一块还缺少一个引入

//update log intensity state 就完毕了
void update_log_intensity_state(const double& ts, const int& x, const int& y, const bool& polarity)
{
    //先计算delta 这是个局部变量
    const double delta_t=(ts-ts_array_.at<double>(y,x));
    double contrast_threshold;
    //这两个也是局部变量
    //判断时间是否小于0
    //下面这一句也有可能报错
    if (delta_t<0)
    {
        ROS_INFO("This is the Local Warning: non-monotonic timestamp detected, resetting...");
        //然后再重新输入
        initialise_image_states(log_intensity_state_.rows, log_intensity_state_.cols);
        return;
    }
    //这里也有全局变量
    contrast_threshold=(polarity)? contrast_threshold_on_user_defined_: contrast_threshold_off_user_defined_;
    log_intensity_state_.at<double>(y,x)=std::exp(-cutoff_frequency_global_*delta_t)*log_intensity_state_.at<double>(y,x)+contrast_threshold;
    ts_array_.at<double>(y,x)=ts;
}

//下面这个是全局更新
void update_log_intensity_state_global(const double& ts)
{
    cv::Mat beta;
    //临时变量
    cv::Mat delta_t=ts-ts_array_;
    double min;
    cv::minMaxLoc(delta_t, &min,nullptr);
    //这里如果我们不进行保存 初始化设置 得到的图片是否是好的
    if(min<0)
    {
        ROS_INFO("This is the Global Warning: non_monotonic timestamp detected, resetting...");
        //跟上面一样 也有可能报错
        initialise_image_states(log_intensity_state_.rows, log_intensity_state_.cols);
        return;
    }
    //这里换成了opencv的exp
    cv::exp(-cutoff_frequency_global_*(ts-ts_array_),beta);
    log_intensity_state_=log_intensity_state_.mul(beta);
    ts_array_.setTo(ts);
}




void EventCallBack(const prophesee_event_msgs::EventArray::ConstPtr &msg){
    if(!initialised_)
    {
	    initialise_image_states(msg->height,msg->width);
    }

	if (msg->events.size()>0)
    {
        for(int i=0;i<msg->events.size();++i)
        {
            const int x=msg->events[i].x;
            const int y=msg->events[i].y;
            if (x<msg->width && x>0 && y<msg->height && y>0)
            {
                const double ts=msg->events[i].ts.toSec();
                const bool polarity=msg->events[i].polarity;
                // myfile<<ts<<std::endl;

                //越新的事件ts肯定越来越大 越旧的事件ts肯定很小
                //ROS_INFO("The current ts is %.12f\n",ts);
                //ROS_INFO("The previous ts is %.12f\n",previous_ts);
                // if (ts-previous_ts<0)
                // {
                //     ROS_INFO("ts is %f",ts);
                //     ROS_INFO("previous ts is %f",previous_ts);
                //     ROS_INFO("The time gap between the current ts and the previosu one is %f",ts-previous_ts);
                //     ROS_INFO("The events are definitely not monotonic");
                // }
            	update_log_intensity_state(ts,x,y,polarity);
		//std::string output_event_path;
		//output_event_path += "~/catkin_e_ws/result.txt";
		//events_output_txt_.open(output_event_path);
		//std::cout << "Save filtered event data to: " << output_event_path << "\n";
		//events_output_txt_ << ts << " "  << x << " "  << y << " "  << " " << polarity << "\n";
                if(publish_framerate_>0 && ts>t_next_publish_)
                {
                    update_log_intensity_state_global(ts);
                    //ROS_INFO("this is being executed!");
                    display();
                    t_next_publish_=ts+1/publish_framerate_;
                }
	        }
            //print all it out
            
        } 
	const double ts=msg->events.back().ts.toSec();
	// if(publish_framerate_<0)
	// {
	// 	update_log_intensity_state_global(ts);
		// display();
	// }
    }
}

int main(int argc, char **argv){
    ros::init(argc,argv,"EventSub");
    ros::NodeHandle nh;
    // myfile.open(path_of_events);
    // myfile.setf(std::ios::fixed,std::ios::floatfield);
    // myfile.precision(12);

    ros::Subscriber topic_sub=nh.subscribe("/prophesee/camera/cd_events_buffer",10000,EventCallBack);
    ros::spin();
    return 0;

}
