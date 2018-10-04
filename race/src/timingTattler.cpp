#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <stdlib.h>

class timingTattler
{
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber sub_proc;
  std::vector<double> diff;
  double start_time, prev_time;

  void callback_proc(const sensor_msgs::Image& msg)
  {
    double time = msg.header.stamp.toSec();
    if (start_time==0) {
        start_time = time;
        prev_time = start_time;
        return;
    }
    else if (time - start_time > 60) {
        getAverageProcessingTime();
        ros::shutdown();
        return;
    } 
    diff.push_back(time-prev_time);
    prev_time = time;
    //printf("Saved value\n");
    return;
  }

  public:
  void getAverageProcessingTime() 
  { 
    double sum = 0;
    for (int i =0;i<diff.size();i++) {
        sum += diff[i];
    } 
    double avg = sum / diff.size();
    printf("Average elapsed time was %f sec (data from 1 min of input)\n",avg);
  }

  timingTattler():nh_(), private_nh_("~")
  {
    start_time = 0;
    //sub_raw = nh_.subscribe("/zed/right/image_raw_color",10,&timingTattler::callback_raw,this);
    sub_proc = nh_.subscribe("/right_reconstruct",10,&timingTattler::callback_proc,this);
  }
  
};


int main(int argc, char** argv)
{
    ros::init(argc,argv,"timer");
    timingTattler t;
    while (ros::ok()) ros::spinOnce();
    return 0;
}


