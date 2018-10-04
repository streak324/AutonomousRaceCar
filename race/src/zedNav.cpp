#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <math.h>
#define square(x) x*x
#define THRESHOLD 70

#include "zedMagic.h"
//extern "C" int testMain();

struct polar_point { int roh; int theta; };

class zedNav
{
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher pub_r_,pub_l_;
  ros::Subscriber sub_r_,sub_l_;

  int process_type;

  //START NEW STUFF
  int counter;
  int frameSkip;
  std::vector<unsigned char>  publishedImage;
  //END NEW STUFF*/

  void callback(const sensor_msgs::Image& msg, sensor_msgs::Image& edge)
  {
    int i,j;
    int W = msg.width, H = msg.height;
    int cropH=H*55/100;
    int newH=cropH/2;
    int newW=W/2;

    edge.header = msg.header;
    edge.height = newH;
    edge.width = newW;
    edge.encoding = "mono8"; // now single color (B/W)
    edge.is_bigendian = msg.is_bigendian;
    edge.step = msg.step/3/2; // only have 1/6 the data: bw, zoom

    //edge.data = processImage(msg.data, process_type);

    if (counter%frameSkip==0) {
        publishedImage=processImage(msg.data,process_type);
    }
    counter++;
    edge.data=publishedImage;

    edge.header.stamp = ros::Time::now();
    //printf("Took %f seconds to process\n", edge.header.stamp.toSec() - msg.header.stamp.toSec());

    return;
     
    /*
    int hough_max = 0;
    int final_r, final_t;
    for (theta=0;theta<180;theta++) {
        for (roh=roh_min;roh<roh_max+1;roh++) {
            if (count(voting,theta,roh)>hough_max) {
                hough_max = count(voting,theta,roh);
                final_r = roh;
                final_t = theta;
            }
        }
    }

    printf("r %d, t %d\n", roh, theta);

    for (i=0;i<newH;i++) {
        for (j=0;j<newW;j++) {
            int x = j;
            int y = newH-1-i;
            if (abs(x*sin(final_t*M_PI/180) - y*cos(final_t*M_PI/180) + final_r) < 1) edge.data.push_back(255);
            else edge.data.push_back(0);
        }
    }
    */

  }

  int count(std::vector <int> * voting, int theta, int roh) {
    int i,n;
    for (i=0;i<voting[theta].size();i++) if (voting[theta][i]==roh) n++;
    return n;
  }


  void callback_r(const sensor_msgs::Image& msg)
  {
    if (msg.encoding.compare("bgr8")) {
        printf("Incorrect Image format type (not currently able to handle other than 'bgr8')\n");
        return;
    }
    sensor_msgs::Image edge;
    callback(msg,edge); // do operations on data
    pub_r_.publish(edge); 
    
  }

  void callback_l(const sensor_msgs::Image& msg)
  {
    if (msg.encoding.compare("bgr8")) {
        printf("Incorrect Image format type (not currently able to handle other than 'bgr8')\n");
        return;
    }
    sensor_msgs::Image edge;
    callback(msg,edge); // do operations on data
    pub_l_.publish(edge); 
    
  }
  
  public:
  zedNav():nh_(), private_nh_("~")
  {
    pub_r_ = nh_.advertise<sensor_msgs::Image>("right_edges",10);
   // pub_l_ = nh_.advertise<sensor_msgs::Image>("left_edges",10);
    sub_r_ = nh_.subscribe("/zed/right/image_raw_color",10,&zedNav::callback_r,this);
    //sub_l_ = nh_.subscribe("/zed/left/image_raw_color",10,&zedNav::callback_l,this);
    process_type = CUDA_SMART;
    //counter=5; //
    frameSkip=2;//
    counter = frameSkip;
  }
  
};

int main(int argc, char** argv)
{
    ros::init(argc,argv,"zed_nav");
    zedNav z;
    ros::spin();
    return 0;
}


