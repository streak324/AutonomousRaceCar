#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <stdlib.h>
#include <cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include "zedMagic.h"

#define NORMAL
#define CUHOUGH

namespace cv
{
    using std::vector;
}

class lineDetector 
{
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber sub;
  ros::Publisher pub;
  ros::Publisher is_side;

  void hough(const sensor_msgs::Image& msg, sensor_msgs::Image& edge) {
    int i,j;
    int count;
 
    edge.header = msg.header;
    edge.height = H_O;
    edge.width = W_O;
    edge.is_bigendian = msg.is_bigendian;

#ifdef NORMAL
    cv::vector<cv::Vec2f> lines;
#else
    cv::vector<cv::Vec4i> lines;
#endif
#ifdef CUHOUGH 
    edge.encoding = "mono8";
    edge.step = msg.step;
#else
    edge.encoding = "bgr8";
    edge.step = msg.step*3;
#endif
    cv::Mat dst(H_O,W_O,CV_8UC1);
    std::memcpy(dst.data, msg.data.data(), msg.data.size()*sizeof(unsigned char));
    //cv::Mat dst(msg.data);
    cv::Mat cdst;
    cv::cvtColor(dst,cdst,cv::COLOR_GRAY2BGR);
    //cv::HoughLines(dst,lines,10,CV_PI/180,80,0,0);

#ifdef NORMAL
#ifdef CUHOUGH
    unsigned char *d_edge, *a_edge = (unsigned char*) malloc(sizeof(unsigned char)*W_O*H_O);
    for (int index=0;index<W_O*H_O;index++) a_edge[index] = msg.data[index]/2;
    cudaMalloc(&d_edge,W_O*H_O*sizeof(unsigned char));
    cudaMemcpy(d_edge,a_edge,W_O*H_O*sizeof(unsigned char),cudaMemcpyHostToDevice);
    cv::HoughLines(dst,lines,1,5*CV_PI/180,180,0,0);
    printf("Lines size: %d", (int)lines.size());
    for (size_t i = 0; i< lines.size(); i++) {
        float roh = lines[i][0], theta = lines[i][1];
        if (false and not (abs(theta*180/CV_PI)>150 or abs(theta*180/CV_PI)<30)) continue;
        count++;
        drawMath(theta,roh,d_edge);
        //printf("(%frad,%f)\n",theta,roh);
    }
    cudaMemcpy(a_edge,d_edge,W_O*H_O*sizeof(unsigned char),cudaMemcpyDeviceToHost);
    for (int index=0;index<W_O*H_O;index++) edge.data.push_back(a_edge[index]);
    printf("CUDA NORMAL \n");
    free(a_edge);
    cudaFree(d_edge);
#endif
#endif
#ifdef NORMAL
#ifndef CUHOUGH
    cv::HoughLines(dst,lines,15,5*CV_PI/180,480,0,0);
    for (size_t i = 0; i< lines.size(); i++) {
        float roh = lines[i][0], theta = lines[i][1];
        if (not (abs(theta*180/CV_PI)>150 or abs(theta*180/CV_PI)<30)) continue;
        cv::Point pt1,pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*roh, y0 = b*roh;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        cv::line(cdst,pt1,pt2,cv::Scalar(0,0,255),3,3);
        count++;
    }
 
    std::vector<unsigned char> edge_r;
    if (cdst.isContinuous()) {
        edge_r.assign(cdst.datastart, cdst.dataend);
    } else {
        for (int i = 0;i < cdst.rows; ++i) {
            edge_r.insert(edge_r.end(), cdst.ptr<unsigned char>(i), cdst.ptr<unsigned char>(i) + cdst.cols);
        }
    }
    edge.data = edge_r;
    printf("LInes size: %d;",lines.size()); 
    printf("PLAIN NORMAL\n");
#endif
#endif
#ifndef NORMAL
    cv::HoughLinesP(dst,lines,1,CV_PI/180,200,50,10);
    for (size_t i = 0; i < lines.size(); i++) { 
        cv::Vec4i l = lines[i];
        cv::Point pt1 (l[0],l[1]);
        cv::Point pt2 (l[2],l[3]);
        float theta = atan((pt2.y-pt1.y)/(pt2.x-pt1.x));
        if (not (abs(theta*180/CV_PI)>150 or abs(theta*180/CV_PI)<30)) continue;
        count++;
        cv::line(cdst,pt1,pt2,cv::Scalar(0,0,255),3,3);
    }
    printf("LInes size: %d",lines.size()); 
    std::vector<unsigned char> edge_r;
    if (cdst.isContinuous()) {
        edge_r.assign(cdst.datastart, cdst.dataend);
    } else {
        for (int i = 0;i < cdst.rows; ++i) {
            edge_r.insert(edge_r.end(), cdst.ptr<unsigned char>(i), cdst.ptr<unsigned char>(i) + cdst.cols);
        }
    }
    edge.data = edge_r;
    printf("PROBABILITY\n");
#endif

    //bool msg_data = count>0;
    std_msgs::Bool message;
    message.data = (count>0);
    is_side.publish(message);

    return;

  }

  void callback(const sensor_msgs::Image& msg) {
    if (msg.encoding.compare("mono8")) {
        printf("Incorrect Image format type (not currently able to handle other than 'mono8')\n");
        return;
    }
    sensor_msgs::Image edge;
    hough(msg,edge); // do operations on data
    pub.publish(edge); 
 }

public:
  lineDetector():nh_(), private_nh_("~")
  {
    sub = nh_.subscribe("/right_edges",10,&lineDetector::callback,this);
    pub = nh_.advertise<sensor_msgs::Image>("/right_reconstruct",10);
    is_side = nh_.advertise<std_msgs::Bool>("/sees_side_wall",10);
  }
  
};


int main(int argc, char** argv)
{
    ros::init(argc,argv,"line_detector");
    lineDetector l;
    ros::spin(); 
    return 0;
}


