#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <math.h>
#define square(x) x*x
#define THRESHOLD 70

struct polar_point { int roh; int theta; };

class zedBW
{
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher pub_r_,pub_l_;
  ros::Subscriber sub_r_,sub_l_;

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
   
    std::vector <unsigned char> scaled; //to scale down image

    for (i=0;i<newH;i++) {
        for (j=0;j<newW*3;j++) {
            long sum=0;
            //sum += scaled[3*newW*i + 3*j +k];
            sum=sum+msg.data[(2*j)   + 3*W*(2*i)  ];
            sum=sum+msg.data[(2*j+3) + 3*W*(2*i)  ];
            sum=sum+msg.data[(2*j)   + 3*W*(2*i+1)];
            sum=sum+msg.data[(2*j+3) + 3*W*(2*i+1)];
            scaled.push_back(sum/4);
        }
    }
    
    std::vector <unsigned char> bw; // to hold raw bw
    
    /* Consolidate color into black and white photo */
    for (i=0;i<newH;i++) {
        for (j=0;j<newW;j++) {
            long sum = 0;
            int k;
            for (k=-1;k<2;k++) sum += scaled[3*newW*i + 3*j +k];
            bw.push_back(sum/3);
        }
    }
    
    std::vector <double> edges; // to hold sgm 
    /* Calculate dx, dy, sgm */
    int dx,dy;
    double max = 0;
    for (i=0;i<newH;i++) {
        if (i==0 || i==(newH-1)) { // top/bottom row
            for (j=0;j<newW;j++) edges.push_back(0);
        }
        else {
            edges.push_back(0); // left column
            for (j=1;j<newW-1;j++) {
                int index = newW*i + j; 
                dx = bw[index+newW+1] + 2*bw[index+1] + bw[index-newW+1] \
                - (bw[index+newW-1] + 2*bw[index-1] + bw[index-newW-1]);
                dy = bw[index-newW-1] + 2*bw[index-newW] + bw[index-newW+1]\
                - (bw[index+newW-1] + 2*bw[index+newW] + bw[index+newW+1]);
                double sgm = sqrt(square(dx) + square(dy));
                max = (max<sgm)? sgm: max;
                edges.push_back(sgm);
            }
            edges.push_back(0); // right column
        }
    }
    bw.clear();

    std::vector <unsigned char> edge_img;
    /* Scale edge (sgm) data to 0 to 255 range */
    for (i=0;i<edges.size();i++) edge_img.push_back((edges[i]/max*255 > THRESHOLD)?255 :0);
    edges.clear();

    edge.data.swap(edge_img);
    
    edge_img.clear();   
     
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
  zedBW():nh_(), private_nh_("~")
  {
    pub_r_ = nh_.advertise<sensor_msgs::Image>("right_edges",10);
    //pub_l_ = nh_.advertise<sensor_msgs::Image>("left_edges",10);
    sub_r_ = nh_.subscribe("/zed/right/image_raw_color",10,&zedBW::callback_r,this);
    //sub_l_ = nh_.subscribe("/zed/left/image_raw_color",10,&zedBW::callback_l,this);
  }
  
};

int main(int argc, char** argv)
{
    ros::init(argc,argv,"zed_bw");
    zedBW z;
    ros::spin();
    return 0;
}


