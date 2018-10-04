#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <unistd.h> //unsigned int sleep(unsigned int seconds); usleep is in microseconds

#include <iostream>
using namespace std;

#include <string>


void brakePump();
int distance_from_node();
int distance(int p1_x, int p1_y, int p2_x, int p2_y);
void setSpeed(int s);
void setTurning(bool turn);
void do_stuff();
void callback(const geometry_msgs::PoseWithCovarianceStamped data);






bool brake_activated = false;
bool speed_up_activated = false;


int car_x=0;
int car_y=0;

int in_threshold=7;
int out_threshold=5;
double nodes[4][2][4] = { {{0.5,-.2},{0,1,1,0}}, {{29,0},{0,0,1,1}}, {{29.5,-17.9},{1,0,0,1}}, {{1,-19},{1,1,0,0}}};
int numOfNodes=4;
int in_threshold_turn=12;
int out_threshold_turn=2;
int direction;
int currentNode;
ros::Publisher em_pub;
ros::Publisher turn_pub;
ros::NodeHandle n;




int main(int argc, char** argv) {
	ros::init(argc, argv, "speed_control");

	std::string int_str;

	em_pub = n.advertise<std_msgs::Int32>("drive_velocity", 1);
	turn_pub = n.advertise<std_msgs::Bool>("is_turning", 1);
	if (!n.hasParam("direction")) {
		direction = 1;
	} else {
		n.getParam("direction",int_str);
		sscanf(int_str.c_str(),"%d",&direction);
	}
	if (!n.hasParam("current_node")) {
  		currentNode = 1;
	} else {
		n.getParam("current_node", int_str);
		sscanf(int_str.c_str(), "%d",&currentNode);
    	}
	setSpeed(0);
	usleep(5*1000*1000);
	setSpeed(23);	
	ros::Subscriber sub = n.subscribe("amcl_pose",1, callback);

	ros::spin();
}





void brakePump() {
	std_msgs::Int32 v_msg;
	cout << "Starting BRAKE PUMP at " << car_x << "," << car_y;
	for (int i=0;i<2;i++) {
		v_msg.data = -70;
		for (int j=0;j<11;j++) {
			v_msg.data = v_msg.data - 10;
			em_pub.publish(v_msg);
		}
		usleep(0.4*1000*1000);

		for (int j=0;j<20;j++) {
			v_msg.data = v_msg.data + 10;
			if (v_msg.data > 12) {
				v_msg.data = 12;
			}
			em_pub.publish(v_msg);
		}

		usleep(0.3*1000*1000);
	}
            
        cout << "BRAKES PUMPED at " << car_x << "," << car_y;
}

      
int distance_from_node() {
	int x, y, x_next,y_next,sign;
	x=nodes[currentNode][0][0];
	y=nodes[currentNode][0][1];
	x_next=nodes[(currentNode+1)%numOfNodes][0][0];
	y_next=nodes[(currentNode+1)%numOfNodes][0][1];
	if (distance(x,y,x_next,y_next)>distance(car_x,car_y,x_next,y_next)) {
		sign=-1;
	}
	else {
		sign=1;
	}
	return sign*distance(car_x,car_y,x,y);
}

int distance(int p1_x, int p1_y, int p2_x, int p2_y) {
    return (int)(sqrt((p2_y-p1_y)*(p2_y-p1_y)+(p2_x-p1_x)*(p2_x-p1_x)));
}







void setSpeed(int s){
	std_msgs::Int32 msg;
	msg.data = s;
	//print("set speed to ",s) 
	em_pub.publish(msg);
}


void setTurning(bool turn){
	std_msgs::Bool msg;
	msg.data = turn;	
	turn_pub.publish(msg);
	//print("Set 'turning' to ",turn)
}

void do_stuff() {
	int current_dist=distance_from_node();
	if (current_dist>=0) {
		/*if (current_dist<in_threshold) {
			setSpeed(12)
		}*/
		if (current_dist<in_threshold_turn) {
			if (!brake_activated) {
				brake_activated = true;
				speed_up_activated = false;
				brakePump();
				//in_threshold_turn = in_threshold_turn + 3;
				//cout << "Lag incrementing: in_threshold_turn at " << in_threshold_turn;
				setTurning(true);
			}
		}
	}
	else if (current_dist<0) {
		if (current_dist<-1*out_threshold) {
			currentNode=(currentNode+1)%numOfNodes;
			//print("Set current node to ",currentNode)
		}
		if (current_dist<-1*out_threshold_turn) {
			if (!speed_up_activated) {
				brake_activated = false; 
				speed_up_activated = true;
				setSpeed(23);
				setTurning(false);
			}
			//do_stuff();
		}
	}
}
 



void callback(const geometry_msgs::PoseWithCovarianceStamped data) {
	double x = data.pose.pose.position.x;
	double y = data.pose.pose.position.y;
	if (abs(x-car_x)>=1 || abs(y-car_y)>=1) {
		cout << "X: " << car_x << " << Y: " << car_y << "; Dist: " << distance_from_node() << ", CurrNode: " << currentNode; 
		car_x = x;
		car_y = y;
		do_stuff();
	}  
}
