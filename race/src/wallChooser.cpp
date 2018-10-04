#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Int32.h>
#include <math.h>

#define NUM_NODES 5

double car_x = 0, car_y = 0;
const int default_val = 1;
int direction, currentNode;
int in_threshold = 10;
int out_threshold = 1;
bool side_published = false;
ros::Publisher side_pub;

double nodes[NUM_NODES][2][4] = { {{0.5,-.2},{0,1,1,0}},
                    {{21,0},{0,1,1,1}}, 
                    {{29,0},{0,0,1,1}}, 
                    {{29.5,-17.9},{1,0,0,1}}, 
                    {{1,-19},{1,1,0,0}}};
 
void callback(const geometry_msgs::PoseWithCovarianceStamped msg);
void do_stuff(void);
void set_side(int);
int find_side(void);
int calculateSide(int, int, int);
int findOut(int);
int findIn(int);
int giveDirection(double[], double[]);
int distance_from_node(void);
int distance(double,double,double,double);


int main(int argc, char** argv){
  ros::init(argc, argv, "side_control");

  ros::NodeHandle n;
  ros::Publisher side_pub = n.advertise<std_msgs::Int32>("side", 1);
  ros::Subscriber pose_sub = n.subscribe("amcl_pose",1, callback);

  std::string int_str;
  int initial_side; 
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
  if (!n.hasParam("/initial_side")) {
    initial_side = -1;
  } else { 
    n.getParam("/initial_side",int_str);
    sscanf(int_str.c_str(), "%d", &initial_side);
  }

  if (direction==-1) {
    //NEED TO FIX IF WE WANT TO GO IN OPPOSITE
    //nodes.reverse();
    currentNode = NUM_NODES-1-currentNode;
  }

  set_side(initial_side);
  ros::spin();
}

void callback(const geometry_msgs::PoseWithCovarianceStamped msg) {
  double x = msg.pose.pose.position.x;
  double y = msg.pose.pose.position.y;
  if (abs(x-car_x)>=1 || abs(y-car_y)>=1) {
        ROS_INFO("X: %f, Y: %f; Dist: %d, CurrNode: %d",car_x,car_y,distance_from_node(),currentNode); 
        car_x = x;
        car_y = y;
        do_stuff();  
  }
}

void do_stuff(void) {
  //Functionality: function called every time (x,y) is updated noticeably     
  int current_dist=distance_from_node();      //Distance from the current node (for checking thresholds)
  if (current_dist>=0) {                       //If 'entering' current node
        if ((current_dist<in_threshold) && !side_published) {
                side_published = true;
                set_side(find_side());  //Publish side
        }
  }
  else if (current_dist<0){                                          //If 'exiting' current node
        if (current_dist<-1*out_threshold){                           //If car has fully exited node's range
            side_published = false;           
            currentNode=(currentNode+1)%NUM_NODES;                         //Update node to the next node
            ROS_INFO("Set current node to %d",currentNode); 
            do_stuff(); 
        }
  }
}

int calculateSide(int in_dir, int out_dir, int current_node_index) {
  //Input: 'in', 'out' directions (0,1,2,3) and current node index (from list 'nodes')
  //Output: which side to follow (-1 for left, 1 for right)
  if (abs(in_dir-out_dir)==2){ //If car will go straight through node ('in' and 'out' are opposite to each other)
        if (nodes[current_node_index][1][(in_dir-1)%4]==1)             //If a right turn is possible
            return -1;                                                  //Follow left wall
        else if (nodes[current_node_index][1][(in_dir+1)%4]==1)           //If a left turn is possible
            return 1;                                                      //Follow right wall
  }
  else {                                                           //Else car will turn at this node
        if ((in_dir-out_dir)%4==1)                                     //If turn is a right turn (in is 1 more than out)
            return 1;                                                        //Follow right wall
        else                                                          //Else left turn
            return -1;                                                       //Follow left wall
  }
}

int findOut(int current_node_index) {
//Input: Current node index (from list 'nodes')
//Output: out direction (0,1,2,3)
  double x,y;
  x = nodes[current_node_index][0][0];
  y = nodes[current_node_index][0][1];
  double p1[] = {x,y};
  int i = (current_node_index+1)%NUM_NODES;
  x = nodes[i][0][0];
  y = nodes[i][0][0];
  double p2[] = {x,y}; 
  //double p1[] = nodes[current_node_index][0];                                 //(x,y) position for current node
 //double p2[] = nodes[(current_node_index+1)%NUM_NODES][0];                  //(x,y) position for next node
  return giveDirection(p1,p2);                                     //compute direction 
}

int findIn(int current_node_index) {
   //Input: and the index of the incoming node (from list 'nodes')
  //Output: direction the car will enter the node (0,1,2,3)
  double x,y;
  int i = current_node_index;
  x = nodes[i][0][0];
  y = nodes[i][0][1];
  double p1[] = {x,y};
  i = (current_node_index+1)%NUM_NODES;
  x = nodes[i][0][0];
  y = nodes[i][0][1];
  double p2[] = {x,y};

   //double p1[] = nodes[(current_node_index-1)%NUM_NODES][0];                                 //(x,y) position for current node
   //double p2[] = nodes[current_node_index][0];                  //(x,y) position for next node
  return giveDirection(p2,p1);                                     //compute direction 
}
    
int giveDirection(double p1[], double p2[]) {
  //Input: p1=(x1,y1) and p2=(x2,y2)
  //Output: Direction exiting p1 if going to p2 (0,1,2,3)
    double dx=p2[0]-p1[0];                                                  //Change in x values (x2-x1)
    double dy=p2[1]-p1[1];                                                  //Change in y values (y2-y1)
    
    if (abs(dx)>abs(dy)){                                           //If traveling in the x direction (assuming either x or y is relevant, not both)
        if (dx>0)                                                      //If moving in positive x direction
            return 1;                                                        //Output 'right'
        else                                                           //If moving in negative x direction
            return 3;                                                        //Output 'left'
    }else{                                                          //If traveling in the y direction
        if (dy>0)                                                      //If moving in positive y direction
            return 0;                                                       //Output 'up'
        else                                                          //If moving in negative y direction
            return 2;                                                        //Output 'down'
         }
}

int distance_from_node() {
  //Output: distance of car from current node as an integer (negative value means it passed the current node)
  /*double curr[] = nodes[currentNode][0]; //Current node's position on map
  double x = curr[0];
  double y = curr[1];
  double goal[] = nodes[(currentNode+1)%NUM_NODES][0]; //Next node (x,y) to see if car has passed the current node or not
  double x_next = goal[0];
  double y_next = goal[1]; */

  double x = nodes[currentNode][0][0];
  double y = nodes[currentNode][0][1];

  int i = (currentNode + 1)%NUM_NODES;

  double x_next = nodes[i][0][0];
  double y_next = nodes[i][0][1]; 

  int sign = (distance(x,y,x_next,y_next) > distance(car_x,car_y,x_next,y_next))?-1:1; 
  //int sign = (distance((x,y),(x_next,y_next))>distance((car_x,car_y),(x_next,y_next)))? -1 : 1;
      //Finds distance's sign; negative means car has passed node (car is closer to next node than current node is)
  return sign*distance(car_x,car_y,x,y);      //Integer distance from car to node
}
        
int distance(double x1, double y1, double x2, double y2) {
  //Functionality: Distance formula calculator
  return (int) sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)); 
  //return (int) sqrt((p2[1]-p1[1])*(p2[1]-p1[1])+(p2[0]-p1[0])*(p2[0]-p1[0]));
}

void set_side(int s) {
//Input: Integer 's' that is 1 for right, -1 for left
//Functionality: Sets 'side' variable to 's'
    std_msgs::Int32 msg; //Message of type Int32, which will be published
    msg.data = s;       //Sets message data to 's'
    ROS_INFO("set side to %d at %f, %f", s, car_x, car_y);
    side_pub.publish(msg);  //Publishes message to 'side' variable
}

int find_side(void) {
//Functionality: generates current side to follow using earlier functions
    return calculateSide(findIn(currentNode), 
                         findOut(currentNode),currentNode);
    //Calculate needed side by calling in/out functions on current node
}
