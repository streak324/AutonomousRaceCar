import rosbag
import numpy

#this code is specific for the "sparkfun 9dof razor imu sen-14001" device.
#could work with other IMUs by making minor changes to the code
#kinematic equations. have to do something a little different since acceleration won't be constant
#v = a*t + v0
#p = v0*t + 0.5 * a^2
#p = t/2 * (v + v0)

bag = rosbag.Bag('new_hallway.bag');
imu_sample_rate = 0.02;#in seconds
nanosecs_to_secs = 1000000000.0;
imu_data = [];
msg_count = 0;
begin_time = 0.0;
prev_time = 0.0;
curr_vel = numpy.array([0.0, 0.0, 0.0]);

def update_velocity(vel, accel, secs_elapsed):
    return vel + accel * secs_elapsed;

for topic, msg, t in bag.read_messages(topics=['/imu']):
    curr_time = t.secs + t.nsecs/nanosecs_to_secs;
    if(msg_count is 0):
        begin_time = curr_time;
        prev_time = begin_time;
    delta_time = curr_time - prev_time;

    accel = numpy.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]);
    curr_vel = update_velocity(curr_vel, accel, delta_time);
    print("time: {0}, acceleraton:{1}, velocity:{2}".format(curr_time - begin_time, accel, curr_vel));

    msg_count += 1;
    prev_time = curr_time;
print (msg_count);
