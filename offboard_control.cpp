#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

#define Z_OFFSET 	5
#define FLIGHT_ALTITUDE -0.5f
#define RATE            20  // loop rate hz
#define RADIUS          3.0 // radius of X-Y circle
#define CYCLE_S         30   // time to complete one circle
#define STEPS           (CYCLE_S*RATE)

#define PI  3.14159265358979323846264338327950

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;


class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        	vehicle_command_listener_ = this->create_subscription<VehicleControlMode>("/fmu/out/vehicle_control_mode", qos, [this](const px4_msgs::msg::VehicleControlMode::UniquePtr msg){
            	c_mode = *msg;
            	//if(c_mode.flag_control_offboard_enabled==1){
            	//    printf("offboard away!!!");
            	// }
        });

		offboard_setpoint_counter_ = 0;



		InitPath();

		auto timer_callback = [this]() -> void {


			// offboard_control_mode needs to be paired with trajectory_setpoint

			
            double intpart;
            //printf("modf:%7.3f", );
            if(modf(((double)offboard_setpoint_counter_)/2, &intpart)==0.0){
                publish_offboard_control_mode();
                //printf("published mode msg");

            }
            //print(modf(offboard_setpoint_counter_/5, &intpart))
			publish_trajectory_setpoint();


		};
		timer_ = this->create_wall_timer(50ms, timer_callback);
	}


private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    	rclcpp::Subscription<VehicleControlMode>::SharedPtr vehicle_command_listener_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
    

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
        
    TrajectorySetpoint path[STEPS];
    VehicleControlMode c_mode;
        
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	//void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void InitPath();
};

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);

}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint()
{

        // Increment the setpoint counter
        offboard_setpoint_counter_++;
        if(offboard_setpoint_counter_>=STEPS){
		offboard_setpoint_counter_ = 0;
	}
        
	path[offboard_setpoint_counter_].timestamp = this->get_clock()->now().nanoseconds() / 1000;
	
    	if(c_mode.flag_control_offboard_enabled==1){
    		trajectory_setpoint_publisher_->publish(path[offboard_setpoint_counter_]);
    		printf("x:%7.3f y:%7.3f z: %7.3f yaw:%7.1f\n", path[offboard_setpoint_counter_].position[0], 				
    		path[offboard_setpoint_counter_].position[1],path[offboard_setpoint_counter_].position[2], 		
    		path[offboard_setpoint_counter_].yaw*180.0f/PI);
    	}
    	else{
        	offboard_setpoint_counter_=0;
    	}


}

void OffboardControl::InitPath()
{
// generate a path following Bernoulli's lemiscate as a parametric equation
// note this is in ENU coordinates since mavros will convert to NED
// x right, y forward, z up.
    int i;
    const double dt = 1.0/RATE;
    const double dadt = (2.0*PI)/CYCLE_S; // first derivative of angle with respect to time
    const double r = RADIUS;

    for(i=0;i<STEPS;i++){
        // basic fields in the message
        //path[i].coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        //path[i].type_mask = 0;

        // calculate the parameter a which is an angle sweeping from -pi/2 to 3pi/2
        // through the curve
        double a = (-PI/2.0) + i*(2.0*PI/STEPS);
	
        // Position
        path[i].position[0] = r*cos(a);
        path[i].position[1] = r*sin(a);
        path[i].position[2] = FLIGHT_ALTITUDE*sin(4.0*a) - Z_OFFSET;

        // Velocity
        path[i].velocity[0] =  -dadt*r* sin(a);
        if(path[i].velocity[0]>5)
        	path[i].velocity[0]=5;
        if(path[i].velocity[0]<-5)
        	path[i].velocity[0]=-5;

        path[i].velocity[1] =   dadt*r* cos(a);
        if(path[i].velocity[1]>5)
        	path[i].velocity[0]=5;
        if(path[i].velocity[1]<-5)
        	path[i].velocity[0]=-5;

        path[i].velocity[2] = 4.0*dadt*FLIGHT_ALTITUDE*cos(4.0*a);
        if(path[i].velocity[2]>5)
        	path[i].velocity[0]= 5;
        if(path[i].velocity[2]<-5)
        	path[i].velocity[0]=-5;

        // Acceleration
        path[i].acceleration[0] =  -r*dadt*dadt*cos(a);
        path[i].acceleration[1] =  -r*dadt*dadt*sin(a);
        path[i].acceleration[2] =  -16*dadt*dadt*FLIGHT_ALTITUDE*sin(4.0*a);
	/*if(path[i].acceleration[2]>1)
        	path[i].velocity[0]=1;
        if(path[i].acceleration[2]<-1)
        	path[i].velocity[0]=-1;*/
	
        // calculate yaw as direction of velocity
        // plus pi/2 since ROS yaw=0 lines up with x axis with points out to
        // the right, not forward along y axis
        //path[i].yaw = atan2(-path[i].velocity[0], path[i].velocity[1]) + (PI/2.0f);
	path[i].yaw = atan2(-path[i].velocity[0], path[i].velocity[1]) + PI;
	
        printf("x:%7.3f y:%7.3f z: %7.3f yaw:%7.1f\n", path[i].position[0], path[i].position[1], path[i].position[2], 
        path[i].yaw*180.0f/PI);
    }

    // calculate yaw_rate by dirty differentiating yaw
    for(i=0;i<STEPS;i++){
        double next = path[(i+1)%STEPS].yaw;
        double curr = path[i].yaw;
        // account for wrap around +- PI
        if((next-curr) < -PI) next+=(2.0*PI);
        if((next-curr) >  PI) next-=(2.0*PI);
        path[i].yawspeed = (next-curr)/dt;
    }

}


int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
