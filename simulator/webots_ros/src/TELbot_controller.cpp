/*
 * Author: Majd Kassawat
 * Email: majd.kassawat@gmail.com
 */
#include "ros/ros.h"
#include <math.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <cstdlib>


#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Illuminance.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Range.h>
#include <signal.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8MultiArray.h>
#include <webots_ros/BoolStamped.h>
#include <webots_ros/Float64Stamped.h>
#include <webots_ros/Int32Stamped.h>
#include <webots_ros/Int8Stamped.h>
#include <webots_ros/RadarTarget.h>
#include <webots_ros/RecognitionObject.h>
#include <webots_ros/StringStamped.h>

#include <webots_ros/get_bool.h>
#include <webots_ros/get_float.h>
#include <webots_ros/get_int.h>
#include <webots_ros/get_string.h>
#include <webots_ros/get_uint64.h>
#include <webots_ros/set_bool.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_float_array.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_string.h>

#include <webots_ros/camera_get_focus_info.h>
#include <webots_ros/camera_get_info.h>
#include <webots_ros/camera_get_zoom_info.h>
#include <webots_ros/display_draw_line.h>
#include <webots_ros/display_draw_oval.h>
#include <webots_ros/display_draw_pixel.h>
#include <webots_ros/display_draw_polygon.h>
#include <webots_ros/display_draw_rectangle.h>
#include <webots_ros/display_draw_text.h>
#include <webots_ros/display_get_info.h>
#include <webots_ros/display_image_copy.h>
#include <webots_ros/display_image_delete.h>
#include <webots_ros/display_image_load.h>
#include <webots_ros/display_image_new.h>
#include <webots_ros/display_image_paste.h>
#include <webots_ros/display_image_save.h>
#include <webots_ros/display_set_font.h>
#include <webots_ros/field_get_bool.h>
#include <webots_ros/field_get_color.h>
#include <webots_ros/field_get_count.h>
#include <webots_ros/field_get_float.h>
#include <webots_ros/field_get_int32.h>
#include <webots_ros/field_get_node.h>
#include <webots_ros/field_get_rotation.h>
#include <webots_ros/field_get_string.h>
#include <webots_ros/field_get_type.h>
#include <webots_ros/field_get_type_name.h>
#include <webots_ros/field_get_vec2f.h>
#include <webots_ros/field_get_vec3f.h>
#include <webots_ros/field_import_node.h>
#include <webots_ros/field_remove.h>
#include <webots_ros/field_set_bool.h>
#include <webots_ros/field_set_color.h>
#include <webots_ros/field_set_float.h>
#include <webots_ros/field_set_int32.h>
#include <webots_ros/field_set_rotation.h>
#include <webots_ros/field_set_string.h>
#include <webots_ros/field_set_vec2f.h>
#include <webots_ros/field_set_vec3f.h>
#include <webots_ros/lidar_get_frequency_info.h>
#include <webots_ros/lidar_get_info.h>
#include <webots_ros/motor_set_control_pid.h>
#include <webots_ros/node_get_center_of_mass.h>
#include <webots_ros/node_get_contact_point.h>
#include <webots_ros/node_get_field.h>
#include <webots_ros/node_get_id.h>
#include <webots_ros/node_get_name.h>
#include <webots_ros/node_get_number_of_contact_points.h>
#include <webots_ros/node_get_orientation.h>
#include <webots_ros/node_get_parent_node.h>
#include <webots_ros/node_get_position.h>
#include <webots_ros/node_get_static_balance.h>
#include <webots_ros/node_get_status.h>
#include <webots_ros/node_get_type.h>
#include <webots_ros/node_get_velocity.h>
#include <webots_ros/node_remove.h>
#include <webots_ros/node_reset_functions.h>
#include <webots_ros/node_set_velocity.h>
#include <webots_ros/node_set_visibility.h>
#include <webots_ros/pen_set_ink_color.h>
#include <webots_ros/range_finder_get_info.h>
#include <webots_ros/receiver_get_emitter_direction.h>
#include <webots_ros/robot_get_device_list.h>
#include <webots_ros/robot_set_mode.h>
#include <webots_ros/robot_wait_for_user_input_event.h>
#include <webots_ros/save_image.h>
#include <webots_ros/speaker_play_sound.h>
#include <webots_ros/speaker_speak.h>
#include <webots_ros/supervisor_get_from_def.h>
#include <webots_ros/supervisor_get_from_id.h>
#include <webots_ros/supervisor_movie_start_recording.h>
#include <webots_ros/supervisor_set_label.h>
#include <webots_ros/supervisor_virtual_reality_headset_get_orientation.h>
#include <webots_ros/supervisor_virtual_reality_headset_get_position.h>
#define TIME_STEP 32
#define MAX_SPEED 2
#define PI 3.14159265

//Defining cases as integers 
#define _Start_ 0
#define _MoveToTarget_ 1
#define _TouchingTarget_ 2
#define _FixingOrientation_ 3
#define _Synchronize_ 4
#define _Lifting_ 5
#define _MaintainAndSync_ 6
#define _MaintainAndTransport_ 7
#define _Exit_ 10

using namespace std;

static int model_count;
static vector<string> model_list;

string ModelName = "TELbot_1_2901_majd_Aspire_TC_105";
string WheelMotor1Name = "WheelMotor1";
string WheelMotor2Name = "WheelMotor2";
string WheelMotor3Name = "WheelMotor3";
string TractionMotor1Name = "TractionMotor1";
string TractionMotor2Name = "TractionMotor2";
string TouchSensor1Name = "TouchSensor1";
string TouchSensor2Name = "TouchSensor2";
string DistanceSensor1Name = "DistanceSensor1";
string DistanceSensor2Name = "DistanceSensor2";
string LightSensor1Name = "LightSensor1";
float TouchSensor1Value = 0;
float TouchSensor2Value = 0;
float DistanceSensor1Value = 0;
float DistanceSensor2Value = 0;
float LightSensor1Value = 0;
float V1 = 0;
float V2 = 0;
float V3 = 0;
float Vz = 0;
float Vx = 0;
float W = 0;
float k = 1;
float TV1 = 0;
float TV2 = 0;
int WaitAfterAction = 0;

int State = 0;
bool IsFinished = false;
std_msgs::Bool IsAllReady_msg;
std_msgs::Float64 TouchSensor1Value_msg;
std_msgs::Float64 TouchSensor2Value_msg;
bool IsAllReady = false;
ros::Time LastNotReadyTime;
ros::Duration Wait2SyncDuration(3.0);

float LiftingForceThreshold = 20;
float PressureMargin = 0;
float RequiredHeightOfObject = 670;
float HeightErrorMargin = 0;
float MinimumPossibleHeight = 428;
float MaximumPossibleHeight = 990;
//ros::ServiceClient time_step_client;
//webots_ros::set_int time_step_srv;

void touchSensor1Callback(const webots_ros::Float64Stamped::ConstPtr& value) {
    //ROS_INFO("Touch sensor 1 sent value %f (time: %d:%d).", value->data, value->header.stamp.sec, value->header.stamp.nsec);
    TouchSensor1Value = TouchSensor1Value * 0.95 + (value->data)*0.05;
}

void touchSensor2Callback(const webots_ros::Float64Stamped::ConstPtr& value) {
    //ROS_INFO("Touch sensor 2 sent value %f (time: %d:%d).", value->data, value->header.stamp.sec, value->header.stamp.nsec);
    TouchSensor2Value = TouchSensor2Value * 0.95 + (value->data)*0.05;
}

void distanceSensor1Callback(const sensor_msgs::Range::ConstPtr& value) {
    DistanceSensor1Value = value->range;
}

void distanceSensor2Callback(const sensor_msgs::Range::ConstPtr& value) {
    DistanceSensor2Value = value->range;
}

void lightSensor1Callback(const sensor_msgs::Illuminance::ConstPtr& value) {
    LightSensor1Value = value->illuminance;
}

void IsAllReadyCallback(const std_msgs::Bool::ConstPtr& msg) {
    if ((msg->data)&&((ros::Time::now() - LastNotReadyTime) >= Wait2SyncDuration))
        IsAllReady = true;
    else if ((msg->data)&&!((ros::Time::now() - LastNotReadyTime) >= Wait2SyncDuration)) {
        //wait for the others to be ready 
    } else if (!(msg->data))
        LastNotReadyTime = ros::Time::now();



}

void modelNameCallback(const std_msgs::String::ConstPtr& name) {
    model_count++;
    model_list.push_back(name->data);
    ROS_INFO("Model #%d: %s.", model_count, model_list.back().c_str());
}

void quit(int sig) {
    // time_step_srv.request.value = 0;
    // time_step_client.call(time_step_srv);
    ROS_INFO("User stopped the 'TELbot_controller' node.");
    ros::shutdown();
    exit(0);
}




const float r = 0.5;
const float R = 0.5;
const float alpha1 = -90;
const float alpha2 = 150;
const float alpha3 = 30;

/*************omnidicontroller**************/
float cs(float angle) {
    return cos(angle * PI / 180.0);
}

float sn(float angle) {
    return sin(angle * PI / 180.0);
}

void omnidicontroller(float Theta, float k, float W, float Vz, float Vx, float &V1, float &V2, float &V3) {
    //V1 = k*(-0.156*Vz -0.578*Vx +0.422*W/r);
    //V2 = k*(-0.636*Vz +0.365*Vx +0.365*W/r);
    //V3 = k*(+0.788*Vz +0.211*Vx +0.211*W/r);
    V1 = k * (cs(Theta + alpha1) * Vz + sn(Theta + alpha1) * Vx + R * W) / r;
    V2 = k * (cs(Theta + alpha2) * Vz + sn(Theta + alpha2) * Vx + R * W) / r;
    V3 = k * (cs(Theta + alpha3) * Vz + sn(Theta + alpha3) * Vx + R * W) / r;


}

void sync() {
    if (!IsAllReady) {
        //Publish Ready true

        IsAllReady_msg.data = true;


    } else {

        IsFinished = true;

    }
}

void up(float traction_speed) {
    TV1 = -traction_speed;
    TV2 = -traction_speed;
}

void down(float traction_speed) {
    TV1 = traction_speed;
    TV2 = traction_speed;
}

void maintain_pressure(float PressureThreshold) {
    if ((TouchSensor1Value < PressureThreshold - PressureMargin) && (TouchSensor2Value < PressureThreshold - PressureMargin)) {
        //if both sensors have low readings move forward to put more pressure
        float Error = 0;
        float P = 0.02;
        if (TouchSensor1Value >= TouchSensor2Value)
            Error = PressureThreshold - TouchSensor1Value;
        else Error = PressureThreshold - TouchSensor2Value;

        Vz = (0.5 / 3) * P*Error;
        Vx = (0.866 / 3) * P*Error;
        W = 0;
        float Theta = 0;
        omnidicontroller(Theta, k, W, Vz, Vx, V1, V2, V3);

    } else if ((TouchSensor1Value > PressureThreshold + PressureMargin) && (TouchSensor2Value > PressureThreshold + PressureMargin)) {
        //if both sensors have high readings move backwards to reduce the pressure
        float Error = 0;
        float P = 0.02;
        if (TouchSensor1Value <= TouchSensor2Value)
            Error = PressureThreshold - TouchSensor1Value;
        else Error = PressureThreshold - TouchSensor2Value;

        Vz = (0.5 / 3) * P*Error;
        Vx = (0.866 / 3) * P*Error;
        W = 0;
        float Theta = 0;
        omnidicontroller(Theta, k, W, Vz, Vx, V1, V2, V3);

    } else if (
            ((TouchSensor1Value > PressureThreshold + PressureMargin) && (TouchSensor2Value < PressureThreshold - PressureMargin)) ||
            ((TouchSensor1Value < PressureThreshold - PressureMargin) && (TouchSensor2Value > PressureThreshold + PressureMargin))) {

        //if one sensor is above threshold and the other is below then turn to the opposite
        //direction to balance the pressure (the turning is done by moving forward + angular velocity)
        float Error = TouchSensor2Value - TouchSensor1Value;
        float P = 0.008;


        Vz = (0.5 / 3)*0.08;
        Vx = (0.866 / 3)*0.08;
        W = P*Error;
        float Theta = 0;
        omnidicontroller(Theta, k, W, Vz, Vx, V1, V2, V3);
    } else if ((TouchSensor1Value > PressureThreshold - PressureMargin) && (TouchSensor2Value < PressureThreshold - PressureMargin)
            &&(TouchSensor1Value < PressureThreshold + PressureMargin) && (TouchSensor2Value > PressureThreshold - PressureMargin)) {
        Vz = 0;
        Vx = 0;
        W = 0;
        float Theta = 0;
        omnidicontroller(Theta, k, W, Vz, Vx, V1, V2, V3);
    }

}

void maintain_height(float height) {
    if ((DistanceSensor1Value < height - HeightErrorMargin)&&(DistanceSensor1Value > MinimumPossibleHeight)) {
        up(0.2);
    } else if ((DistanceSensor1Value > height + HeightErrorMargin)&&(DistanceSensor1Value < MaximumPossibleHeight)) {
        down(0.2);
    } else {
        down(0);
    }
}

void decide() {
    //Here we decide what to do depending on the current state 
    //and the readings from the sensors. 
    //Note that the conditions to pass into a state are processed with the previous state
    switch (State) {
        case _Start_:
        {
            if (std::cin.ignore())//start the program directly
            {
                State = _MoveToTarget_;
                IsFinished = false;
            }
            break;
        }
        case _MoveToTarget_:
        {
            if (IsFinished) {
                State = _TouchingTarget_;
                IsFinished = false;
            }
            break;
        }
        case _TouchingTarget_:
        {
            if (IsFinished) {
                State = _FixingOrientation_;
                IsFinished = false;
            }
            break;
        }
        case _FixingOrientation_:
        {
            if (IsFinished) {
                State = _Synchronize_;
                IsFinished = false;
            }
            break;
        }
        case _Synchronize_:
        {
            if (IsFinished) {
                State = _Lifting_;
                IsFinished = false;
            }
            break;
        }
        case _Lifting_:
        {
            if (IsFinished) {
                State = _MaintainAndSync_;
                IsFinished = false;
            }
            break;
        }
        case _MaintainAndSync_:
        {
            if (IsFinished) {
                State = _MaintainAndTransport_;
                IsFinished = false;
            }
            break;
        }
        case _MaintainAndTransport_:
        {
            if (IsFinished) {
                State = _Exit_;
                IsFinished = false;
            }
            break;
        }


        default:
        {
            ROS_INFO("Default Decide State");
            break;
        }
    }
}

void process() {
    switch (State) {
        case _Start_:
        {
            ROS_INFO("State Machine Started");
            IsFinished = true;
            break;
        }
        case _MoveToTarget_:
        {
            if ((TouchSensor1Value < 5) && (TouchSensor2Value < 5)) {
                ROS_INFO("_MoveToTarget_ State");
                Vz = 0.5 / 3;
                Vx = 0.866 / 3;
                W = 0;
                float Theta = 0;
                omnidicontroller(Theta, k, W, Vz, Vx, V1, V2, V3);
            } else IsFinished = true;
            break;
        }
        case _TouchingTarget_:
        {
            ROS_INFO("_TouchingTarget_ State");
            V1 = 0;
            V2 = 0;
            V3 = 0;
            TV1 = 0;
            TV2 = 0;
            WaitAfterAction = 2; //wait for 2 seconds
            IsFinished = true;
            break;
        }
        case _FixingOrientation_:
        {
            ROS_INFO("_FixingOrientation_ State");
            if ((TouchSensor1Value < 2) && (TouchSensor2Value < 2)) {
                Vz = 0.5 / 3;
                Vx = 0.866 / 3;
                W = 0;
                float Theta = 0;
                omnidicontroller(Theta, k, W, Vz, Vx, V1, V2, V3);
                TV1 = 0;
                TV2 = 0;
            } else if ((TouchSensor1Value > 2) && (TouchSensor2Value < 2)) {
                Vz = 0;
                Vx = 0;
                W = -0.3;
                float Theta = 0;
                omnidicontroller(Theta, k, W, Vz, Vx, V1, V2, V3);
                TV1 = 0;
                TV2 = 0;
            } else if ((TouchSensor1Value < 2) && (TouchSensor2Value > 2)) {
                Vz = 0;
                Vx = 0;
                W = 0.3;
                float Theta = 0;
                omnidicontroller(Theta, k, W, Vz, Vx, V1, V2, V3);
                TV1 = 0;
                TV2 = 0;
            } else if ((TouchSensor1Value > 2) && (TouchSensor2Value > 2)) {
                Vz = 0;
                Vx = 0;
                W = 0;
                float Theta = 0;
                omnidicontroller(Theta, k, W, Vz, Vx, V1, V2, V3);
                TV1 = 0;
                TV2 = 0;

                IsFinished = true;

            }

            break;
        }

        case _Synchronize_:
        {
            ROS_INFO("_Synchronize_ State");
            sync();
            break;
        }
        case _Lifting_:
        {
            ROS_INFO("_Lifting_ State");
            maintain_pressure(LiftingForceThreshold);
            up(0.2);
            if ((DistanceSensor1Value > 660) && (DistanceSensor1Value < 680))
                IsFinished = true;
            break;
        }
        case _MaintainAndSync_:
        {
            ROS_INFO("_MaintainAndSync_ State");
            // sync();
            maintain_pressure(LiftingForceThreshold);
            maintain_height(RequiredHeightOfObject);
            break;
        }
        case _MaintainAndTransport_:
        {
            ROS_INFO("_MaintainAndTransport_ State");
            float AppliedForceThreshold = LiftingForceThreshold;
            if (LightSensor1Value < 100)
                AppliedForceThreshold = AppliedForceThreshold + 10;
            else if (LightSensor1Value > 100)
                AppliedForceThreshold = AppliedForceThreshold - 7;
            maintain_pressure(AppliedForceThreshold);
            maintain_height(RequiredHeightOfObject);


            //            IsFinished = true;
            break;
        }



        case _Exit_:
        {
            ROS_INFO("Exit State");
            std::cin.ignore();

            break;
        }
        default:
        {
            ROS_INFO("Default Process State");
            break;
        }
    }
}

/*****************************************/

int main(int argc, char **argv) {


    if (argc != 1) {
        ROS_INFO("Usage: $ TELbot_controller.");
        //return 1;
    }

    ros::init(argc, argv, "TELbot_controller", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    ModelName = argv[1];

    signal(SIGINT, quit);
    // int nStep = 0;

    /********************Getting Model Name*********************/
    ros::Subscriber name_sub = n.subscribe("model_name", 100, modelNameCallback);
    while (model_count == 0 || model_count < name_sub.getNumPublishers()) {
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
    }
    ros::spinOnce();
    name_sub.shutdown();
    /*********************Shuts Down after getting all the names********************/

    //There is no need to call time service because we are using the simulator time
    //Using the simulator time is obligatory in our case since we have multiple nodes in on world. 


    //Setting up time_client 
    //    time_step_client = n.serviceClient<webots_ros::set_int>(ModelName + "/robot/time_step");
    //    time_step_srv.request.value = TIME_STEP;
    //    //Checking if time_service works
    //    if (time_step_client.call(time_step_srv) && time_step_srv.response.success) {
    //        ROS_INFO("time_step service works.");
    //        // nStep++;
    //    } else
    //        ROS_ERROR("Failed to call service time_step to update robot's time step.");
    ////////////////////////////////
    /*****Setting up Synchronization signal topic (Subscriber/publisher) IsAllReady*********/
    ros::Publisher IsAllReadyPublisher = n.advertise<std_msgs::Bool>("/IsAllReady", 1);
    ros::Subscriber IsAllReadySubscriber = n.subscribe("/IsAllReady", 100, IsAllReadyCallback);
    LastNotReadyTime = ros::Time::now();

    /***************************************************************************************/
    /********************* set up the touch sensors*********************/

    ros::ServiceClient EnableTouchSensor1Client = n.serviceClient<webots_ros::set_int>(ModelName + "/" + TouchSensor1Name + "/enable");
    ros::ServiceClient EnableTouchSensor2Client = n.serviceClient<webots_ros::set_int>(ModelName + "/" + TouchSensor2Name + "/enable");

    webots_ros::set_int touch_sensor_srv;
    touch_sensor_srv.request.value = 32;

    if (EnableTouchSensor1Client.call(touch_sensor_srv) && touch_sensor_srv.response.success)
        ROS_INFO("TouchSensor1 enabled.");
    else {
        ROS_ERROR("Failed to enable touch_sensor.");
    }
    if (EnableTouchSensor2Client.call(touch_sensor_srv) && touch_sensor_srv.response.success)
        ROS_INFO("TouchSensor2 enabled.");
    else {
        ROS_ERROR("Failed to enable touch_sensor.");
    }

    ros::Subscriber sub_touch_sensor_1_32 = n.subscribe(ModelName + "/" + TouchSensor1Name + "/value", 100, touchSensor1Callback);
    ros::Subscriber sub_touch_sensor_2_32 = n.subscribe(ModelName + "/" + TouchSensor2Name + "/value", 100, touchSensor2Callback);


    ros::Publisher FilteredTouchSensor1Publisher = n.advertise<std_msgs::Float64>(ModelName + "/" + TouchSensor1Name + "/filteredvalue", 1);
    ros::Publisher FilteredTouchSensor2Publisher = n.advertise<std_msgs::Float64>(ModelName + "/" + TouchSensor2Name + "/filteredvalue", 1);
    /*********************************************************************/
    /********************* set up the distance sensors*********************/
    ros::ServiceClient EnableDistanceSensor1Client = n.serviceClient<webots_ros::set_int>(ModelName + "/" + DistanceSensor1Name + "/enable");
    ros::ServiceClient EnableDistanceSensor2Client = n.serviceClient<webots_ros::set_int>(ModelName + "/" + DistanceSensor2Name + "/enable");

    webots_ros::set_int distance_sensor_srv;
    distance_sensor_srv.request.value = 32;

    if (EnableDistanceSensor1Client.call(distance_sensor_srv) && distance_sensor_srv.response.success)
        ROS_INFO("DistanceSensor1 enabled.");
    else {
        ROS_ERROR("Failed to enable distance_sensor.");
    }
    if (EnableDistanceSensor2Client.call(distance_sensor_srv) && distance_sensor_srv.response.success)
        ROS_INFO("DistanceSensor2 enabled.");
    else {
        ROS_ERROR("Failed to enable distance_sensor.");
    }

    ros::Subscriber sub_distance_sensor_1_32 = n.subscribe(ModelName + "/" + DistanceSensor1Name + "/value", 100, distanceSensor1Callback);
    ros::Subscriber sub_distance_sensor_2_32 = n.subscribe(ModelName + "/" + DistanceSensor2Name + "/value", 100, distanceSensor2Callback);

    /*********************************************************************/
    /********************* set up the Light sensor*********************/
    ros::ServiceClient EnableLightSensor1Client = n.serviceClient<webots_ros::set_int>(ModelName + "/" + LightSensor1Name + "/enable");
    //    ros::ServiceClient EnableDistanceSensor2Client = n.serviceClient<webots_ros::sensor_enable>(ModelName + "/" + DistanceSensor2Name + "/enable");

    webots_ros::set_int light_sensor_srv;
    light_sensor_srv.request.value = 32;

    if (EnableLightSensor1Client.call(light_sensor_srv) && light_sensor_srv.response.success)
        ROS_INFO("LightSensor1 enabled.");
    else {
        ROS_ERROR("Failed to enable light_sensor.");
    }
    //    if (EnableDistanceSensor2Client.call(distance_sensor_srv) && distance_sensor_srv.response.success)
    //        ROS_INFO("DistanceSensor2 enabled.");
    //    else {
    //        ROS_ERROR("Failed to enable distance_sensor.");
    //    }

    ros::Subscriber sub_light_sensor_1_32 = n.subscribe(ModelName + "/" + LightSensor1Name + "/value", 100, lightSensor1Callback);
    //    ros::Subscriber sub_distance_sensor_2_32 = n.subscribe(ModelName + "/" + DistanceSensor2Name + "/value", 100, distanceSensor2Callback);



    /*********************************************************************/
    /***********************Initialize motor velocities with zero *************/
    //This step is done before changing to velocity control because setting the position to infinity directly
    //makes the robots spin until the initialization command is executed. 
    ros::ServiceClient WheelMotor1SetVelocityClient = n.serviceClient<webots_ros::set_float>(ModelName + "/" + WheelMotor1Name + "/set_velocity");
    ros::ServiceClient WheelMotor2SetVelocityClient = n.serviceClient<webots_ros::set_float>(ModelName + "/" + WheelMotor2Name + "/set_velocity");
    ros::ServiceClient WheelMotor3SetVelocityClient = n.serviceClient<webots_ros::set_float>(ModelName + "/" + WheelMotor3Name + "/set_velocity");
    ros::ServiceClient TractionMotor1SetVelocityClient = n.serviceClient<webots_ros::set_float>(ModelName + "/" + TractionMotor1Name + "/set_velocity");
    ros::ServiceClient TractionMotor2SetVelocityClient = n.serviceClient<webots_ros::set_float>(ModelName + "/" + TractionMotor2Name + "/set_velocity");

    webots_ros::set_float SetWheelMotor1VelocitySrv;
    webots_ros::set_float SetWheelMotor2VelocitySrv;
    webots_ros::set_float SetWheelMotor3VelocitySrv;
    webots_ros::set_float SetTractionMotor1VelocitySrv;
    webots_ros::set_float SetTractionMotor2VelocitySrv;
    SetWheelMotor1VelocitySrv.request.value = 0;
    SetWheelMotor2VelocitySrv.request.value = 0;
    SetWheelMotor3VelocitySrv.request.value = 0;
    SetTractionMotor1VelocitySrv.request.value = 0;
    SetTractionMotor2VelocitySrv.request.value = 0;

    WheelMotor1SetVelocityClient.call(SetWheelMotor1VelocitySrv);
    WheelMotor2SetVelocityClient.call(SetWheelMotor2VelocitySrv);
    WheelMotor3SetVelocityClient.call(SetWheelMotor3VelocitySrv);
    TractionMotor1SetVelocityClient.call(SetTractionMotor1VelocitySrv);
    TractionMotor2SetVelocityClient.call(SetTractionMotor2VelocitySrv);




    /********************* set the motors to velocity control********************/


    ros::ServiceClient WheelMotor1PositionClient = n.serviceClient<webots_ros::set_float>(ModelName + "/" + WheelMotor1Name + "/set_position");
    ros::ServiceClient WheelMotor2PositionClient = n.serviceClient<webots_ros::set_float>(ModelName + "/" + WheelMotor2Name + "/set_position");
    ros::ServiceClient WheelMotor3PositionClient = n.serviceClient<webots_ros::set_float>(ModelName + "/" + WheelMotor3Name + "/set_position");
    ros::ServiceClient TractionMotor1PositionClient = n.serviceClient<webots_ros::set_float>(ModelName + "/" + TractionMotor1Name + "/set_position");
    ros::ServiceClient TractionMotor2PositionClient = n.serviceClient<webots_ros::set_float>(ModelName + "/" + TractionMotor2Name + "/set_position");


    webots_ros::set_float WheelMotorSrv;
    WheelMotorSrv.request.value = INFINITY;

    WheelMotor1PositionClient.call(WheelMotorSrv);
    WheelMotor2PositionClient.call(WheelMotorSrv);
    WheelMotor3PositionClient.call(WheelMotorSrv);
    TractionMotor1PositionClient.call(WheelMotorSrv);
    TractionMotor2PositionClient.call(WheelMotorSrv);
    /*****************************************************************************/







    while ((ros::ok())&&(State != _Exit_)) {

        ros::spinOnce();
        loop_rate.sleep();


        //        if (time_step_client.call(time_step_srv) && time_step_srv.response.success) {
        //            //ROS_INFO("time_step service works.");
        //            // nStep++;
        //        } else
        //            ROS_ERROR("Failed to call service time_step to update robot's time step.");

        decide();
        process();
        /***********Execute************/
        SetWheelMotor1VelocitySrv.request.value = V1;
        SetWheelMotor2VelocitySrv.request.value = V2;
        SetWheelMotor3VelocitySrv.request.value = V3;
        SetTractionMotor1VelocitySrv.request.value = TV1;
        SetTractionMotor2VelocitySrv.request.value = TV2;

        WheelMotor1SetVelocityClient.call(SetWheelMotor1VelocitySrv);
        WheelMotor2SetVelocityClient.call(SetWheelMotor2VelocitySrv);
        WheelMotor3SetVelocityClient.call(SetWheelMotor3VelocitySrv);
        TractionMotor1SetVelocityClient.call(SetTractionMotor1VelocitySrv);
        TractionMotor2SetVelocityClient.call(SetTractionMotor2VelocitySrv);
        
        
        TouchSensor1Value_msg.data=TouchSensor1Value;
        TouchSensor2Value_msg.data=TouchSensor2Value;
        FilteredTouchSensor1Publisher.publish(TouchSensor1Value_msg);
        FilteredTouchSensor2Publisher.publish(TouchSensor2Value_msg);
        
        
        if (WaitAfterAction != 0)
            ros::Duration(WaitAfterAction).sleep();
        WaitAfterAction = 0;
        IsAllReadyPublisher.publish(IsAllReady_msg);
        IsAllReady_msg.data = false;
        /******************************/

    }




    //dont forget to shutdown all the clients before returning 
    TractionMotor1SetVelocityClient.shutdown();
    //    time_step_client.call(time_step_srv);

    return 0;
}

