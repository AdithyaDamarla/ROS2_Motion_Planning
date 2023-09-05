#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <arm_robot_msgs/srv/euler_to_quaternion.hpp> 
#include <arm_robot_msgs/srv/quaternion_to_euler.hpp>
#include <tf2/utils.h>


using namespace std::placeholders;

class AnglesConverter : public rclcpp::Node{
    public:
        AnglesConverter(): Node("angles_conversion_service"){
            euler_to_quaternion_= create_service<arm_robot_msgs::srv::EulerToQuaternion>("euler_to_quaternion", std::bind(&AnglesConverter::euler_to_quaternioncallback, this ,_1,_2));
            quaternion_to_euler_= create_service<arm_robot_msgs::srv::QuaternionToEuler>("quaternion_to_euler", std::bind(&AnglesConverter::quaternion_to_eulercallback, this ,_1,_2));
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Angles conversion services are ready");
        }
        
    private:
        rclcpp::Service<arm_robot_msgs::srv::EulerToQuaternion>::SharedPtr euler_to_quaternion_;
        rclcpp::Service<arm_robot_msgs::srv::QuaternionToEuler>::SharedPtr quaternion_to_euler_;

        void euler_to_quaternioncallback(const std::shared_ptr<arm_robot_msgs::srv::EulerToQuaternion::Request> req, const std::shared_ptr<arm_robot_msgs::srv::EulerToQuaternion::Response> res)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Request to conversion euler angles Roll : " << req->roll << " Pitch : " << req->pitch << ", Yaw : " << req->yaw << " into a quaternion");
            tf2::Quaternion q;
            q.setRPY(req->roll, req->pitch, req->yaw);
            res->x = q.getX();
            res->y = q.getY();
            res->z = q.getZ();
            res->w = q.getW();
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Corresponding quaternion x : " << res->x << ", y : " << res->y << ", z : " << res->z << ", w : " << res->w);

        }
            
                                        
        void quaternion_to_eulercallback(const std::shared_ptr<arm_robot_msgs::srv::QuaternionToEuler::Request> req, const std::shared_ptr<arm_robot_msgs::srv::QuaternionToEuler::Response> res){

            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Request to conversion quaternion x : " << req->x << ", y : " << req->y << ", z : " << req->z << ", w : " << req->w);
            tf2::Quaternion q( req->x, req->y, req->z, req->w);
            tf2 :: Matrix3x3 m(q);
            m.getRPY(res->roll,res->pitch,res->yaw);
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Corresponding Euler angles Roll : " << res->roll<< ", Pitch : " << res->pitch << ", Yaw : " << res->yaw);
        }
                                            
                                         
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AnglesConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
