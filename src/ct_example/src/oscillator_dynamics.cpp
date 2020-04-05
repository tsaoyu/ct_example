#include <ct/optcon/optcon.h>
#include "configDir.h"
#include "ManualController.h"

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Point.h>

using namespace ct::core;
using namespace ct::optcon;


class OscillatorDynamicsSimulator
{ 
    public:
        
        OscillatorDynamicsSimulator(ros::NodeHandle *n){
            OscillatorDynamicsSimulator::create_controlled_system();
            pos_pub = n->advertise<geometry_msgs::Point>("/position", 10);
            cmd_wrench_sub = n->subscribe("/cmd_wrench", 10,  &OscillatorDynamicsSimulator::callback_cmd_wrench, this);
        }

        void create_controlled_system(){
            const size_t state_dim = SecondOrderSystem::STATE_DIM;
            const size_t control_dim = SecondOrderSystem::CONTROL_DIM;
        
            double w_n = 0.1;
            double zeta = 5.0;
            double g_dc = 1.0;
            // Step 1: create a manual controller to accept external control command 
            std::shared_ptr<ManualController<state_dim, control_dim>> manual_controller (new
                ManualController<state_dim, control_dim>()
            );

            this->manual_controller = manual_controller;
            this->manual_controller -> updateControl({0});
            
            // Step 2: create a controlled system for dynamics simulation
            std::shared_ptr<ct::core::ControlledSystem<state_dim, control_dim>> oscillatorDynamics(
                new ct::core::SecondOrderSystem(w_n, zeta, g_dc, this->manual_controller));

            // Step 3: configure the integrator and integration type
            this->integrator = new ct::core::Integrator<SecondOrderSystem::STATE_DIM>(oscillatorDynamics, IntegrationType::RK78);
            x.setZero();
        }


        void reset_time(){
            t_now = 0.0;
        }

        void callback_cmd_wrench(const geometry_msgs::Wrench::ConstPtr& msg){
            if (first_received) {
                OscillatorDynamicsSimulator::reset_time();
                first_received = false;
                rostime_now = ros::Time::now().toSec();
                return ;
            }
            const size_t state_dim = SecondOrderSystem::STATE_DIM;
            const size_t control_dim = SecondOrderSystem::CONTROL_DIM;
            // Receive control signal from message 
            std::vector<double> control = { msg->force.x};
            // Manual control not only accepts manual control but also message from other publishers
            // this allow us to simulate the system with other controllers
            this -> manual_controller->updateControl(control);

            double dt = ros::Time::now().toSec() - rostime_now;
            t_final = t_now + dt;
            // Integrate to current time
            this -> integrator -> integrate_adaptive(x, t_now, t_final);
            t_now = t_final;
            rostime_now = ros::Time::now().toSec();
        }

        void publish_state(){
            geometry_msgs::Point p;
            p.x = x(0);
            p.y = x(1);
            pos_pub.publish(p);

        }

    private:
        ros::Subscriber cmd_wrench_sub;
        ros::Publisher pos_pub;

        bool first_received = true;
        double rostime_now; 
        ct::core::Time t_now;
        ct::core::Time t_final;

        ct::core::Integrator<SecondOrderSystem::STATE_DIM> * integrator;
        StateVector<SecondOrderSystem::STATE_DIM> x;
        std::shared_ptr<ManualController<SecondOrderSystem::STATE_DIM, SecondOrderSystem::CONTROL_DIM>> manual_controller;
};


int main(int argc, char **argv){

    ros::init(argc, argv, "oscillator_dynamics_simulator_node");
    ros::NodeHandle n;

    ros::NodeHandle private_node_handle("~");
    OscillatorDynamicsSimulator dynamic_simulator  = OscillatorDynamicsSimulator(&n);

    int rate = 100;
    ros::Rate r(rate);

    while (n.ok())

    {   
        ros::spinOnce();
        dynamic_simulator.publish_state();
        r.sleep();
        
    }

}
