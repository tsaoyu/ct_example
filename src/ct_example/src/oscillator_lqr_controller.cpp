#include <ct/optcon/optcon.h>

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Point.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace ct::core;
using namespace ct::optcon;


class LQRController {

    public:
        typedef std::shared_ptr<ct::core::StateFeedbackController<SecondOrderSystem::STATE_DIM, SecondOrderSystem::CONTROL_DIM>> ControllerPtr_t;
        typedef std::shared_ptr<ct::core::SystemLinearizer<SecondOrderSystem::STATE_DIM, SecondOrderSystem::CONTROL_DIM>> LinearSystemPtr_t;

        LQRController(ros::NodeHandle *n){
            pose_ref_sub = n->subscribe("/pose_ref",10, &LQRController::pose_ref_callback, this);
            cmd_wrench_pub = n->advertise<geometry_msgs::Wrench>("/cmd_wrench", 10);
            pos_sub = n->subscribe("/position", 10, &LQRController::pos_callback, this);
        }


        void create_lqr_controller(const ct::core::StateVector<SecondOrderSystem::STATE_DIM>& x_init, 
                                   const ct::core::StateVector<SecondOrderSystem::STATE_DIM>& x_ref){

            // Step 1: create dynamics instance

            const size_t state_dim = SecondOrderSystem::STATE_DIM;
            const size_t control_dim = SecondOrderSystem::CONTROL_DIM;
        
            double w_n = 0.1;
            double zeta = 5.0;
            double g_dc = 1.0;

            std::shared_ptr<ct::core::ControlledSystem<state_dim, control_dim>> oscillatorDynamics(
                new ct::core::SecondOrderSystem(w_n, zeta, g_dc));
            
            
            // Step 2: create a numerical linearizer

            std::shared_ptr<ct::core::SystemLinearizer<state_dim, control_dim>> adLinearizer(new
                SystemLinearizer<state_dim, control_dim>(oscillatorDynamics)            
            );
            

            // Step 3: setup LQR controller

            ct::core::ControlVector<control_dim> u;
            u.setZero();
          
            double t = 0;

            auto A = adLinearizer->getDerivativeState(x_init, u, t);
            auto B = adLinearizer->getDerivativeControl(x_init, u, t);
            std::cout << "A matrix is: " << A << "\n";
            std::cout << "B matrix is: " << B << "\n" ;


            this->Linearizer = adLinearizer;

            ct::optcon::TermQuadratic<state_dim, control_dim> quadraticCost;
            ct::core::StateMatrix<state_dim> Q = ct::core::StateMatrix<state_dim>::Identity();
            ct::core::ControlMatrix<control_dim> R = ct::core::ControlMatrix<control_dim>::Identity();

            ct::core::FeedbackMatrix<state_dim, control_dim> K;
            ct::optcon::LQR<state_dim, control_dim> lqrSolver;
            lqrSolver.compute(Q, R, A, B, K);

            size_t N = 100;
            FeedbackArray<state_dim, control_dim> u0_fb(N, K);
            ControlVectorArray<control_dim> u0_ff(N, ControlVector<control_dim>::Zero());
            StateVectorArray<state_dim> x_ref_init(N + 1, x_ref);

            ControllerPtr_t controller (new 
                ct::core::StateFeedbackController<state_dim, control_dim>(x_ref_init, u0_ff, u0_fb, 0.01));
            this->controller = controller;
            start_time = ros::Time::now().toSec();

        }

        void pos_message_converter(const geometry_msgs::Point::ConstPtr & msg,
                                  ct::core::StateVector<SecondOrderSystem::STATE_DIM>& x)
            {

                x(0) = msg->x;
                x(1) = msg->y;
            }

        void pos_callback(const geometry_msgs::Point::ConstPtr & msg){
            if (first_pass_) {
                first_pass_ =  false;
                LQRController::pos_message_converter(msg, this->x_now);
                return;
            }
            if (controller_not_created_){
                return; // Only start to update reference when the controller is created
            }

            LQRController::pos_message_converter(msg, this->x_now);
            
        }


        void pose_ref_callback(const geometry_msgs::Point::ConstPtr & msg){
            
            if (controller_not_created_){
                if (first_pass_){
                    return;
                }
                LQRController::pos_message_converter(msg, this->x_ref);
                LQRController::create_lqr_controller(this->x_now, this->x_ref);
                controller_not_created_ = false;
                return; 
            }

            LQRController::pos_message_converter(msg, this->x_ref);
            
          
        }

       
        void publish_cmd_wrench(){

            if (controller_not_created_){
                return; // Only start to publish when the controller is created
            }
            const size_t state_dim = SecondOrderSystem::STATE_DIM;
            const size_t control_dim = SecondOrderSystem::CONTROL_DIM;
        
            current_time = ros::Time::now().toSec();
            ct::core::Time t = current_time - start_time;
            t_now = t;

            auto A = this->Linearizer->getDerivativeState(this->x_now, u, t_now);
            auto B = this->Linearizer->getDerivativeControl(this->x_now, u, t_now);

            
            ct::optcon::TermQuadratic<state_dim, control_dim> quadraticCost;
            ct::core::StateMatrix<state_dim> Q ;
            Q(0,0) = 1;
            Q(1,1) = 1.6;

            ct::core::ControlMatrix<control_dim> R;
            R(0,0) = 1;

            ct::core::FeedbackMatrix<state_dim, control_dim> K;
            ct::optcon::LQR<state_dim, control_dim> lqrSolver;
            lqrSolver.compute(Q, R, A, B, K);

            u = K * (this->x_ref - this->x_now);
            std::cout << "Reference point: " << this->x_ref(0) << " " << this->x_ref(1) 
            << " Current point: "  << this->x_now(0) << " " << this->x_now(1) << " K: " << K << "\n";

            geometry_msgs::Wrench wrench;
            wrench.force.x = u(0);
            cmd_wrench_pub.publish(wrench);
        }

        private:
            ros::Subscriber pose_ref_sub, pos_sub;
            ros::Publisher cmd_wrench_pub;
            ct::core::StateVector<SecondOrderSystem::STATE_DIM> x_now;
            ct::core::StateVector<SecondOrderSystem::STATE_DIM> x_ref;
            ct::core::ControlVector<SecondOrderSystem::CONTROL_DIM> u;  

            double rostime_now;
            ct::core::Time t_now;
            ct::core::Time t_final;

            ControllerPtr_t controller;
            LinearSystemPtr_t Linearizer; 

            double start_time;
            double current_time;  
            bool first_pass_ = true;
            bool controller_not_created_ = true;

};


int main(int argc, char** argv){
    

    ros::init(argc, argv, "soc_lqr_controller_node");
    ros::NodeHandle n;

    ros::NodeHandle private_node_handle("~");
    LQRController lqr_controller  = LQRController(&n);

    int rate = 50;
    ros::Rate r(rate);

    while (n.ok())

    {   
        ros::spinOnce();
        lqr_controller.publish_cmd_wrench();
        r.sleep();
        
    }
}

