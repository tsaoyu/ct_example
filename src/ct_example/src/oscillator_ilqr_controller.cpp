#include <ct/optcon/optcon.h>
#include "configDir.h"
#include "plotResult.h"

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Point.h>

using namespace ct::core;
using namespace ct::optcon;


class ILQRController {

    public:
        typedef std::shared_ptr<ct::core::StateFeedbackController<SecondOrderSystem::STATE_DIM, SecondOrderSystem::CONTROL_DIM>> ControllerPtr_t;
        typedef ct::core::StateFeedbackController<SecondOrderSystem::STATE_DIM, SecondOrderSystem::CONTROL_DIM> * ControllerConstPtr_t;
        typedef std::shared_ptr<ct::core::SystemLinearizer<SecondOrderSystem::STATE_DIM, SecondOrderSystem::CONTROL_DIM>> LinearSystemPtr_t;
        typedef std::shared_ptr<NLOptConSolver<SecondOrderSystem::STATE_DIM, SecondOrderSystem::CONTROL_DIM>> NLOPPtr_t;
        ILQRController(ros::NodeHandle *n){
            pose_ref_sub = n->subscribe("/pose_ref",10, &ILQRController::pose_ref_callback, this);
            cmd_wrench_pub = n->advertise<geometry_msgs::Wrench>("/cmd_wrench", 10);
            pos_sub = n->subscribe("/position", 10, &ILQRController::pos_callback, this);
        }


        void create_controller(const ct::core::StateVector<SecondOrderSystem::STATE_DIM>& x_init, 
                               const ct::core::StateVector<SecondOrderSystem::STATE_DIM>& x_ref){

            // Step 1: setup Nonlinear Optimal Control Problem

            const size_t state_dim = SecondOrderSystem::STATE_DIM;
            const size_t control_dim = SecondOrderSystem::CONTROL_DIM;
        
            double w_n = 0.1;
            double zeta = 5.0;
            double g_dc = 1.0;

            // Step 1-A: create controller instance

            std::shared_ptr<ct::core::ControlledSystem<state_dim, control_dim>> oscillatorDynamics(
                new ct::core::SecondOrderSystem(w_n, zeta, g_dc));
            
            // Step 1-B: create a numerical linearizer

            std::shared_ptr<ct::core::SystemLinearizer<state_dim, control_dim>> adLinearizer(new
                SystemLinearizer<state_dim, control_dim>(oscillatorDynamics)            
            );
            

            // STEP 1-C: create the cost function.
            std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> intermediateCost(
                new ct::optcon::TermQuadratic<state_dim, control_dim>());
            std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> finalCost(
                new ct::optcon::TermQuadratic<state_dim, control_dim>());

            intermediateCost->loadConfigFile(configDir + "/soc_Cost.info", "intermediateCost");
            finalCost->loadConfigFile(configDir + "/soc_Cost.info", "finalCost");
            intermediateCost->updateReferenceState(this->x_ref);
            finalCost->updateReferenceState(this->x_ref);


            std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction(
                new CostFunctionAnalytical<state_dim, control_dim>());
            costFunction->addIntermediateTerm(intermediateCost);
            costFunction->addFinalTerm(finalCost);


            // STEP 1-D: initialization with initial state and desired time horizon

            ct::core::Time timeHorizon = 3.0;

            // STEP 1-E: create and initialize an "optimal control problem"
            ContinuousOptConProblem<state_dim, control_dim> optConProblem(
                timeHorizon, x_init, oscillatorDynamics, costFunction, adLinearizer);

            // STEP 2-A: NLOpt settings

            NLOptConSettings nloc_settings;
            nloc_settings.load(configDir + "/soc_nloc.info", true, "ilqr");
        
            // STEP 2-B: provide an initial guess
            size_t N = nloc_settings.computeK(timeHorizon);
            FeedbackMatrix<state_dim, control_dim> u_fb;
            u_fb << -0.5, -1;
            FeedbackArray<state_dim, control_dim> u0_fb(N, u_fb);
            ControlVectorArray<control_dim> u0_ff(N, ControlVector<control_dim>::Random());
            StateVectorArray<state_dim> x_ref_init(N + 1, x_init);
            NLOptConSolver<state_dim, control_dim>::Policy_t initController(x_ref_init, u0_ff, u0_fb, nloc_settings.dt);
            // How to create a more complicated controller for iterations?

            // STEP 2-C: create an NLOptConSolver instance
     
            NLOPPtr_t iLQR(new NLOptConSolver<state_dim, control_dim>(optConProblem, nloc_settings));
            this->nlop_problem = iLQR;
            this->nlop_problem->setInitialGuess(initController);

            // STEP 3: solve the optimal control problem
            this->nlop_problem->solve();

   

        }


        void pos_message_converter(const geometry_msgs::Point::ConstPtr & msg,
                                  ct::core::StateVector<SecondOrderSystem::STATE_DIM>& x)
            {

                x(0) = msg->x;
                x(1) = msg->y;
            }

        void pos_callback(const geometry_msgs::Point::ConstPtr & msg){
            if (first_pass_) {
                ILQRController::pos_message_converter(msg, this->x_now);
                first_pass_ =  false;
                return;
            }
            if (controller_not_created_){
                return; // Only start to update reference when controller is created
            }

            ILQRController::pos_message_converter(msg, this->x_now);
            
        }

        void plot_solution(){
            const size_t state_dim = SecondOrderSystem::STATE_DIM;
            const size_t control_dim = SecondOrderSystem::CONTROL_DIM;

            ct::core::StateFeedbackController<state_dim, control_dim> solution = this->nlop_problem->getSolution();
            

            std::cout << solution.x_ref()[10] << std::endl;

            plotResultsOscillator<state_dim, control_dim>(solution.x_ref(),
                                                   solution.K(),
                                                   solution.uff(), 
                                                   solution.time());

            
        }


        void pose_ref_callback(const geometry_msgs::Point::ConstPtr & msg){
            
            if (controller_not_created_){
                if (first_pass_){
                    return;
                }
                ILQRController::pos_message_converter(msg, this->x_ref);
                ILQRController::create_controller(this->x_now, this->x_ref);
                x_ref_current = this->x_ref;
                ILQRController::plot_solution();
                start_time = ros::Time::now().toSec();
                controller_not_created_ = false;
                return; 
            }

            ILQRController::pos_message_converter(msg, this->x_ref);
        

            if (x_ref_current == this->x_ref){
                return;
                // If the reference is the same as last time received do nothing
            }
            else
            {
                ILQRController::create_controller(this->x_now, this->x_ref);
                x_ref_current = this->x_ref;
                start_time = ros::Time::now().toSec();
                // Otherwise create a new controller that start from current state to new reference point
                // The internal start time should also updated for the new controller
            }
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

            ControlVector<control_dim> u;
            ct::core::StateFeedbackController<state_dim, control_dim> solution = this->nlop_problem->getSolution();
            // Compute control from the optimised controller
            solution.computeControl(this->x_now, t, u);
       
            std::cout << "Time now: " << t << " Reference point: " << this->x_ref(0) << " " << this->x_ref(1) 
            << " Current point: "  << this->x_now(0) << " " << this->x_now(1) << " \n";

            geometry_msgs::Wrench wrench;
            wrench.force.x = u(0);
            cmd_wrench_pub.publish(wrench);
        }

        private:
            ros::Subscriber pose_ref_sub, pos_sub;
            ros::Publisher cmd_wrench_pub;
            ct::core::StateVector<SecondOrderSystem::STATE_DIM> x_now;
            ct::core::StateVector<SecondOrderSystem::STATE_DIM> x_ref;
            ct::core::StateVector<SecondOrderSystem::STATE_DIM> x_ref_current;
            ct::core::ControlVector<SecondOrderSystem::CONTROL_DIM> u;  

            double rostime_now;
            ct::core::Time t_now;
            ct::core::Time t_final;

            ControllerPtr_t controller;
            LinearSystemPtr_t Linearizer;
            NLOPPtr_t nlop_problem; 

            double start_time;
            double current_time;  
            bool first_pass_ = true;
            bool controller_not_created_ = true;
            bool overview_ = true;

};


int main(int argc, char** argv){
    

    ros::init(argc, argv, "soc_ilqr_controller_node");
    ros::NodeHandle n;

    ros::NodeHandle private_node_handle("~");
    ILQRController ilqr_controller  = ILQRController(&n);

    int rate = 50;
    ros::Rate r(rate);

    while (n.ok())

    {   
        ros::spinOnce();
        ilqr_controller.publish_cmd_wrench();
        r.sleep();
        
    }
}

