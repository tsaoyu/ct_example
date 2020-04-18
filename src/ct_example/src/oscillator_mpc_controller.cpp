#include <ct/optcon/optcon.h>
#include "configDir.h"
#include "plotResult.h"

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Point.h>

using namespace ct::core;
using namespace ct::optcon;


class MPCController {

    public:
        typedef std::shared_ptr<ct::core::StateFeedbackController<SecondOrderSystem::STATE_DIM, SecondOrderSystem::CONTROL_DIM>> ControllerPtr_t;
        typedef ct::core::StateFeedbackController<SecondOrderSystem::STATE_DIM, SecondOrderSystem::CONTROL_DIM> * ControllerConstPtr_t;
        typedef std::shared_ptr<ct::core::SystemLinearizer<SecondOrderSystem::STATE_DIM, SecondOrderSystem::CONTROL_DIM>> LinearSystemPtr_t;
        typedef std::shared_ptr<NLOptConSolver<SecondOrderSystem::STATE_DIM, SecondOrderSystem::CONTROL_DIM>> NLOPPtr_t;
        typedef std::shared_ptr<MPC<NLOptConSolver<SecondOrderSystem::STATE_DIM, SecondOrderSystem::CONTROL_DIM>>> MPCPtr_t;
        typedef ct::core::StateFeedbackController<SecondOrderSystem::STATE_DIM, SecondOrderSystem::CONTROL_DIM> PolicyPtr_t;
        typedef std::shared_ptr<CostFunctionQuadratic<SecondOrderSystem::STATE_DIM, SecondOrderSystem::CONTROL_DIM>> CostFuncPtr_t;
        typedef std::shared_ptr<ct::optcon::TermQuadratic<SecondOrderSystem::STATE_DIM, SecondOrderSystem::CONTROL_DIM>> TermPtr_t;



        MPCController(ros::NodeHandle *n){
            pose_ref_sub = n->subscribe("/pose_ref",10, &MPCController::pose_ref_callback, this);
            cmd_wrench_pub = n->advertise<geometry_msgs::Wrench>("/cmd_wrench", 10);
            pos_sub = n->subscribe("/position", 10, &MPCController::pos_callback, this);
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
            

            // STEP 1-C: create a cost function.
            std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> intermediateCost(
                new ct::optcon::TermQuadratic<state_dim, control_dim>());
            std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> finalCost(
                new ct::optcon::TermQuadratic<state_dim, control_dim>());
            this->termQuad_interm = intermediateCost;
            this->termQuad_final = finalCost;

            this->termQuad_interm->loadConfigFile(configDir + "/soc_Cost.info", "intermediateCost");
            this->termQuad_final->loadConfigFile(configDir + "/soc_Cost.info", "finalCost");
            this->termQuad_interm->updateReferenceState(this->x_ref);
            this->termQuad_final->updateReferenceState(this->x_ref);
      

            CostFuncPtr_t costFunction(
                new CostFunctionAnalytical<state_dim, control_dim>());
            this->costFunc = costFunction;

            this->costFunc->addIntermediateTerm(this->termQuad_interm);
            this->costFunc->addFinalTerm(this->termQuad_final);

            // Check which cost function should I use here.

            // STEP 1-D: initialization with initial state and desired time horizon

            ct::core::Time timeHorizon = 3.0;

            // STEP 1-E: create and initialize an "optimal control problem"
            ContinuousOptConProblem<state_dim, control_dim> optConProblem(
                timeHorizon, x_init, oscillatorDynamics, this->costFunc, adLinearizer);

            // STEP 2-A: Create the settings.

            NLOptConSettings nloc_settings;
            nloc_settings.load(configDir + "/soc_nloc.info", true, "ilqr");
        
            // STEP 2-B: provide an initial guess
            N = nloc_settings.computeK(timeHorizon);
            FeedbackMatrix<state_dim, control_dim> u_fb;
            u_fb << 0.5, 1;
            FeedbackArray<state_dim, control_dim> u0_fb(N, -u_fb);
            ControlVectorArray<control_dim> u0_ff(N, ControlVector<control_dim>::Zero());
            StateVectorArray<state_dim> x_ref_init(N + 1, x_ref);
            NLOptConSolver<state_dim, control_dim>::Policy_t initController(x_ref_init, u0_ff, u0_fb, nloc_settings.dt);
            // How to create a more complicated controller for iterations?

            // STEP 2-C: create an NLOptConSolver instance
            NLOPPtr_t iLQR(new NLOptConSolver<state_dim, control_dim>(optConProblem, nloc_settings));
            this->nlop_problem = iLQR;

            this->nlop_problem->setInitialGuess(initController);

            // STEP 3: solve the optimal control problem
            this->nlop_problem->solve();

            // STEP 4: retrieve the solution

            // plotResultsOscillator<state_dim, control_dim>(x_ref_init,
            //                                             u0_fb,
            //                                             u0_ff,
            //                                             TimeArray(N + 1, timeHorizon)); 

            ct::core::StateFeedbackController<state_dim, control_dim> solution = this->nlop_problem->getSolution();

            /*  MPC-EXAMPLE
            * we store the initial solution obtained from solving the initial optimal control problem,
            * and re-use it to initialize the MPC solver in the following. */
            /* STEP 1: first, we set up an MPC instance for the iLQR solver and configure it. Since the MPC
            * class is wrapped around normal Optimal Control Solvers, we need to different kind of settings,
            * those for the optimal control solver, and those specific to MPC: */
            // 1) settings for the iLQR instance used in MPC. Of course, we use the same settings
            // as for solving the initial problem ...
            NLOptConSettings ilqr_settings_mpc = nloc_settings;
            // ... however, in MPC-mode, it makes sense to limit the overall number of iLQR iterations (real-time iteration scheme)
            ilqr_settings_mpc.max_iterations = 1;
            // and we limited the printouts, too.
            ilqr_settings_mpc.printSummary = true;
            // 2) settings specific to model predictive control. For a more detailed description of those, visit ct/optcon/mpc/MpcSettings.h
            ct::optcon::mpc_settings mpc_settings;
            mpc_settings.stateForwardIntegration_ = true;
            mpc_settings.postTruncation_ = true;
            mpc_settings.measureDelay_ = true;
            mpc_settings.delayMeasurementMultiplier_ = 1.0;
            mpc_settings.mpc_mode = ct::optcon::MPC_MODE::CONSTANT_RECEDING_HORIZON;
            mpc_settings.coldStart_ = false;
            // STEP 2 : Create the iLQR-MPC object, based on the optimal control problem and the selected settings.
            MPCPtr_t ilqr_mpc(new MPC<NLOptConSolver<state_dim, control_dim>>(optConProblem, ilqr_settings_mpc, mpc_settings));

            // initialize it using the previously computed initial controller
            this->mpc = ilqr_mpc;
            this->mpc->setInitialGuess(solution);


        }


        void pos_message_converter(const geometry_msgs::Point::ConstPtr & msg,
                                  ct::core::StateVector<SecondOrderSystem::STATE_DIM>& x)
            {

                x(0) = msg->x;
                x(1) = msg->y;
            }

        void pos_callback(const geometry_msgs::Point::ConstPtr & msg){
            if (first_pass_) {
                MPCController::pos_message_converter(msg, this->x_now);
                first_pass_ =  false;
                return;
            }
            if (controller_not_created_){
                return; // Only start to update when the mpc controller is created
            }

            MPCController::pos_message_converter(msg, this->x_now);
            
        }


        void pose_ref_callback(const geometry_msgs::Point::ConstPtr & msg){
            
            if (controller_not_created_){
                if (first_pass_){
                    return;
                }
                MPCController::pos_message_converter(msg, this->x_ref);
                MPCController::create_controller(this->x_now, this->x_ref);
                x_ref_current = this->x_ref;
                start_time = ros::Time::now().toSec();
                controller_not_created_ = false;
                return; 
            }
            MPCController::pos_message_converter(msg, this->x_ref);

            if (x_ref_current == this->x_ref){
                return;
            }
            else
            {
                // MPCController::create_controller(this->x_now, this->x_ref);
                this->termQuad_interm->updateReferenceState(this->x_ref);
                this->termQuad_final->updateReferenceState(this->x_ref);
                this->costFunc->addIntermediateTerm(this->termQuad_interm);
                this->costFunc->addFinalTerm(this->termQuad_final);
                x_ref_current = this->x_ref;
                start_time = ros::Time::now().toSec();
            }
                       

        }

       
        void publish_cmd_wrench(){

            if (controller_not_created_){
                return; // Only start to publish when the mpc controller is created
            }
            const size_t state_dim = SecondOrderSystem::STATE_DIM;
            const size_t control_dim = SecondOrderSystem::CONTROL_DIM;
        
            current_time = ros::Time::now().toSec();
            ct::core::Time t = current_time - start_time;
            this->mpc->prepareIteration(current_time);
            current_time = ros::Time::now().toSec();
            t = current_time - start_time;
            this->mpc->finishIteration(this->x_now, t, newPolicy, ts_newPolicy);
            current_time = ros::Time::now().toSec();
            t = current_time - start_time;
            ControlVector<control_dim> u;
            newPolicy.computeControl(this->x_now, t - ts_newPolicy, u);
            std::cout << "Time now: " << t << " " << ts_newPolicy << " Reference point: " << this->x_ref(0) << " " << this->x_ref(1) 
            << " Current point: "  << this->x_now(0) << " " << this->x_now(1) << " \n";

            // plotResultsOscillator<state_dim, control_dim>(newPolicy.x_ref(),
            //                                                 newPolicy.K(),
            //                                                 newPolicy.uff(),
            //                                                 newPolicy.time());

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
            ct::core::Time ts_newPolicy;


            ControllerPtr_t controller;
            LinearSystemPtr_t Linearizer;
            NLOPPtr_t nlop_problem;
            MPCPtr_t mpc; 
            PolicyPtr_t newPolicy;
            CostFuncPtr_t costFunc;
            TermPtr_t termQuad_interm, termQuad_final;
            size_t N;

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
    MPCController mpc_controller  = MPCController(&n);

    int rate = 50;
    ros::Rate r(rate);

    while (n.ok())

    {   
        ros::spinOnce();
        mpc_controller.publish_cmd_wrench();
        r.sleep();
        
    }
}

