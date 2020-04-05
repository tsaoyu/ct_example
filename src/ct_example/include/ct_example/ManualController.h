#include <ct/optcon/optcon.h>
#include <vector>

using namespace ct::core;
using namespace ct::optcon;

template <size_t STATE_DIM, size_t CONTROL_DIM, typename SCALAR = double>
class ManualController : public Controller<STATE_DIM, CONTROL_DIM>{

    public:

        ManualController(){this->control_vals = {};};
        ManualController(const ManualController& other):
        control_vals(other.control_vals)
        {};
        ~ManualController() {};

        ManualController* clone() const {
            return new ManualController();
        };

        void updateControl(const std::vector<double>& msg){
            this -> control_vals = msg;
        };

        void computeControl(const StateVector<STATE_DIM, SCALAR>& state,
        const double& t,
        ControlVector<CONTROL_DIM, SCALAR>& controlAction){
            int j = 0;
            for (auto i: this->control_vals){
                controlAction(j) = i;
                j += 1; 
            };
        };

    private:
        std::vector<double> control_vals;
};