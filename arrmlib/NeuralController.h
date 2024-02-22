#ifndef INCLUDE_GUARD_NEURALCONTROLLER
#define INCLUDE_GUARD_NEURALCONTROLLER

#include <string>
#include <OpenSim/Common/FunctionSet.h>
#include <OpenSim/Simulation/Control/Controller.h>

namespace OpenSim {

	class NeuralController : public Controller {
		OpenSim_DECLARE_CONCRETE_OBJECT(NeuralController, Controller);
	public:

		OpenSim_DECLARE_PROPERTY(InputFunctions, FunctionSet,
			"Functions describing the inputs to the network"
			"specified for this controller.");

		NeuralController();

		NeuralController(const std::string& controlsFileName, int interpMethodType);

		virtual ~NeuralController();

		void computeControls(const SimTK::State& s,
			SimTK::Vector& controls) const OVERRIDE_11;

		void associateFunctionWithAlphaInput(std::string muscle_name, Function* function);
		void associateFunctionWithBetaInput(std::string muscle_name, Function* function);
		void associateFunctionWithGammaInput(std::string muscle_name, Function* function);
		void associateFunctionWithValue(std::string muscle_name, std::string appended_name, Function* function);

		void prescribeControlForActuator(const std::string actName);

	public:
		/** Model component interface */
		void extendConnectToModel(Model& model) OVERRIDE_11;
	private:
		// construct and initialize properties
		void constructProperties();

		void preamble();

		void setNull();

		mutable std::map<std::string, double> _value_name_to_activation;
		mutable std::map<int, std::string> _actuator_id_muscle_name;
		mutable std::map<unsigned int, std::string> _function_id_to_value_name;

		unsigned int _num_functions;
	};
}
#endif