#include "NeuralController.h"
#include <OpenSim/OpenSim.h>

namespace OpenSim {

	NeuralController::NeuralController(const std::string& controlsFileName, int interpMethodType = 1)
		: Controller(), _num_functions(0) {
		constructProperties();
	}

	NeuralController::NeuralController()
		: Controller(), _num_functions(0) {
		constructProperties();
	}

	NeuralController::~NeuralController() {}

	void NeuralController::setNull()
	{
		setAuthors("Hugh Osborne from code by Sergio Verduzco from code by Ajay Seth");
	}

	void NeuralController::constructProperties()
	{
		constructProperty_InputFunctions(FunctionSet());
	}

	void NeuralController::extendConnectToModel(Model& model)
	{
		Super::extendConnectToModel(model);
		// Probably would be better to call this in some init function which
		// I think there is but can't be bothered to check.
		preamble();
	}

	void NeuralController::computeControls(const SimTK::State& s, SimTK::Vector& controls) const 
	{
		// Calculate the functions which define the inputs
		SimTK::Vector time(1, s.getTime());
		std::vector<double> inputs;
		for (int i = 0; i < get_InputFunctions().getSize(); i++) {
			_value_name_to_activation[_function_id_to_value_name[i]] = get_InputFunctions()[i].calcValue(time);
		}

		for (int i = 0; i < getActuatorSet().getSize(); i++) {
			std::string muscle_name = _actuator_id_muscle_name[i];
			SimTK::Vector actControls(3, 0.0);
			std::string value_name = muscle_name + std::string("_in_alpha");
			actControls[0] = _value_name_to_activation[value_name] / 50.0;
			value_name = muscle_name + std::string("_in_gamma");
			actControls[1] = _value_name_to_activation[value_name];
			value_name = muscle_name + std::string("_in_beta");
			actControls[2] = _value_name_to_activation[value_name];
			getActuatorSet()[i].addInControls(actControls, controls);
		}
	}

	void NeuralController::associateFunctionWithAlphaInput(std::string muscle_name, Function* function) {
		associateFunctionWithValue(muscle_name, std::string("_in_alpha"), function);
	}

	void NeuralController::associateFunctionWithBetaInput(std::string muscle_name, Function* function) {
		associateFunctionWithValue(muscle_name, std::string("_in_beta"), function);
	}

	void NeuralController::associateFunctionWithGammaInput(std::string muscle_name, Function* function) {
		associateFunctionWithValue(muscle_name, std::string("_in_gamma"), function);
	}

	void NeuralController::associateFunctionWithValue(std::string muscle_name, std::string appended_name, Function* function) {
		_num_functions++;
		upd_InputFunctions().setSize(_num_functions);
		upd_InputFunctions().set(_num_functions - 1, function);
		std::string value_name = muscle_name + appended_name;
		_function_id_to_value_name[_num_functions - 1] = value_name;
		_value_name_to_activation[value_name] = 0.0;
	}

	void NeuralController::prescribeControlForActuator(const std::string actName) {
		int index = getProperty_actuator_list().findIndex(actName);
		if (index < 0)
			throw Exception("NeuralController does not have " + actName + " in its list of actuators to control.");
		_actuator_id_muscle_name[index] = actName;
	}

	void NeuralController::preamble() {
	}
}

