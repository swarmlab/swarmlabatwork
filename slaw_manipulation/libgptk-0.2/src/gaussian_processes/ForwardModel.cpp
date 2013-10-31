#include "ForwardModel.h"

/**
 * Default constructor
 *
 * @param Inputs   The dimension of the input space
 * @param Outputs  The dimension of the output space
 */
ForwardModel::ForwardModel(int Inputs, int Outputs)
{
	inputDimensions = Inputs;
	outputDimensions = Outputs;
}

/**
 * Default destructor
 */
ForwardModel::~ForwardModel()
{
}

/**
 * Returns the dimension of the input space
 *
 * @return the dimension of the input space
 */
int ForwardModel::getInputDimensions() const
{
	return inputDimensions;
}

/**
 * Returns the dimension of the output space
 *
 * @return
 */
int ForwardModel::getOutputDimensions() const
{
	return outputDimensions;
}
