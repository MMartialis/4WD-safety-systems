#include <stdint.h>

#define NEURAL_NETWORK_NUM_INPUTS 8
#define NEURAL_NETWORK_NUM_OUTPUTS 4
#define NEURAL_NETWORK_LAYERS 4

class NeuralNetwork {
private: 
    int16_t weights[NEURAL_NETWORK_LAYERS][NEURAL_NETWORK_NUM_INPUTS][NEURAL_NETWORK_NUM_INPUTS];
    int16_t prevWeights[NEURAL_NETWORK_LAYERS][NEURAL_NETWORK_NUM_INPUTS][NEURAL_NETWORK_NUM_INPUTS];
    int16_t outputWeights[NEURAL_NETWORK_NUM_OUTPUTS][NEURAL_NETWORK_NUM_INPUTS];
    int16_t prevOutputWeights[NEURAL_NETWORK_NUM_OUTPUTS][NEURAL_NETWORK_NUM_INPUTS];
    int16_t biases[NEURAL_NETWORK_LAYERS][NEURAL_NETWORK_NUM_INPUTS];
    int16_t prevBiases[NEURAL_NETWORK_LAYERS][NEURAL_NETWORK_NUM_INPUTS];
    int16_t outputBiases[NEURAL_NETWORK_NUM_OUTPUTS];
    int16_t prevOutputBiases[NEURAL_NETWORK_NUM_OUTPUTS];
public:
    NeuralNetwork();
    float* evaluate(float inputs[NEURAL_NETWORK_NUM_INPUTS]);
    float* evaluate(int16_t inputs[NEURAL_NETWORK_NUM_INPUTS]);
    void mutate(float mutation_rate);
    float fitness();
};

int16_t reLU (int32_t x);

int16_t random(int16_t min, int16_t max);
