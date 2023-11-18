#include <stdint.h>
#include <random>

#define NEURAL_NETWORK_INPUTS 8
#define NEURAL_NETWORK_OUTPUTS 4
#define NEURAL_NETWORK_LAYERS 2
#define NEURAL_NETWORK_LAYER_SIZE 8

#define MAX_CURRENT_FRONT 60.0
#define MAX_CURRENT_REAR 90.0

class NeuralNetwork {
private:
    //for 1st layer
    float inputWeights[NEURAL_NETWORK_LAYER_SIZE][NEURAL_NETWORK_INPUTS];
    float inputBiases[NEURAL_NETWORK_LAYER_SIZE];
    //for other internal layers
    float internalWeights[NEURAL_NETWORK_LAYERS-1][NEURAL_NETWORK_LAYER_SIZE][NEURAL_NETWORK_LAYER_SIZE];
    float internalBiases[NEURAL_NETWORK_LAYERS-1][NEURAL_NETWORK_LAYER_SIZE];
    //for outputs
    float outputWeights[NEURAL_NETWORK_OUTPUTS][NEURAL_NETWORK_LAYER_SIZE];
    float outputBiases[NEURAL_NETWORK_OUTPUTS];
public:
    NeuralNetwork();
    NeuralNetwork(int id);
    void save(int id); // save to sd card
    float* evaluate(float inputs[NEURAL_NETWORK_INPUTS]);
    // void mutate(float mutation_rate);
    // float fitness();
};

float reLU (float x);

float random(float min, float max);
