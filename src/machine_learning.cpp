#include "machine_learning.hpp"

NeuralNetwork::NeuralNetwork() {
    // get last from sd card OR generate new 
    for (int i = 0; i < NEURAL_NETWORK_LAYERS; i++) {
    }
}

float* NeuralNetwork::evaluate(float inputs[NEURAL_NETWORK_INPUTS]) {
    float* layer1 = new float[NEURAL_NETWORK_LAYER_SIZE];
    float* layer2 = new float[NEURAL_NETWORK_LAYER_SIZE];
    for (int i = 0; i < NEURAL_NETWORK_LAYER_SIZE; i++) { // for each neuron in the first layer
        float sum = 0;
        for (int j = 0; j < NEURAL_NETWORK_INPUTS; j++) { //     for each input compute the activation
            sum += inputs[j] * inputWeights[i][j];
        }
        layer1[i] = reLU(sum + inputBiases[i]);
    }
    for (int i = 0; i < NEURAL_NETWORK_LAYERS - 1; i++) { // for each internal layer
        const float* inputs = i == 0 ? layer1 : layer2;
        float* outputs = i == 0 ? layer2 : layer1;
        for (int j = 0; j < NEURAL_NETWORK_LAYER_SIZE; j++) { // for each neuron in the layer
            float sum = 0;
            for (int k = 0; k < NEURAL_NETWORK_LAYER_SIZE; k++) { // for each activation in the previous layer compute the activation
                sum += inputs[k] * internalWeights[i][j][k];
            }
            outputs[i] = reLU(sum + internalBiases[i][j]);
        }
    }
    for (int i = 0; i < NEURAL_NETWORK_OUTPUTS; i++) {
        const float* inputs = NEURAL_NETWORK_LAYERS % 2 == 0 ? layer1 : layer2;
        float* outputs = new float[NEURAL_NETWORK_OUTPUTS];
        float sum = 0;
        for (int j = 0; j < NEURAL_NETWORK_LAYER_SIZE; j++) {
            sum += outputs[j] * outputWeights[i][j];
        }
        outputs[i] = reLU(sum + outputBiases[i]);
    }
    return NEURAL_NETWORK_OUTPUTS % 2 == 0 ? layer1 : layer2;
}

float reLU(float x) {
    return x > 0 ? x : 0;
}

float random(float min, float max) {
    return min + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (max - min)));
}