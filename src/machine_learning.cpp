#include "machine_learning.hpp"

NeuralNetwork::NeuralNetwork() {
    // get last from sd card OR generate new 
    for (int i = 0; i < NEURAL_NETWORK_LAYERS; i++) {
        for (int j = 0; j < NEURAL_NETWORK_NUM_INPUTS; j++) {
            for (int k = 0; k < NEURAL_NETWORK_NUM_INPUTS; k++) {
                weights[i][j][k] = random(-1000, 1000);
                prevWeights[i][j][k] = weights[i][j][k];
            }
            biases[i][j] = random(-1000, 1000);
            prevBiases[i][j] = biases[i][j];
        }
    }
}

float* NeuralNetwork::evaluate(float inputs[NEURAL_NETWORK_NUM_INPUTS]) {
    for (uint8_t i = 0; i < NEURAL_NETWORK_LAYERS; i++) {
        for (uint8_t j = 0; j < NEURAL_NETWORK_NUM_INPUTS; j++) {
            int32_t sum = 0;
            for (uint8_t k = 0; k < NEURAL_NETWORK_NUM_INPUTS; k++) {
                sum += weights[i][j][k] * inputs[k];
            }
            sum += biases[i][j];
            inputs[j] = reLU(sum);
        }
    }
    float outputs[NEURAL_NETWORK_NUM_OUTPUTS];
    for (uint8_t i = 0; i < NEURAL_NETWORK_NUM_OUTPUTS; i++) {
        int32_t sum = 0;
        for (uint8_t j = 0; j < NEURAL_NETWORK_NUM_INPUTS; j++) {
            sum += outputWeights[i][j] * inputs[j];
        }
        sum += outputBiases[i];
        outputs[i] = (float) sum / 1000;
    }
    return outputs;
}