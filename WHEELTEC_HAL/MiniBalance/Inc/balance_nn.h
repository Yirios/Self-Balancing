/* Auto-generated linear model — 8→2, 18 params */
#ifndef BALANCE_NN_H
#define BALANCE_NN_H

#define NN_INPUT_DIM  8
#define NN_OUTPUT_DIM 2

// Linear layer: output = W @ input + b
static const float nn_w[16] = {-77.77504730f, 6.82568121f, 5480.87353516f, -18866.74609375f, -96.84576416f, -11.26691723f, -445.98208618f, -2956.42968750f, 6.52765942f, -77.56392670f, 5484.37646484f, -18883.43359375f, -11.56652737f, -96.63586426f, -446.39227295f, -2958.38647461f};
static const float nn_b[2] = {0.00000000f, 0.00000000f};

static inline void nn_predict(const float* input, float* output) {
    for (int i = 0; i < 2; ++i) {
        float sum = nn_b[i];
        for (int j = 0; j < 8; ++j)
            sum += nn_w[i * 8 + j] * input[j];
        output[i] = sum;
    }
}

#endif