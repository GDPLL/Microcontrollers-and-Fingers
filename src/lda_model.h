// Auto-generated LDA model for 2-channel EMG
#ifndef LDA_MODEL_H
#define LDA_MODEL_H

#define LDA_N_CLASSES 3
#define LDA_N_FEATURES 2

const float LDA_COEF[3][2] = {
    {-2.382487f, -0.024566f},
    {-1.755239f, -1.326071f},
    {4.137726f, 1.350637f},
};

const float LDA_INTERCEPT[3] = {-1.778594f, -2.164468f, -4.460650f};

const float FEATURE_MEAN[LDA_N_FEATURES] = {0.000026f, 0.000005f};
const float FEATURE_STD[LDA_N_FEATURES] = {0.000034f, 0.000003f};

#endif
