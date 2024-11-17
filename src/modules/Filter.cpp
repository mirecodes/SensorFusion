#include "Filter.h"

std::vector<float> highPassFilter(float beta, const std::vector<float> &lastInput, const std::vector<float> &currentInput, const std::vector<float> &lastOutput)
{
    /*
    Pass the signal with high frequency
    y[n] = beta * (y[n−1] + x[n] − x[n−1])
    <Requirements>
    - coefficient = beta
    - last input = x[i-1]
    - last output = y[i-1]
    - current input = x[i]
    */
    std::vector<float> result(currentInput.size(), 0.0f);

    for (int i = 0; i < currentInput.size(); i++)
    {
        result[i] = beta * (lastOutput[i] + currentInput[i] - lastInput[i]);
    }

    return result;
}

std::vector<float> lowPassFilter(float alpha, const std::vector<float> &last, const std::vector<float> &current)
{
    /*
    Pass the signal with low frequency
    y[n] = alpha * x[i] + (1-alpha) * y[i-1]
    <Requirements>
    - coefficient = alpha
    - last output = y[i-1]
    - current input = x[i]
    */
    std::vector<float> result(current.size(), 0.0);

    for (int i = 0; i < current.size(); i++)
    {
        result[i] = alpha * current[i] + (1.0 - alpha) * last[i];
    }

    return result;
}

std::vector<float> movingAverageFilter(const std::vector<std::vector<float>> &history, int windowSize)
{
    /*
    Return the averaged signal
    */
    if (history.empty() || history[0].empty())
        return {};

    int dimensions = history[0].size();
    std::vector<float> result(dimensions, 0.0);

    for (const auto &vec : history)
    {
        for (int i = 0; i < dimensions; i++)
        {
            result[i] += vec[i];
        }
    }

    for (int i = 0; i < dimensions; i++)
    {
        result[i] /= static_cast<float>(windowSize);
    }

    return result;
}
