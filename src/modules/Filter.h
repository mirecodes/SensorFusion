#ifndef FILTER_H
#define FILTER_H

#include <vector>

std::vector<float> highPassFilter(float alpha, const std::vector<float> &last, const std::vector<float> &current);
std::vector<float> lowPassFilter(float alpha, const std::vector<float> &last, const std::vector<float> &current);
std::vector<float> movingAverageFilter(const std::vector<std::vector<float>> &history, int windowSize);

#endif // FILTER_H
