// gabor.hpp — textura por textones (declaraciones)

#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;

namespace gabor {

const double SMOOTH_SIGMA = 8.0;   // suavizado del mapa visual
const int ORIENTATIONS = 6;        // orientaciones del banco

void sortOrientations(vector<float>& feat);

vector<cv::Mat> makeBank();

cv::Mat innerMask(cv::Mat& bgra);

vector<cv::Mat> responses(cv::Mat& bgra, vector<cv::Mat>& bank, double sigma);

cv::Mat collectSamples(vector<cv::Mat>& imgs, vector<cv::Mat>& bank, double sigma);

cv::Mat trainTextons(const cv::Mat& samples, int k);

cv::Mat textonMap(cv::Mat& bgra, vector<cv::Mat>& bank,
                  cv::Mat& centers, double sigma);

cv::Mat majorityFilter(const cv::Mat& map, int k, int radius, int passes);

vector<float> histogram(cv::Mat& map, int k);

vector<float> bandHistogram(cv::Mat& map, cv::Point2f a, cv::Point2f b,
                            int k, float bandPx);

float chi2(vector<float>& a, vector<float>& b);

}  // namespace gabor
