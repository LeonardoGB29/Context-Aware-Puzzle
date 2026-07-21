// geometria.hpp — geometria compartida por generator y solver (declaraciones)

#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using cv::Point2f;

Point2f rotP(Point2f p, float rot);

void ensureCCW(vector<Point2f>& poly);

vector<Point2f> poissonDisk(float W, float H, float r, int k, float inset, cv::RNG& rng);

vector<Point2f> clipToRect(vector<Point2f>& poly, float W, float H);

bool isFrameEdge(Point2f a, Point2f b, float W, float H);

Point2f applyAffine(cv::Mat& M, Point2f q);

void rotatePiece(cv::Mat& src, vector<Point2f>& poly, float deg,
                 cv::Mat& dst, vector<Point2f>& dstPoly);
