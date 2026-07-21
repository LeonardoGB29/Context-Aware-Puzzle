// salida.hpp — alineacion global, render, precision y analisis (declaraciones)

#pragma once
#include "pieza.hpp"

struct Align { bool ok = false; float th = 0; Point2f mr{0, 0}, mg{0, 0}; };

Point2f applyAlign(Align& al, Point2f g);

Align fitKabsch(vector<Point2f>& rec, vector<Point2f>& gt, vector<int>& idx);

Align alignToGT(vector<Piece>& pieces, bool hasGT);

cv::Mat render(vector<Piece>& pieces, Align& al, int imgW, int imgH);

void reportAccuracy(vector<Piece>& pieces, Align& al);

void writeAnalysis(vector<Piece>& pieces, string& dir);
