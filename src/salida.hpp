#pragma once
#include "pieza.hpp"

struct Align { bool ok = false; float th = 0; Point2f mr{0, 0}, mg{0, 0}; };

Point2f applyAlign(Align& al, Point2f g);

Align fitKabsch(vector<Point2f>& rec, vector<Point2f>& gt, vector<int>& idx);

Align alignToGT(vector<Piece>& pieces, bool hasGT);

// coloreo=false: la foto real; coloreo=true: cada pixel pintado por su texton
cv::Mat render(vector<Piece>& pieces, Align& al, int imgW, int imgH, bool coloreo);

void reportAccuracy(vector<Piece>& pieces, Align& al);

// toda cara interior debe quedar emparejada; lo que sobra delata un error
void reportPairing(vector<Piece>& pieces);

void writeAnalysis(vector<Piece>& pieces, string& dir);
