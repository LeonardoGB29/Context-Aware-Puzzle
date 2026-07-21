// ensamblado.hpp — armado del rompecabezas (declaraciones)

#pragma once
#include "pieza.hpp"

struct SeamEval {
    int seams = 0;
    float avg = 0;
    bool ok = false;
};

struct Cand { int pa, ea, pb, eb; float color; };

int findSeed(vector<Piece>& pieces);

void anchorSeed(Piece& seed);

float texFactor(Edge& a, Edge& b);

bool coincide(Point2f a0, Point2f a1, Point2f b0, Point2f b1);

bool overlaps(vector<Piece>& pieces, int pb, float rot, Point2f pos);

SeamEval evalPlacement(vector<Piece>& pieces, int pb, float rot, Point2f pos);

bool frameOK(vector<Piece>& pieces, int pb, float rot, Point2f pos);

void placeAt(vector<Piece>& pieces, int pb, float rot, Point2f pos);

vector<Cand> collectCandidates(vector<Piece>& pieces, bool borderOnly);

void rigidOf(vector<Piece>& pieces, Cand& c, float& rot, Point2f& pos);

void refineRigid(vector<Piece>& pieces, int pb, float& rot, Point2f& pos);

bool placeNext(vector<Piece>& pieces, bool borderOnly);

void assemble(vector<Piece>& pieces, int seed);
