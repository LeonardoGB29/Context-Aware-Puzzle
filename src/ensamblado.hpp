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

// el armado nunca puede exceder el lienzo conocido (el marco cierra un rectangulo)
bool boundsOK(vector<Piece>& pieces, int pb, float rot, Point2f pos);

// gate=true rechaza costuras con mal color; gate=false solo mide (para rellenar huecos)
SeamEval evalPlacement(vector<Piece>& pieces, int pb, float rot, Point2f pos, bool gate);

bool frameOK(vector<Piece>& pieces, int pb, float rot, Point2f pos);

void placeAt(vector<Piece>& pieces, int pb, float rot, Point2f pos);

vector<Cand> collectCandidates(vector<Piece>& pieces, bool borderOnly);

void rigidOf(vector<Piece>& pieces, Cand& c, float& rot, Point2f& pos);

void refineRigid(vector<Piece>& pieces, int pb, float& rot, Point2f& pos);

bool placeNext(vector<Piece>& pieces, bool borderOnly);

// pasada final por fuerza bruta: coloca una pieza suelta en el hueco donde encaje
// geometricamente (2+ costuras); el color solo desempata, no descarta
bool fillHole(vector<Piece>& pieces);

// energia global del armado (idea mrf del paper): costuras buenas restan,
// malas suman, piezas sueltas castigan
float globalEnergy(vector<Piece>& pieces);

// costo medio de color de las costuras realizadas de una pieza colocada
float placedAvgCost(vector<Piece>& pieces, int b);

// quita una pieza colocada y libera sus costuras
void removePiece(vector<Piece>& pieces, int pb);

// un paso de descenso: recoloca la peor pieza solo si baja la energia global
bool repairWorst(vector<Piece>& pieces);

// foto del armado para poder deshacer un intento
struct Pose { int placed; float rot; Point2f pos; vector<int> used; };

vector<Pose> savePoses(vector<Piece>& pieces);

void restorePoses(vector<Piece>& pieces, vector<Pose>& st);

// desalojo: mete una pieza suelta en su lugar geometrico expulsando al ocupante,
// rellena, y se queda solo si baja la energia global
bool evictAndFill(vector<Piece>& pieces);

void assemble(vector<Piece>& pieces, int seed);
