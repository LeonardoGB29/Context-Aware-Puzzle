// pieza.hpp — modelo de la pieza: parametros, structs y declaraciones

#pragma once
#include <opencv2/opencv.hpp>
#include "geometria.hpp"
#include "gabor.hpp"
#include <string>
#include <vector>

using namespace std;
using cv::Point2f;
using cv::Vec3f;

const int    K_SAMPLES    = 16;     // muestras de color por arista
const float  INSET_PX     = 2.5f;   // profundidad de la tira desde la arista
const float  LEN_TOL      = 0.10f;  // tolerancia de longitud entre aristas
const float  MATCH_THRESH = 45.0f;  // costo de color maximo aceptable
const int    N_TEX_VIS    = 6;      // textones para visualizar
const int    N_TEX_CTX    = 12;     // textones para el contexto del matching
const float  BAND_PX      = 12.0f;  // ancho de la banda de contexto
const float  TEX_TAU      = 0.15f;  // offset del descuento de contexto
const float  SEAM_RELAX   = 1.3f;   // holgura de costuras secundarias
const float  COINC_TOL    = 3.0f;   // px para que dos aristas coincidan
const double PLACE_TOL    = 8.0;    // px de tolerancia vs ground truth

extern float gMatchThresh;          // argv[2] del solver
extern float gTexTau;               // argv[1] del solver (99 = sin textura)
extern float gRectW, gRectH;        // dimensiones del marco (modo a)
extern const cv::Vec3b PALETTE[12]; // colores del analisis por textones

struct Edge {
    Point2f a, b;                // extremos locales (ccw)
    float   len = 0;
    bool    isFrame = false;     // arista sobre el marco
    bool    used = false;        // ya cosida
    vector<Vec3f> strip;         // tira de color interior
    vector<float> band;          // histograma de textones de su banda
};

struct Piece {
    int id = 0;
    cv::Mat img;                 // recorte bgra (rotado)
    vector<Edge> edges;
    vector<Point2f> poly;        // poligono local (ccw, convexo)
    float area = 0;
    Point2f centroid{0, 0};
    int  frameCount = 0;
    bool placed = false;
    float rot = 0;               // colocacion: global = pos + R(rot) * local
    Point2f pos{0, 0};
    Point2f gtC{0, 0};           // centroide original (ground truth)
    cv::Mat texMap;              // mapa de textones por pixel
    vector<float> tex;           // histograma de textones de la pieza

    bool isBorder() { return frameCount > 0; }
};

Point2f xform(Piece& P, Point2f local);

bool colorAt(cv::Mat& bgra, Point2f p, Vec3f& out);

vector<Vec3f> sampleStrip(cv::Mat& bgra, Point2f a, Point2f b, Point2f centroid);

float stripCost(Edge& a, Edge& b);

float matchCost(Edge& a, Edge& b);

vector<Point2f> polyFromAlpha(cv::Mat& bgra);

void buildEdges(Piece& p, vector<Point2f>& poly, const vector<int>& frame);

vector<Piece> loadPieces(string& dir, int& imgW, int& imgH, bool& modeA);

bool loadGroundTruth(string& dir, vector<Piece>& pieces);

void classifyByTexture(vector<Piece>& pieces);
