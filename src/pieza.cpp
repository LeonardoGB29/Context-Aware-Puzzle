// pieza.cpp — variables globales e implementaciones

#include "pieza.hpp"
#include <opencv2/core/utils/filesystem.hpp>
#include <iostream>
#include <limits>
#include <cmath>

float gMatchThresh = MATCH_THRESH;
float gTexTau = TEX_TAU;
float gRectW = 0, gRectH = 0;

const cv::Vec3b PALETTE[12] = {
    {60, 60, 220}, {60, 200, 60}, {220, 140, 40}, {200, 60, 200},
    {40, 200, 220}, {140, 100, 60}, {230, 230, 60}, {120, 120, 120},
    {40, 120, 240}, {160, 60, 120}, {90, 200, 160}, {200, 200, 200},
};

Point2f xform(Piece& P, Point2f local) { return P.pos + rotP(local, P.rot); }

bool colorAt(cv::Mat& bgra, Point2f p, Vec3f& out) {
    int x0 = (int)floor(p.x), y0 = (int)floor(p.y);
    float fx = p.x - x0, fy = p.y - y0;
    Vec3f acc(0, 0, 0);
    float wsum = 0;
    for (int dy = 0; dy <= 1; ++dy)
        for (int dx = 0; dx <= 1; ++dx) {
            int x = x0 + dx, y = y0 + dy;
            if (x < 0 || y < 0 || x >= bgra.cols || y >= bgra.rows) continue;
            cv::Vec4b px = bgra.at<cv::Vec4b>(y, x);
            if (px[3] < 128) continue;
            float w = (dx ? fx : 1 - fx) * (dy ? fy : 1 - fy);
            acc += Vec3f(px[0], px[1], px[2]) * w;
            wsum += w;
        }
    if (wsum < 0.25f) return false;
    out = acc * (1.0f / wsum);
    return true;
}

vector<Vec3f> sampleStrip(cv::Mat& bgra, Point2f a, Point2f b, Point2f centroid) {
    Point2f d = b - a;
    float L = (float)cv::norm(d) + 1e-6f;
    Point2f n(-d.y / L, d.x / L);
    Point2f mid = (a + b) * 0.5f;
    if (n.dot(centroid - mid) < 0) n = -n;

    vector<Vec3f> strip;
    for (int k = 0; k < K_SAMPLES; ++k) {
        float t = 0.12f + (0.76f * k) / (K_SAMPLES - 1);
        Point2f base = a + d * t;
        Vec3f c(0, 0, 0);
        for (float e = INSET_PX; e <= INSET_PX + 6.0f; e += 1.5f)
            if (colorAt(bgra, base + n * e, c)) break;
        strip.push_back(c);
    }
    return strip;
}

float stripCost(Edge& a, Edge& b) {
    double acc = 0;
    for (int k = 0; k < K_SAMPLES; ++k) {
        Vec3f diff = a.strip[k] - b.strip[K_SAMPLES - 1 - k];
        acc += sqrt(diff.dot(diff));
    }
    return (float)(acc / K_SAMPLES);
}

float matchCost(Edge& a, Edge& b) {
    float INF = numeric_limits<float>::infinity();
    if (a.isFrame || b.isFrame || a.used || b.used) return INF;
    if (fabs(a.len - b.len) > max(4.0f, LEN_TOL * max(a.len, b.len))) return INF;
    return stripCost(a, b);
}

vector<Point2f> polyFromAlpha(cv::Mat& bgra) {
    vector<cv::Mat> ch;
    cv::split(bgra, ch);
    cv::Mat mask = ch[3] >= 128;

    vector<vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return {};

    int best = 0;
    double bestArea = 0;
    for (int i = 0; i < (int)contours.size(); ++i) {
        double a = cv::contourArea(contours[i]);
        if (a > bestArea) { bestArea = a; best = i; }
    }
    vector<cv::Point> approx;
    cv::approxPolyDP(contours[best], approx, 2.5, true);

    vector<Point2f> poly(approx.begin(), approx.end());
    ensureCCW(poly);
    return poly;
}

void buildEdges(Piece& p, vector<Point2f>& poly, const vector<int>& frame) {
    Point2f c(0, 0);
    for (Point2f& v : poly) c += v;
    p.centroid = c * (1.0f / poly.size());
    p.poly = poly;
    p.area = (float)cv::contourArea(poly);

    for (int j = 0; j < (int)poly.size(); ++j) {
        Edge e;
        e.a = poly[j];
        e.b = poly[(j + 1) % poly.size()];
        e.len = (float)cv::norm(e.b - e.a);
        e.isFrame = j < (int)frame.size() && frame[j] != 0;
        e.strip = sampleStrip(p.img, e.a, e.b, p.centroid);
        if (e.isFrame) p.frameCount++;
        p.edges.push_back(e);
    }
}

vector<Piece> loadPieces(string& dir, int& imgW, int& imgH, bool& modeA) {
    vector<Piece> pieces;
    if (!cv::utils::fs::exists(dir)) return pieces;
    cv::FileStorage fs(dir + "/pieces.yml", cv::FileStorage::READ);
    modeA = fs.isOpened();

    if (modeA) {
        imgW = (int)fs["image_width"];
        imgH = (int)fs["image_height"];
        for (const cv::FileNode& n : fs["pieces"]) {
            Piece p;
            p.id = (int)n["id"];
            p.img = cv::imread(dir + "/" + (string)n["file"], cv::IMREAD_UNCHANGED);
            if (p.img.empty()) continue;

            vector<float> flat;
            n["poly"] >> flat;
            vector<Point2f> poly;
            for (int i = 0; i + 1 < (int)flat.size(); i += 2)
                poly.emplace_back(flat[i], flat[i + 1]);
            vector<int> frame;
            n["frame"] >> frame;

            buildEdges(p, poly, frame);
            pieces.push_back(p);
        }
    } else {
        imgW = imgH = 0;
        vector<cv::String> files;
        cv::glob(dir + "/piece_*.png", files, false);
        for (int i = 0; i < (int)files.size(); ++i) {
            Piece p;
            p.id = i;
            p.img = cv::imread(files[i], cv::IMREAD_UNCHANGED);
            if (p.img.empty()) continue;

            vector<Point2f> poly = polyFromAlpha(p.img);
            if (poly.size() < 3) continue;
            buildEdges(p, poly, {});
            pieces.push_back(p);
        }
    }
    return pieces;
}

bool loadGroundTruth(string& dir, vector<Piece>& pieces) {
    cv::FileStorage fs(dir + "/ground_truth.yml", cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    for (const cv::FileNode& n : fs["pieces"]) {
        int id = (int)n["id"];
        for (Piece& p : pieces)
            if (p.id == id) p.gtC = Point2f((float)n["cx"], (float)n["cy"]);
    }
    return true;
}

void classifyByTexture(vector<Piece>& pieces) {
    vector<cv::Mat> bank = gabor::makeBank();
    vector<cv::Mat> imgs;
    for (Piece& p : pieces) imgs.push_back(p.img);

    // el contexto primero: kmeans usa el rng global y el orden cambia los centros
    cv::Mat ctxCenters = gabor::trainTextons(
        gabor::collectSamples(imgs, bank, 0), N_TEX_CTX);
    cv::Mat visCenters = gabor::trainTextons(
        gabor::collectSamples(imgs, bank, gabor::SMOOTH_SIGMA), N_TEX_VIS);
    int kv = visCenters.rows, kc = ctxCenters.rows;

    for (Piece& p : pieces) {
        p.texMap = gabor::majorityFilter(
            gabor::textonMap(p.img, bank, visCenters, gabor::SMOOTH_SIGMA), kv, 4, 2);
        cv::Mat ctxMap = gabor::textonMap(p.img, bank, ctxCenters, 0);
        p.tex = gabor::histogram(ctxMap, kc);
        for (Edge& e : p.edges) {
            e.band = gabor::bandHistogram(ctxMap, e.a, e.b, kc, BAND_PX);
            float s = 0;
            for (float v : e.band) s += v;
            if (s <= 0) e.band = p.tex;
        }
    }
    cout << "textones: " << kv << " para ver + " << kc << " para el matching\n";
}
