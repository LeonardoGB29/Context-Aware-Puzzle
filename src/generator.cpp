#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include "geometria.hpp"
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

using namespace std;
using cv::Point2f;

struct RawPiece {
    vector<Point2f> poly;   // vertices locales (ccw)
    vector<int>     frame;  // 1 si la arista j yace en el marco
    cv::Mat         bgra;    // recorte con alpha
    cv::Point       gt;      // posicion original (ground truth)
};

RawPiece cutPiece(cv::Mat& img, vector<Point2f>& poly,
                         cv::Rect& bb) {
    vector<cv::Point> local;
    for (Point2f& p : poly)
        local.emplace_back(cvRound(p.x - bb.x), cvRound(p.y - bb.y));

    cv::Mat mask = cv::Mat::zeros(bb.height, bb.width, CV_8UC1);
    cv::fillConvexPoly(mask, local, cv::Scalar(255), cv::LINE_AA);

    cv::Mat bgra;
    cv::cvtColor(img(bb), bgra, cv::COLOR_BGR2BGRA);
    vector<cv::Mat> ch;
    cv::split(bgra, ch);
    ch[3] = mask;
    cv::merge(ch, bgra);

    RawPiece rp;
    for (Point2f& p : poly) rp.poly.emplace_back(p.x - bb.x, p.y - bb.y);
    for (int j = 0; j < (int)poly.size(); ++j)
        rp.frame.push_back(isFrameEdge(poly[j], poly[(j + 1) % poly.size()],
                                       img.cols, img.rows) ? 1 : 0);
    rp.bgra = bgra;
    rp.gt = bb.tl();
    return rp;
}

vector<RawPiece> buildPieces(cv::Mat& img,
                                    vector<vector<Point2f>>& facets,
                                    cv::Mat& preview) {
    float W = (float)img.cols, H = (float)img.rows;
    vector<RawPiece> pieces;
    for (int fi = 0; fi < (int)facets.size(); ++fi) {
        vector<Point2f> poly = clipToRect(facets[fi], W, H);
        if (poly.size() < 3) continue;
        ensureCCW(poly);

        cv::Rect bb = cv::boundingRect(poly) & cv::Rect(0, 0, img.cols, img.rows);
        if (bb.width < 2 || bb.height < 2) continue;

        vector<cv::Point> outline;
        for (Point2f& p : poly) outline.emplace_back(cvRound(p.x), cvRound(p.y));
        cv::polylines(preview, outline, true, {0, 0, 0}, 1, cv::LINE_AA);

        pieces.push_back(cutPiece(img, poly, bb));
    }
    return pieces;
}

void shufflePieces(vector<RawPiece>& pieces, cv::RNG& rng) {
    for (int i = (int)pieces.size() - 1; i > 0; --i)
        swap(pieces[i], pieces[rng.uniform(0, i + 1)]);
}

void writePieces(string& dir1, string& dir2,
                        vector<RawPiece>& pieces, int imgW, int imgH, cv::RNG& rng) {
    cv::FileStorage meta(dir2 + "/pieces.yml", cv::FileStorage::WRITE);
    cv::FileStorage gt(dir2 + "/ground_truth.yml", cv::FileStorage::WRITE);

    meta << "image_width" << imgW << "image_height" << imgH;
    meta << "count" << (int)pieces.size() << "pieces" << "[";
    gt << "pieces" << "[";
    for (int id = 0; id < (int)pieces.size(); ++id) {
        RawPiece& p = pieces[id];
        char name[64];
        snprintf(name, sizeof(name), "piece_%03d.png", id);

        cv::imwrite(dir1 + "/" + name, p.bgra);

        float angle = rng.uniform(0.0f, 360.0f);
        cv::Mat rot;
        vector<Point2f> rotPoly;
        rotatePiece(p.bgra, p.poly, angle, rot, rotPoly);
        cv::imwrite(dir2 + "/" + name, rot);

        meta << "{" << "id" << id << "file" << name;
        meta << "poly" << "[";
        for (Point2f& v : rotPoly) meta << v.x << v.y;
        meta << "]";
        meta << "frame" << "[";
        for (int f : p.frame) meta << f;
        meta << "]" << "}";

        Point2f c(0, 0);
        for (Point2f& v : p.poly) c += v;
        c *= 1.0f / p.poly.size();
        gt << "{" << "id" << id << "x" << p.gt.x << "y" << p.gt.y
           << "cx" << (p.gt.x + c.x) << "cy" << (p.gt.y + c.y)
           << "angle" << angle << "}";
    }
    meta << "]";
    gt << "]";
}

string findInputImage(string& dir) {
    vector<cv::String> files;
    cv::glob(dir + "/*", files, false);
    for (string& f : files) {
        string low = f;
        transform(low.begin(), low.end(), low.begin(), ::tolower);
        if (low.find("cut_preview") != string::npos) continue;
        if (low.find("resultado") != string::npos) continue;
        if (low.find("clasificacion") != string::npos) continue;
        for (const char* ext : {".png", ".jpg", ".jpeg", ".bmp"})
            if (low.size() >= strlen(ext) &&
                low.compare(low.size() - strlen(ext), strlen(ext), ext) == 0)
                return f;
    }
    return "";
}

cv::Mat syntheticImage(int W = 800, int H = 600) {
    cv::Mat img(H, W, CV_8UC3);
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            uchar b = (uchar)(128 + 100 * sin(x * 0.02) + 25 * sin(y * 0.13 + x * 0.05));
            uchar g = (uchar)(128 + 100 * sin(y * 0.025 + 1.0) + 25 * sin(x * 0.11));
            uchar r = (uchar)(128 + 100 * sin((x + y) * 0.015 + 2.0) + 25 * cos(y * 0.09));
            img.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r);
        }
    return img;
}

cv::Mat loadInput(string& dataDir, string& path) {
    if (path.empty()) path = findInputImage(dataDir);
    return cv::imread(path, cv::IMREAD_COLOR);
}

int main(int argc, char** argv) {
    string baseDir = (argc > 1) ? argv[1] : "data";   // carpeta del ejemplo (data/imagenN)
    float r = (argc > 2) ? stof(argv[2]) : 90.0f;
    string piecesDir = baseDir + "/pieces";
    string dir1 = piecesDir + "/1_piezas_cortadas";
    string dir2 = piecesDir + "/2_piezas_rotadas";
    cv::utils::fs::createDirectory(baseDir);
    cv::utils::fs::createDirectory(piecesDir);
    cv::utils::fs::createDirectory(dir1);
    cv::utils::fs::createDirectory(dir2);

    string imgPath = "";   // se busca la imagen de referencia dentro de baseDir
    cv::Mat img = loadInput(baseDir, imgPath);
    if (img.empty()) {
        cerr << "no pude cargar la imagen: " << imgPath << "\n";
        return 1;
    }
    cout << "entrada: " << imgPath << "  (" << img.cols << "x" << img.rows << ")\n";

    cv::RNG rng(12345);
    vector<Point2f> seeds = poissonDisk(img.cols, img.rows, r, 30, 2.0f, rng);
    cv::Subdiv2D subdiv(cv::Rect(0, 0, img.cols, img.rows));
    for (Point2f& s : seeds) subdiv.insert(s);
    vector<vector<Point2f>> facets;
    vector<Point2f> centers;
    subdiv.getVoronoiFacetList({}, facets, centers);

    cv::Mat preview = img.clone();
    vector<RawPiece> pieces = buildPieces(img, facets, preview);
    cv::imwrite(baseDir + "/cut_preview.png", preview);
    shufflePieces(pieces, rng);
    writePieces(dir1, dir2, pieces, img.cols, img.rows, rng);

    cout << "piezas generadas: " << (int)pieces.size() << "\n"
         << "  " << baseDir << "/cut_preview.png\n"
         << "  " << dir1 << "/  (piezas sin rotar)\n"
         << "  " << dir2 << "/  (piezas rotadas + pieces.yml + ground_truth.yml)\n"
         << "ahora corre:  ./build/solver " << baseDir << "\n";
    return 0;
}
