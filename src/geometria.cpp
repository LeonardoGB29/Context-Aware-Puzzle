#include "geometria.hpp"
#include <algorithm>
#include <cmath>

Point2f rotP(Point2f p, float rot) {
    float c = cos(rot), s = sin(rot);
    return Point2f(c * p.x - s * p.y, s * p.x + c * p.y);
}

void ensureCCW(vector<Point2f>& poly) {
    double area = 0;
    for (size_t i = 0; i < poly.size(); ++i) {
        Point2f p = poly[i];
        Point2f q = poly[(i + 1) % poly.size()];
        area += (double)p.x * q.y - (double)q.x * p.y;
    }
    if (area < 0) reverse(poly.begin(), poly.end());
}

vector<Point2f> poissonDisk(float W, float H, float r, int k, float inset, cv::RNG& rng) {
    float cell = r / sqrt(2.0f);
    int gw = max(1, (int)ceil(W / cell));
    int gh = max(1, (int)ceil(H / cell));
    vector<int> grid(gw * gh, -1);
    vector<Point2f> samples;
    vector<int> active;

    Point2f first(rng.uniform(inset, W - inset), rng.uniform(inset, H - inset));
    grid[min(gh - 1, (int)(first.y / cell)) * gw + min(gw - 1, (int)(first.x / cell))] = 0;
    active.push_back(0);
    samples.push_back(first);

    while (!active.empty()) {
        int i = rng.uniform(0, (int)active.size());
        Point2f base = samples[active[i]];
        bool placed = false;
        for (int t = 0; t < k && !placed; ++t) {
            float ang = rng.uniform(0.0f, (float)(2 * CV_PI));
            float rad = rng.uniform(r, 2 * r);
            Point2f cand(base.x + cos(ang) * rad, base.y + sin(ang) * rad);

            if (cand.x < inset || cand.x > W - inset || cand.y < inset || cand.y > H - inset)
                continue;

            int gx = (int)(cand.x / cell), gy = (int)(cand.y / cell);
            bool lejos = true;
            for (int yy = max(0, gy - 2); yy <= min(gh - 1, gy + 2) && lejos; ++yy)
                for (int xx = max(0, gx - 2); xx <= min(gw - 1, gx + 2) && lejos; ++xx) {
                    int s = grid[yy * gw + xx];
                    if (s >= 0 && cv::norm(samples[s] - cand) < r) lejos = false;
                }
            if (!lejos) continue;

            grid[min(gh - 1, gy) * gw + min(gw - 1, gx)] = (int)samples.size();
            active.push_back((int)samples.size());
            samples.push_back(cand);
            placed = true;
        }
        if (!placed) active.erase(active.begin() + i);
    }
    return samples;
}

vector<Point2f> clipToRect(vector<Point2f>& poly, float W, float H) {
    struct HalfPlane { float nx, ny, d; };
    HalfPlane sides[4] = {
        {  1,  0,  0 }, { -1,  0, -W }, {  0,  1,  0 }, {  0, -1, -H },
    };
    vector<Point2f> out = poly;
    for (int s = 0; s < 4; ++s) {
        vector<Point2f> in = out;
        out.clear();
        if (in.empty()) break;
        for (size_t i = 0; i < in.size(); ++i) {
            Point2f cur = in[i], prev = in[(i + in.size() - 1) % in.size()];
            float sc = sides[s].nx * cur.x + sides[s].ny * cur.y - sides[s].d;
            float sp = sides[s].nx * prev.x + sides[s].ny * prev.y - sides[s].d;
            if (sc >= 0) {
                if (sp < 0) out.push_back(prev + (cur - prev) * (sp / (sp - sc)));
                out.push_back(cur);
            } else if (sp >= 0) {
                out.push_back(prev + (cur - prev) * (sp / (sp - sc)));
            }
        }
    }
    return out;
}

bool isFrameEdge(Point2f a, Point2f b, float W, float H) {
    float e = 0.75f;
    return (fabs(a.x) < e && fabs(b.x) < e) ||
           (fabs(a.x - W) < e && fabs(b.x - W) < e) ||
           (fabs(a.y) < e && fabs(b.y) < e) ||
           (fabs(a.y - H) < e && fabs(b.y - H) < e);
}

Point2f applyAffine(cv::Mat& M, Point2f q) {
    return Point2f((float)(M.at<double>(0, 0) * q.x + M.at<double>(0, 1) * q.y + M.at<double>(0, 2)),
                   (float)(M.at<double>(1, 0) * q.x + M.at<double>(1, 1) * q.y + M.at<double>(1, 2)));
}

void rotatePiece(cv::Mat& src, vector<Point2f>& poly, float deg,
                 cv::Mat& dst, vector<Point2f>& dstPoly) {
    Point2f c(src.cols * 0.5f, src.rows * 0.5f);
    cv::Mat M = cv::getRotationMatrix2D(c, deg, 1.0);

    vector<Point2f> corners = {{0, 0}, {(float)src.cols, 0},
                               {(float)src.cols, (float)src.rows}, {0, (float)src.rows}};
    float minx = 1e9f, miny = 1e9f, maxx = -1e9f, maxy = -1e9f;
    for (size_t i = 0; i < corners.size(); ++i) {
        Point2f t = applyAffine(M, corners[i]);
        minx = min(minx, t.x); maxx = max(maxx, t.x);
        miny = min(miny, t.y); maxy = max(maxy, t.y);
    }
    M.at<double>(0, 2) -= minx;
    M.at<double>(1, 2) -= miny;

    cv::warpAffine(src, dst, M, cv::Size((int)ceil(maxx - minx), (int)ceil(maxy - miny)),
                   cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0, 0));
    dstPoly.clear();
    for (size_t i = 0; i < poly.size(); ++i) dstPoly.push_back(applyAffine(M, poly[i]));
}
