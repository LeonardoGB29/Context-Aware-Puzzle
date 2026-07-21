// salida.cpp — implementaciones

#include "salida.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>

Point2f applyAlign(Align& al, Point2f g) { return rotP(g - al.mr, al.th) + al.mg; }

Align fitKabsch(vector<Point2f>& rec, vector<Point2f>& gt, vector<int>& idx) {
    Align al;
    for (int i : idx) { al.mr += rec[i]; al.mg += gt[i]; }
    al.mr *= 1.0f / idx.size();
    al.mg *= 1.0f / idx.size();
    double s1 = 0, s2 = 0;
    for (int i : idx) {
        Point2f a = rec[i] - al.mr, b = gt[i] - al.mg;
        s1 += a.x * b.x + a.y * b.y;
        s2 += a.x * b.y - a.y * b.x;
    }
    al.th = (float)atan2(s2, s1);
    return al;
}

Align alignToGT(vector<Piece>& pieces, bool hasGT) {
    Align al;
    if (!hasGT) return al;
    vector<Point2f> rec, gt;
    for (Piece& p : pieces) {
        if (!p.placed) continue;
        rec.push_back(xform(p, p.centroid));
        gt.push_back(p.gtC);
    }
    if (rec.size() < 3) return al;

    vector<int> idx(rec.size());
    for (int i = 0; i < (int)rec.size(); ++i) idx[i] = i;
    vector<float> res(rec.size());
    for (int iter = 0; iter < 3; ++iter) {
        al = fitKabsch(rec, gt, idx);
        for (int i = 0; i < (int)rec.size(); ++i)
            res[i] = (float)cv::norm(applyAlign(al, rec[i]) - gt[i]);
        vector<float> sorted = res;
        sort(sorted.begin(), sorted.end());
        float cut = max((float)(2 * PLACE_TOL), sorted[sorted.size() / 2]);
        idx.clear();
        for (int i = 0; i < (int)rec.size(); ++i)
            if (res[i] < cut) idx.push_back(i);
        if (idx.size() < 3) break;
    }
    al.ok = true;
    return al;
}

cv::Mat render(vector<Piece>& pieces, Align& al, int imgW, int imgH) {
    Point2f shift(0, 0);
    int W, H;
    if (al.ok && imgW > 0) {
        W = imgW;
        H = imgH;
    } else {
        float minx = 1e9f, miny = 1e9f, maxx = -1e9f, maxy = -1e9f;
        for (Piece& p : pieces) {
            if (!p.placed) continue;
            for (Edge& e : p.edges) {
                Point2f g = xform(p, e.a);
                minx = min(minx, g.x); maxx = max(maxx, g.x);
                miny = min(miny, g.y); maxy = max(maxy, g.y);
            }
        }
        W = (int)ceil(maxx - minx) + 2;
        H = (int)ceil(maxy - miny) + 2;
        shift = Point2f(minx, miny);
    }
    cv::Mat canvas = cv::Mat::zeros(H, W, CV_8UC3);
    bool useAlign = al.ok && imgW > 0;

    for (Piece& p : pieces) {
        if (!p.placed) continue;

        float bx0 = 1e9f, by0 = 1e9f, bx1 = -1e9f, by1 = -1e9f;
        for (Edge& e : p.edges) {
            Point2f g = xform(p, e.a);
            if (useAlign) g = applyAlign(al, g);
            else g = g - shift;
            bx0 = min(bx0, g.x); bx1 = max(bx1, g.x);
            by0 = min(by0, g.y); by1 = max(by1, g.y);
        }
        int x0 = max(0, (int)floor(bx0) - 1), x1 = min(W - 1, (int)ceil(bx1) + 1);
        int y0 = max(0, (int)floor(by0) - 1), y1 = min(H - 1, (int)ceil(by1) + 1);

        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x) {
                Point2f c((float)x, (float)y);
                Point2f g;
                if (useAlign) g = rotP(c - al.mg, -al.th) + al.mr;
                else g = c + shift;
                Point2f local = rotP(g - p.pos, -p.rot);
                int ix = cvRound(local.x), iy = cvRound(local.y);
                if (ix < 0 || iy < 0 || ix >= p.img.cols || iy >= p.img.rows) continue;
                cv::Vec4b s = p.img.at<cv::Vec4b>(iy, ix);
                if (s[3] < 64) continue;
                canvas.at<cv::Vec3b>(y, x) = cv::Vec3b(s[0], s[1], s[2]);
            }
    }
    return canvas;
}

void reportAccuracy(vector<Piece>& pieces, Align& al) {
    if (!al.ok) {
        cout << "sin ground truth (modo b): no se mide precision\n";
        return;
    }
    int correct = 0, total = 0;
    for (Piece& p : pieces) {
        if (!p.placed) continue;
        total++;
        if (cv::norm(applyAlign(al, xform(p, p.centroid)) - p.gtC) < PLACE_TOL) correct++;
    }
    cout << "precision (bien ubicadas): " << correct << " / " << total << "\n";
}

void writeAnalysis(vector<Piece>& pieces, string& dir) {
    for (Piece& p : pieces) {
        cv::Mat out(p.img.rows, p.img.cols, CV_8UC4, cv::Scalar(0, 0, 0, 0));
        for (int y = 0; y < p.img.rows; ++y)
            for (int x = 0; x < p.img.cols; ++x) {
                int t = p.texMap.at<int>(y, x);
                if (t < 0) continue;
                cv::Vec3b c = PALETTE[t % 12];
                out.at<cv::Vec4b>(y, x) = cv::Vec4b(c[0], c[1], c[2], 255);
            }
        char name[64];
        snprintf(name, sizeof(name), "piece_%03d.png", p.id);
        cv::imwrite(dir + "/" + name, out);
    }
}
