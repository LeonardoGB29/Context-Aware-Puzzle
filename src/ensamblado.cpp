// ensamblado.cpp — implementaciones

#include "ensamblado.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>

int findSeed(vector<Piece>& pieces) {
    for (int i = 0; i < (int)pieces.size(); ++i)
        if (pieces[i].frameCount >= 2) return i;
    for (int i = 0; i < (int)pieces.size(); ++i)
        if (pieces[i].isBorder()) return i;
    return 0;
}

void anchorSeed(Piece& seed) {
    seed.placed = true;
    seed.pos = Point2f(0, 0);
    seed.rot = 0;
    for (Edge& e : seed.edges)
        if (e.isFrame) {
            Point2f d = e.b - e.a;
            seed.rot = -atan2(d.y, d.x);
            break;
        }
}

float texFactor(Edge& a, Edge& b) {
    return 1.0f - exp(-(gabor::chi2(a.band, b.band) + gTexTau));
}

bool coincide(Point2f a0, Point2f a1, Point2f b0, Point2f b1) {
    return cv::norm(a0 - b1) < COINC_TOL && cv::norm(a1 - b0) < COINC_TOL;
}

bool overlaps(vector<Piece>& pieces, int pb, float rot, Point2f pos) {
    vector<Point2f> gb;
    for (Point2f& v : pieces[pb].poly) gb.push_back(pos + rotP(v, rot));
    for (int a = 0; a < (int)pieces.size(); ++a) {
        if (!pieces[a].placed || a == pb) continue;
        vector<Point2f> ga;
        for (Point2f& v : pieces[a].poly) ga.push_back(xform(pieces[a], v));
        vector<Point2f> inter;
        float area = cv::intersectConvexConvex(gb, ga, inter, true);
        if (area > 0.05f * min(pieces[pb].area, pieces[a].area)) return true;
    }
    return false;
}

SeamEval evalPlacement(vector<Piece>& pieces, int pb, float rot, Point2f pos) {
    SeamEval ev;
    float sum = 0;
    for (int a = 0; a < (int)pieces.size(); ++a) {
        if (!pieces[a].placed || a == pb) continue;
        for (Edge& e : pieces[a].edges) {
            Point2f e0 = xform(pieces[a], e.a), e1 = xform(pieces[a], e.b);
            for (Edge& f : pieces[pb].edges) {
                Point2f f0 = pos + rotP(f.a, rot), f1 = pos + rotP(f.b, rot);
                if (!coincide(e0, e1, f0, f1)) continue;
                float c = matchCost(e, f);
                if (c > gMatchThresh * SEAM_RELAX) return SeamEval{};
                sum += c;
                ev.seams++;
            }
        }
    }
    if (ev.seams == 0) return ev;
    if (overlaps(pieces, pb, rot, pos)) return SeamEval{};
    ev.avg = sum / ev.seams;
    ev.ok = true;
    return ev;
}

bool frameOK(vector<Piece>& pieces, int pb, float rot, Point2f pos) {
    float ANG_TOL = 0.03f;
    float LINE_TOL = 4.0f;
    for (Edge& f : pieces[pb].edges) {
        if (!f.isFrame) continue;
        Point2f d = rotP(f.b - f.a, rot);
        if (fabs(remainder(atan2(d.y, d.x), (float)(CV_PI / 2))) > ANG_TOL)
            return false;

        bool horizontal = fabs(d.x) > fabs(d.y);
        Point2f mid = pos + rotP((f.a + f.b) * 0.5f, rot);
        float coord = horizontal ? mid.y : mid.x;

        bool anyParallel = false, okLine = false;
        for (Piece& q : pieces) {
            if (!q.placed) continue;
            for (Edge& e : q.edges) {
                if (!e.isFrame) continue;
                Point2f dq = rotP(e.b - e.a, q.rot);
                if ((fabs(dq.x) > fabs(dq.y)) != horizontal) continue;
                anyParallel = true;
                Point2f mq = xform(q, (e.a + e.b) * 0.5f);
                float diff = fabs(coord - (horizontal ? mq.y : mq.x));
                if (diff < LINE_TOL) okLine = true;
                else if (gRectW > 0 && (fabs(diff - gRectW) < LINE_TOL ||
                                        fabs(diff - gRectH) < LINE_TOL)) okLine = true;
            }
        }
        if (anyParallel && !okLine) return false;
    }
    return true;
}

void placeAt(vector<Piece>& pieces, int pb, float rot, Point2f pos) {
    Piece& B = pieces[pb];
    B.rot = rot;
    B.pos = pos;
    B.placed = true;
    for (int a = 0; a < (int)pieces.size(); ++a) {
        if (!pieces[a].placed || a == pb) continue;
        for (Edge& e : pieces[a].edges) {
            Point2f e0 = xform(pieces[a], e.a), e1 = xform(pieces[a], e.b);
            for (Edge& f : B.edges) {
                Point2f f0 = xform(B, f.a), f1 = xform(B, f.b);
                if (coincide(e0, e1, f0, f1)) { e.used = true; f.used = true; }
            }
        }
    }
}

vector<Cand> collectCandidates(vector<Piece>& pieces, bool borderOnly) {
    vector<Cand> out;
    for (int a = 0; a < (int)pieces.size(); ++a) {
        if (!pieces[a].placed) continue;
        for (int i = 0; i < (int)pieces[a].edges.size(); ++i) {
            if (pieces[a].edges[i].isFrame || pieces[a].edges[i].used) continue;
            for (int b = 0; b < (int)pieces.size(); ++b) {
                if (pieces[b].placed) continue;
                if (borderOnly && !pieces[b].isBorder()) continue;
                for (int j = 0; j < (int)pieces[b].edges.size(); ++j) {
                    float c = matchCost(pieces[a].edges[i], pieces[b].edges[j]);
                    if (c < gMatchThresh) out.push_back({a, i, b, j, c});
                }
            }
        }
    }
    return out;
}

void rigidOf(vector<Piece>& pieces, Cand& c, float& rot, Point2f& pos) {
    Edge& EA = pieces[c.pa].edges[c.ea];
    Edge& EB = pieces[c.pb].edges[c.eb];
    Point2f ga = xform(pieces[c.pa], EA.a), gb = xform(pieces[c.pa], EA.b);
    Point2f d0 = EB.b - EB.a, d1 = ga - gb;
    rot = atan2(d1.y, d1.x) - atan2(d0.y, d0.x);
    pos = gb - rotP(EB.a, rot);
}

void refineRigid(vector<Piece>& pieces, int pb, float& rot, Point2f& pos) {
    for (int iter = 0; iter < 2; ++iter) {
        vector<Point2f> src, dst;
        for (int a = 0; a < (int)pieces.size(); ++a) {
            if (!pieces[a].placed || a == pb) continue;
            for (Edge& e : pieces[a].edges) {
                Point2f e0 = xform(pieces[a], e.a), e1 = xform(pieces[a], e.b);
                for (Edge& f : pieces[pb].edges) {
                    Point2f f0 = pos + rotP(f.a, rot), f1 = pos + rotP(f.b, rot);
                    if (cv::norm(e0 - f1) < 2 * COINC_TOL && cv::norm(e1 - f0) < 2 * COINC_TOL) {
                        src.push_back(f.a); dst.push_back(e1);
                        src.push_back(f.b); dst.push_back(e0);
                    }
                }
            }
        }
        if (src.size() < 2) return;

        Point2f ms(0, 0), md(0, 0);
        for (int i = 0; i < (int)src.size(); ++i) { ms += src[i]; md += dst[i]; }
        ms *= 1.0f / src.size();
        md *= 1.0f / src.size();
        double s1 = 0, s2 = 0;
        for (int i = 0; i < (int)src.size(); ++i) {
            Point2f a = src[i] - ms, b = dst[i] - md;
            s1 += a.x * b.x + a.y * b.y;
            s2 += a.x * b.y - a.y * b.x;
        }
        rot = (float)atan2(s2, s1);
        pos = md - rotP(ms, rot);
    }
}

bool placeNext(vector<Piece>& pieces, bool borderOnly) {
    vector<Cand> cands = collectCandidates(pieces, borderOnly);
    if (cands.empty()) return false;

    map<long, int> bestOpen, bestFree;
    for (int k = 0; k < (int)cands.size(); ++k) {
        long ko = (long)cands[k].pa * 64 + cands[k].ea;
        long kf = (long)cands[k].pb * 64 + cands[k].eb;
        if (!bestOpen.count(ko) || cands[k].color < cands[bestOpen[ko]].color) bestOpen[ko] = k;
        if (!bestFree.count(kf) || cands[k].color < cands[bestFree[kf]].color) bestFree[kf] = k;
    }

    struct PassSpec { bool mutualOnly; int minSeams; };
    PassSpec passes[4] = {{true, 2}, {false, 2}, {true, 1}, {false, 1}};

    int bestK = -1;
    float bestRot = 0, bestScore = numeric_limits<float>::infinity();
    Point2f bestPos;
    for (int pi = 0; pi < 4; ++pi) {
        PassSpec ps = passes[pi];
        for (int k = 0; k < (int)cands.size(); ++k) {
            long ko = (long)cands[k].pa * 64 + cands[k].ea;
            long kf = (long)cands[k].pb * 64 + cands[k].eb;
            bool mutual = bestOpen[ko] == k && bestFree[kf] == k;
            if (ps.mutualOnly && !mutual) continue;
            float rot;
            Point2f pos;
            rigidOf(pieces, cands[k], rot, pos);
            refineRigid(pieces, cands[k].pb, rot, pos);
            if (!frameOK(pieces, cands[k].pb, rot, pos)) continue;
            SeamEval ev = evalPlacement(pieces, cands[k].pb, rot, pos);
            if (!ev.ok || ev.seams < ps.minSeams) continue;
            float score = ev.avg
                          * texFactor(pieces[cands[k].pa].edges[cands[k].ea],
                                      pieces[cands[k].pb].edges[cands[k].eb])
                          / (1.0f + 0.25f * (ev.seams - 1));
            if (score < bestScore) { bestScore = score; bestK = k; bestRot = rot; bestPos = pos; }
        }
        if (bestK >= 0) break;
    }

    if (bestK < 0) {
        vector<pair<float, int>> order;
        for (int k = 0; k < (int)cands.size(); ++k) order.push_back({cands[k].color, k});
        sort(order.begin(), order.end());
        for (int i = 0; i < (int)order.size(); ++i) {
            int k = order[i].second;
            float rot;
            Point2f pos;
            rigidOf(pieces, cands[k], rot, pos);
            refineRigid(pieces, cands[k].pb, rot, pos);
            if (!frameOK(pieces, cands[k].pb, rot, pos)) continue;
            if (overlaps(pieces, cands[k].pb, rot, pos)) continue;
            bestK = k; bestRot = rot; bestPos = pos;
            break;
        }
        if (bestK < 0) return false;
    }
    placeAt(pieces, cands[bestK].pb, bestRot, bestPos);
    return true;
}

void assemble(vector<Piece>& pieces, int seed) {
    anchorSeed(pieces[seed]);

    while (placeNext(pieces, /*borderonly=*/true)) {}

    int bTotal = 0, bPlaced = 0;
    for (Piece& p : pieces)
        if (p.isBorder()) { bTotal++; bPlaced += p.placed; }
    if (bTotal) cout << "marco armado: " << bPlaced << " / " << bTotal
                     << " piezas de borde\n";

    while (placeNext(pieces, /*borderonly=*/false)) {}

    int placed = 0;
    for (Piece& p : pieces) placed += p.placed;
    cout << "piezas colocadas: " << placed << " / " << (int)pieces.size() << "\n";
}
