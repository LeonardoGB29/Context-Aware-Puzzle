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

bool boundsOK(vector<Piece>& pieces, int pb, float rot, Point2f pos) {
    if (gRectW <= 0) return true;
    float minx = 1e9f, miny = 1e9f, maxx = -1e9f, maxy = -1e9f;
    for (int a = 0; a < (int)pieces.size(); ++a) {
        if (a != pb && !pieces[a].placed) continue;
        for (Point2f& v : pieces[a].poly) {
            Point2f g = (a == pb) ? pos + rotP(v, rot) : xform(pieces[a], v);
            minx = min(minx, g.x); maxx = max(maxx, g.x);
            miny = min(miny, g.y); maxy = max(maxy, g.y);
        }
    }
    float tol = 8.0f;
    return (maxx - minx) <= gRectW + tol && (maxy - miny) <= gRectH + tol;
}

SeamEval evalPlacement(vector<Piece>& pieces, int pb, float rot, Point2f pos, bool gate) {
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
                if (gate && c > gMatchThresh * SEAM_RELAX) return SeamEval{};
                sum += isfinite(c) ? c : gMatchThresh;   // color solo para el promedio
                ev.seams++;
            }
        }
    }
    if (ev.seams == 0) return ev;
    if (overlaps(pieces, pb, rot, pos)) return SeamEval{};
    if (!boundsOK(pieces, pb, rot, pos)) return SeamEval{};
    ev.avg = sum / ev.seams;
    ev.ok = true;
    return ev;
}

float globalEnergy(vector<Piece>& pieces) {
    float E = 0;
    for (int a = 0; a < (int)pieces.size(); ++a) {
        if (!pieces[a].placed) { E += gMatchThresh; continue; }
        for (int b = a + 1; b < (int)pieces.size(); ++b) {
            if (!pieces[b].placed) continue;
            for (Edge& e : pieces[a].edges) {
                if (e.isFrame) continue;
                Point2f e0 = xform(pieces[a], e.a), e1 = xform(pieces[a], e.b);
                for (Edge& f : pieces[b].edges) {
                    if (f.isFrame) continue;
                    Point2f f0 = xform(pieces[b], f.a), f1 = xform(pieces[b], f.b);
                    if (coincide(e0, e1, f0, f1))
                        E += stripCost(e, f) - gMatchThresh;
                }
            }
        }
    }
    return E;
}

float placedAvgCost(vector<Piece>& pieces, int b) {
    float sum = 0;
    int n = 0;
    for (int a = 0; a < (int)pieces.size(); ++a) {
        if (!pieces[a].placed || a == b) continue;
        for (Edge& e : pieces[a].edges) {
            if (e.isFrame) continue;
            Point2f e0 = xform(pieces[a], e.a), e1 = xform(pieces[a], e.b);
            for (Edge& f : pieces[b].edges) {
                if (f.isFrame) continue;
                Point2f f0 = xform(pieces[b], f.a), f1 = xform(pieces[b], f.b);
                if (coincide(e0, e1, f0, f1)) { sum += stripCost(e, f); n++; }
            }
        }
    }
    return n ? sum / n : -1.0f;
}

void removePiece(vector<Piece>& pieces, int pb) {
    Piece& B = pieces[pb];
    for (int a = 0; a < (int)pieces.size(); ++a) {
        if (!pieces[a].placed || a == pb) continue;
        for (Edge& e : pieces[a].edges) {
            Point2f e0 = xform(pieces[a], e.a), e1 = xform(pieces[a], e.b);
            for (Edge& f : B.edges) {
                Point2f f0 = xform(B, f.a), f1 = xform(B, f.b);
                if (coincide(e0, e1, f0, f1)) e.used = false;
            }
        }
    }
    B.placed = false;
    for (Edge& f : B.edges) f.used = false;
}

bool repairWorst(vector<Piece>& pieces) {
    // peor pieza colocada: costo medio de costura por encima del umbral
    int worst = -1;
    float worstAvg = gMatchThresh;
    for (int b = 0; b < (int)pieces.size(); ++b) {
        if (!pieces[b].placed) continue;
        float avg = placedAvgCost(pieces, b);
        if (avg > worstAvg) { worstAvg = avg; worst = b; }
    }
    if (worst < 0) return false;

    float E0 = globalEnergy(pieces);
    float oldRot = pieces[worst].rot;
    Point2f oldPos = pieces[worst].pos;
    removePiece(pieces, worst);

    // mejor recolocacion con el resto fijo (color sin compuerta)
    int bestSeams = 0;
    float bestScore = numeric_limits<float>::infinity(), bestRot = 0;
    Point2f bestPos;
    for (int a = 0; a < (int)pieces.size(); ++a) {
        if (!pieces[a].placed) continue;
        for (int i = 0; i < (int)pieces[a].edges.size(); ++i) {
            Edge& EA = pieces[a].edges[i];
            if (EA.isFrame || EA.used) continue;
            for (int j = 0; j < (int)pieces[worst].edges.size(); ++j) {
                Edge& EB = pieces[worst].edges[j];
                if (EB.isFrame) continue;
                if (fabs(EA.len - EB.len) > max(4.0f, LEN_TOL * max(EA.len, EB.len))) continue;
                Cand c{a, i, worst, j, 0};
                float rot;
                Point2f pos;
                rigidOf(pieces, c, rot, pos);
                refineRigid(pieces, worst, rot, pos);
                if (!frameOK(pieces, worst, rot, pos)) continue;
                SeamEval ev = evalPlacement(pieces, worst, rot, pos, false);
                if (!ev.ok) continue;
                float score = ev.avg / (1.0f + 0.25f * (ev.seams - 1));
                if (score < bestScore) {
                    bestScore = score; bestSeams = ev.seams;
                    bestRot = rot; bestPos = pos;
                }
            }
        }
    }
    if (bestSeams > 0) {
        placeAt(pieces, worst, bestRot, bestPos);
        if (globalEnergy(pieces) < E0 - 1e-3f) return true;   // solo si mejora global
        removePiece(pieces, worst);
    }
    placeAt(pieces, worst, oldRot, oldPos);   // volver como estaba
    return false;
}

vector<Pose> savePoses(vector<Piece>& pieces) {
    vector<Pose> st;
    for (Piece& p : pieces) {
        Pose s;
        s.placed = p.placed ? 1 : 0;
        s.rot = p.rot;
        s.pos = p.pos;
        for (Edge& e : p.edges) s.used.push_back(e.used ? 1 : 0);
        st.push_back(s);
    }
    return st;
}

void restorePoses(vector<Piece>& pieces, vector<Pose>& st) {
    for (int i = 0; i < (int)pieces.size(); ++i) {
        pieces[i].placed = st[i].placed != 0;
        pieces[i].rot = st[i].rot;
        pieces[i].pos = st[i].pos;
        for (int j = 0; j < (int)pieces[i].edges.size(); ++j)
            pieces[i].edges[j].used = st[i].used[j] != 0;
    }
}

bool evictAndFill(vector<Piece>& pieces) {
    float E0 = globalEnergy(pieces);
    vector<Pose> snap = savePoses(pieces);

    for (int u = 0; u < (int)pieces.size(); ++u) {
        if (pieces[u].placed) continue;
        // pose candidata desde cualquier arista colocada, aunque este usada
        for (int a = 0; a < (int)pieces.size(); ++a) {
            if (!pieces[a].placed || a == u) continue;
            for (int i = 0; i < (int)pieces[a].edges.size(); ++i) {
                Edge& EA = pieces[a].edges[i];
                if (EA.isFrame) continue;
                for (int j = 0; j < (int)pieces[u].edges.size(); ++j) {
                    Edge& EB = pieces[u].edges[j];
                    if (fabs(EA.len - EB.len) > max(4.0f, LEN_TOL * max(EA.len, EB.len))) continue;
                    Cand c{a, i, u, j, 0};
                    float rot;
                    Point2f pos;
                    rigidOf(pieces, c, rot, pos);
                    if (!frameOK(pieces, u, rot, pos)) continue;

                    // ocupantes: piezas colocadas que se solapan con esa pose
                    vector<Point2f> gu;
                    for (Point2f& v : pieces[u].poly) gu.push_back(pos + rotP(v, rot));
                    int victim = -1, nvic = 0;
                    for (int b = 0; b < (int)pieces.size() && nvic < 2; ++b) {
                        if (!pieces[b].placed || b == u) continue;
                        vector<Point2f> gb;
                        for (Point2f& v : pieces[b].poly) gb.push_back(xform(pieces[b], v));
                        vector<Point2f> inter;
                        float area = cv::intersectConvexConvex(gu, gb, inter, true);
                        if (area > 0.05f * min(pieces[u].area, pieces[b].area)) { victim = b; nvic++; }
                    }
                    if (nvic != 1) continue;   // solo desalojo simple

                    removePiece(pieces, victim);
                    refineRigid(pieces, u, rot, pos);
                    SeamEval ev = evalPlacement(pieces, u, rot, pos, false);
                    if (!ev.ok || ev.seams < 2) { restorePoses(pieces, snap); continue; }
                    placeAt(pieces, u, rot, pos);
                    while (fillHole(pieces)) {}   // la victima puede reubicarse
                    if (globalEnergy(pieces) < E0 - 1e-3f) return true;
                    restorePoses(pieces, snap);   // no mejoro: deshacer todo
                }
            }
        }
    }
    return false;
}

// fuerza bruta: cada pieza suelta contra cada arista abierta; gana la que cierra
// mas costuras geometricamente (color desempata). No usa la compuerta de color.
bool fillHole(vector<Piece>& pieces) {
    int bestB = -1, bestSeams = 0;
    float bestColor = numeric_limits<float>::infinity(), bestRot = 0;
    Point2f bestPos;
    for (int b = 0; b < (int)pieces.size(); ++b) {
        if (pieces[b].placed) continue;
        for (int a = 0; a < (int)pieces.size(); ++a) {
            if (!pieces[a].placed) continue;
            for (int i = 0; i < (int)pieces[a].edges.size(); ++i) {
                Edge& EA = pieces[a].edges[i];
                if (EA.isFrame || EA.used) continue;
                for (int j = 0; j < (int)pieces[b].edges.size(); ++j) {
                    Edge& EB = pieces[b].edges[j];
                    if (fabs(EA.len - EB.len) > max(4.0f, LEN_TOL * max(EA.len, EB.len))) continue;
                    Cand c{a, i, b, j, 0};
                    float rot;
                    Point2f pos;
                    rigidOf(pieces, c, rot, pos);
                    refineRigid(pieces, b, rot, pos);
                    if (!frameOK(pieces, b, rot, pos)) continue;
                    SeamEval ev = evalPlacement(pieces, b, rot, pos, false);
                    if (!ev.ok || ev.seams < 2) continue;   // hueco real: 2+ lados
                    if (ev.seams > bestSeams ||
                        (ev.seams == bestSeams && ev.avg < bestColor)) {
                        bestSeams = ev.seams; bestColor = ev.avg;
                        bestB = b; bestRot = rot; bestPos = pos;
                    }
                }
            }
        }
    }
    if (bestB < 0) return false;
    placeAt(pieces, bestB, bestRot, bestPos);
    return true;
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
            SeamEval ev = evalPlacement(pieces, cands[k].pb, rot, pos, true);
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
            if (!boundsOK(pieces, cands[k].pb, rot, pos)) continue;
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

    int greedy = 0;
    for (Piece& p : pieces) greedy += p.placed;

    while (fillHole(pieces)) {}   // fuerza bruta sobre las que quedaron sueltas

    int rep = 0;                  // descenso de energia global (idea mrf del paper)
    while (rep < 8 && (repairWorst(pieces) || evictAndFill(pieces))) {
        rep++;
        while (fillHole(pieces)) {}
    }
    if (rep) cout << "reparacion por energia: " << rep << " movimientos\n";

    int placed = 0;
    for (Piece& p : pieces) placed += p.placed;
    if (placed > greedy)
        cout << "relleno de huecos: +" << (placed - greedy) << " por geometria\n";
    cout << "piezas colocadas: " << placed << " / " << (int)pieces.size() << "\n";
}
