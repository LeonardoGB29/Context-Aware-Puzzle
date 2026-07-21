// solver.cpp — rearma el rompecabezas
// entrada: data/pieces/2_piezas_rotadas/ (modo a: con pieces.yml, modo b: solo pngs)
// salida:  data/pieces/3_analisis_coloreo/ y data/resultado.png

#include "ensamblado.hpp"
#include "salida.hpp"
#include <opencv2/core/utils/filesystem.hpp>
#include <iostream>

int main(int argc, char** argv) {
    if (argc > 1) gTexTau = stof(argv[1]);       // peso de textura (99 = sin textura)
    if (argc > 2) gMatchThresh = stof(argv[2]);  // umbral de color
    string dirRot = "data/pieces/2_piezas_rotadas";
    string dirAna = "data/pieces/3_analisis_coloreo";

    int imgW = 0, imgH = 0;
    bool modeA = false;
    vector<Piece> pieces = loadPieces(dirRot, imgW, imgH, modeA);
    if (pieces.empty()) {
        cerr << "no encontre piezas en " << dirRot << "/\n"
             << "  modo a: corre ./build/generator (genera piezas + metadata)\n"
             << "  modo b: pon tus piece_*.png (bgra con alpha) en esa carpeta\n";
        return 1;
    }
    bool hasGT = loadGroundTruth(dirRot, pieces);
    gRectW = (float)imgW;   // dimensiones del marco para frameOK (0 en modo b)
    gRectH = (float)imgH;
    cout << "piezas cargadas: " << (int)pieces.size()
         << (modeA ? "  (modo a: con metadata)" : "  (modo b: geometria desde alpha)") << "\n";

    classifyByTexture(pieces);
    cv::utils::fs::createDirectory(dirAna);
    writeAnalysis(pieces, dirAna);

    int seed = findSeed(pieces);
    cout << "semilla: pieza " << pieces[seed].id
         << " (aristas de marco: " << pieces[seed].frameCount << ")\n";

    assemble(pieces, seed);

    Align al = alignToGT(pieces, hasGT);
    cv::imwrite("data/resultado.png", render(pieces, al, imgW, imgH));
    reportAccuracy(pieces, al);
    cout << "analisis por pieza: " << dirAna << "/\n"
         << "resultado: data/resultado.png\n";
    return 0;
}
