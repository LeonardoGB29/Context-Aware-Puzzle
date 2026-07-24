#include "ensamblado.hpp"
#include "salida.hpp"
#include <opencv2/core/utils/filesystem.hpp>
#include <iostream>

// carpeta con piezas dentro de baseDir (prioridad: rotadas > cortadas)
string pickPiecesDir(string base) {
    string cand[4] = {
        base + "/pieces/2_piezas_rotadas",
        base + "/pieces/cortadas",
        base + "/pieces/1_piezas_cortadas",
        base + "/pieces",
    };
    for (int i = 0; i < 4; ++i) {
        if (!cv::utils::fs::exists(cand[i])) continue;
        vector<cv::String> f;
        cv::glob(cand[i] + "/*.png", f, false);
        if (!f.empty()) return cand[i];
    }
    return cand[0];
}

int main(int argc, char** argv) {
    string baseDir = (argc > 1) ? argv[1] : "data";  // carpeta del ejemplo
    string dirPiezas = (argc > 2 && argv[2][0]) ? argv[2] : pickPiecesDir(baseDir);
    if (argc > 3) gTexTau = stof(argv[3]);           // peso de textura (99 = sin textura)
    if (argc > 4) gMatchThresh = stof(argv[4]);      // umbral de color
    string dirAna = baseDir + "/pieces/3_analisis_coloreo";

    int imgW = 0, imgH = 0;
    bool modeA = false;
    vector<Piece> pieces = loadPieces(dirPiezas, imgW, imgH, modeA);
    if (pieces.empty()) {
        cerr << "no encontre piezas en " << dirPiezas << "/\n"
             << "  modo a: corre ./build/generator " << baseDir << " (genera piezas + metadata)\n"
             << "  modo b: pon tus piezas (png con alpha) en " << baseDir << "/pieces/cortadas/\n";
        return 1;
    }
    if (!modeA) {   // modo b: el lienzo lo da la imagen de referencia si existe
        vector<cv::String> files;
        cv::glob(baseDir + "/*", files, false);
        for (string& f : files) {
            string low = f;
            transform(low.begin(), low.end(), low.begin(), ::tolower);
            if (low.find("referencia") == string::npos) continue;
            cv::Mat ref = cv::imread(f);
            if (!ref.empty()) { imgW = ref.cols; imgH = ref.rows; }
            break;
        }
        if (imgW > 0) cout << "lienzo por referencia: " << imgW << "x" << imgH
                           << " (el armado no puede excederlo)\n";
    }
    bool hasGT = loadGroundTruth(dirPiezas, pieces);
    gRectW = (float)imgW;   // cota del rectangulo (frameOK y boundsOK)
    gRectH = (float)imgH;
    cout << "piezas cargadas: " << (int)pieces.size() << " desde " << dirPiezas
         << (modeA ? "  (modo a: con metadata)" : "  (modo b: geometria desde alpha)") << "\n";

    bool desdeColoreo = (dirPiezas == dirAna);   // se estan armando las piezas coloreadas
    classifyByTexture(pieces);
    if (!desdeColoreo) {                         // no pisar el analisis si es la entrada
        cv::utils::fs::createDirectory(dirAna);
        writeAnalysis(pieces, dirAna);
    }

    // solo diagnostico: el conteo de parejas no decide colocaciones (medido: empeora)
    countPartners(pieces);
    int forced = 0, frameE = 0;
    for (Piece& p : pieces)
        for (Edge& e : p.edges) {
            if (e.isFrame) frameE++;
            else if (e.partners == 1) forced++;
        }
    cout << "caras: " << forced << " con pareja unica, " << frameE << " de marco\n";

    int seed = findSeed(pieces);
    cout << "semilla: pieza " << pieces[seed].id
         << " (aristas de marco: " << pieces[seed].frameCount << ")\n";

    assemble(pieces, seed);

    Align al = alignToGT(pieces, hasGT);
    string salida = baseDir + (desdeColoreo ? "/resultado_desde_coloreo.png" : "/resultado.png");
    cv::imwrite(salida, render(pieces, al, imgW, imgH, false));
    if (!desdeColoreo)
        cv::imwrite(baseDir + "/resultado_coloreo.png", render(pieces, al, imgW, imgH, true));
    reportAccuracy(pieces, al);
    reportPairing(pieces);
    cout << "resultado: " << salida << "\n";
    if (!desdeColoreo)
        cout << "resultado coloreo: " << baseDir << "/resultado_coloreo.png\n"
             << "analisis por pieza: " << dirAna << "/\n";
    return 0;
}
