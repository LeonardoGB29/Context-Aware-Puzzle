// gabor.cpp — implementaciones

#include "gabor.hpp"
#include <algorithm>
#include <cmath>

namespace gabor {

void sortOrientations(vector<float>& feat) {
    for (int s = 0; s + ORIENTATIONS <= (int)feat.size(); s += ORIENTATIONS)
        sort(feat.begin() + s, feat.begin() + s + ORIENTATIONS);
}

vector<cv::Mat> makeBank() {
    int    ksize        = 21;
    int    orientations = 6;
    double sigma = 4.0, gamma = 0.5, psi = 0.0;
    double lambdas[3] = {5.0, 9.0, 15.0};

    vector<cv::Mat> bank;
    for (double lambda : lambdas)
        for (int o = 0; o < orientations; ++o) {
            double theta = CV_PI * o / orientations;
            bank.push_back(cv::getGaborKernel({ksize, ksize}, sigma, theta,
                                              lambda, gamma, psi, CV_32F));
        }
    return bank;
}

cv::Mat innerMask(cv::Mat& bgra) {
    vector<cv::Mat> ch;
    cv::split(bgra, ch);
    cv::Mat mask = ch[3] >= 128;
    cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, {5, 5}));
    return mask;
}

vector<cv::Mat> responses(cv::Mat& bgra, vector<cv::Mat>& bank, double sigma) {
    cv::Mat gray;
    cv::cvtColor(bgra, gray, cv::COLOR_BGRA2GRAY);
    gray.convertTo(gray, CV_32F, 1.0 / 255.0);

    vector<cv::Mat> out;
    out.reserve(bank.size());
    for (cv::Mat& k : bank) {
        cv::Mat r;
        cv::filter2D(gray, r, CV_32F, k);
        r = cv::abs(r);
        if (sigma > 0) cv::GaussianBlur(r, r, cv::Size(), sigma);
        out.push_back(r);
    }
    return out;
}

cv::Mat collectSamples(vector<cv::Mat>& imgs, vector<cv::Mat>& bank, double sigma) {
    int nf   = (int)bank.size();
    int step = 6;

    vector<float> buf;
    vector<float> feat(nf);
    for (cv::Mat& img : imgs) {
        cv::Mat mask = innerMask(img);
        vector<cv::Mat> resp = responses(img, bank, sigma);
        long idx = 0;
        for (int y = 0; y < img.rows; ++y)
            for (int x = 0; x < img.cols; ++x) {
                if (mask.at<uchar>(y, x) < 128) continue;
                if ((idx++ % step) != 0) continue;
                for (int f = 0; f < nf; ++f) feat[f] = resp[f].at<float>(y, x);
                sortOrientations(feat);
                buf.insert(buf.end(), feat.begin(), feat.end());
            }
    }
    int n = (int)buf.size() / nf;
    return cv::Mat(n, nf, CV_32F, buf.data()).clone();
}

cv::Mat trainTextons(const cv::Mat& samples, int k) {
    if (samples.rows < k) k = max(1, samples.rows);
    cv::Mat labels, centers;
    cv::kmeans(samples, k, labels,
               cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 15, 1.0),
               2, cv::KMEANS_PP_CENTERS, centers);
    return centers.clone();
}

cv::Mat textonMap(cv::Mat& bgra, vector<cv::Mat>& bank,
                  cv::Mat& centers, double sigma) {
    int nf = (int)bank.size();
    cv::Mat mask = innerMask(bgra);
    vector<cv::Mat> resp = responses(bgra, bank, sigma);

    cv::Mat map(bgra.rows, bgra.cols, CV_32S, cv::Scalar(-1));
    vector<float> feat(nf);
    for (int y = 0; y < bgra.rows; ++y)
        for (int x = 0; x < bgra.cols; ++x) {
            if (mask.at<uchar>(y, x) < 128) continue;
            for (int f = 0; f < nf; ++f) feat[f] = resp[f].at<float>(y, x);
            sortOrientations(feat);

            int best = 0;
            double bestDist = 1e30;
            for (int c = 0; c < centers.rows; ++c) {
                float* ce = centers.ptr<float>(c);
                double d = 0;
                for (int f = 0; f < nf; ++f) { double e = feat[f] - ce[f]; d += e * e; }
                if (d < bestDist) { bestDist = d; best = c; }
            }
            map.at<int>(y, x) = best;
        }
    return map;
}

cv::Mat majorityFilter(const cv::Mat& map, int k, int radius, int passes) {
    cv::Mat cur = map.clone();
    vector<int> count(k);
    for (int p = 0; p < passes; ++p) {
        cv::Mat next = cur.clone();
        for (int y = 0; y < cur.rows; ++y)
            for (int x = 0; x < cur.cols; ++x) {
                if (cur.at<int>(y, x) < 0) continue;
                fill(count.begin(), count.end(), 0);
                for (int dy = -radius; dy <= radius; ++dy)
                    for (int dx = -radius; dx <= radius; ++dx) {
                        int yy = y + dy, xx = x + dx;
                        if (yy < 0 || xx < 0 || yy >= cur.rows || xx >= cur.cols) continue;
                        int t = cur.at<int>(yy, xx);
                        if (t >= 0) count[t]++;
                    }
                int best = 0;
                for (int c = 1; c < k; ++c)
                    if (count[c] > count[best]) best = c;
                next.at<int>(y, x) = best;
            }
        cur = next;
    }
    return cur;
}

vector<float> histogram(cv::Mat& map, int k) {
    vector<float> hist(k, 0.0f);
    float total = 0;
    for (int y = 0; y < map.rows; ++y)
        for (int x = 0; x < map.cols; ++x) {
            int t = map.at<int>(y, x);
            if (t < 0) continue;
            hist[t] += 1.0f;
            total += 1.0f;
        }
    if (total > 0) for (float& h : hist) h /= total;
    return hist;
}

vector<float> bandHistogram(cv::Mat& map, cv::Point2f a, cv::Point2f b,
                            int k, float bandPx) {
    vector<float> hist(k, 0.0f);
    float total = 0;
    cv::Point2f d = b - a;
    float len2 = d.x * d.x + d.y * d.y + 1e-6f;

    int x0 = max(0, (int)floor(min(a.x, b.x) - bandPx));
    int x1 = min(map.cols - 1, (int)ceil(max(a.x, b.x) + bandPx));
    int y0 = max(0, (int)floor(min(a.y, b.y) - bandPx));
    int y1 = min(map.rows - 1, (int)ceil(max(a.y, b.y) + bandPx));
    for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x) {
            int tx = map.at<int>(y, x);
            if (tx < 0) continue;
            cv::Point2f p((float)x - a.x, (float)y - a.y);
            float t = (p.x * d.x + p.y * d.y) / len2;
            if (t < 0 || t > 1) continue;
            float dx = p.x - t * d.x, dy = p.y - t * d.y;
            if (dx * dx + dy * dy > bandPx * bandPx) continue;
            hist[tx] += 1.0f;
            total += 1.0f;
        }
    if (total > 0) for (float& h : hist) h /= total;
    return hist;
}

float chi2(vector<float>& a, vector<float>& b) {
    double s = 0;
    for (int i = 0; i < (int)a.size() && i < (int)b.size(); ++i) {
        double diff = a[i] - b[i], sum = a[i] + b[i];
        if (sum > 1e-9) s += diff * diff / sum;
    }
    return (float)(0.5 * s);
}

}  // namespace gabor
