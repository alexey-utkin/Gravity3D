#include <opencv2/opencv.hpp>
#include <vector>
#include <cstdlib>

using namespace cv;
using namespace std;

const int WIDTH = 1000;
const int HEIGHT = 1000;
const int FPS = 100;
const int cBody = 2000;
const int cTailSize = 10;
const double maxQ = 1;
const double maxRadius = 2;

struct Particle {
    cv::Vec3d position{};
    cv::Vec3d velocity{};
    cv::Vec3d force{};
    double q{};
    bool active = true;
    deque<cv::Vec3d> trace; // Trace to store the last 10 positions.

    // Method to add position to the trace and maintain size limit.
    void addTrace() {
        trace.push_back(position);
        if (trace.size() > cTailSize) {
            trace.pop_front(); // Remove the oldest position if trace exceeds size 10.
        }
    }
};

vector<Particle> particles;

const double radiusC = (exp(maxRadius) - 1) / maxQ;

double sr(double q) {
    return log(1 + radiusC * q);
}

void initParticles() {
    particles.resize(cBody);
    Particle S;

    for (auto &p: particles) {
        p.q = maxQ * exp((double) rand() / RAND_MAX - 1.0);
        p.position = {WIDTH * ((double) rand() / RAND_MAX) - WIDTH / 2,
                      HEIGHT * ((double) rand() / RAND_MAX) - HEIGHT / 2,
                      HEIGHT * ((double) rand() / RAND_MAX) - HEIGHT / 2};
        p.velocity = {0, 0, 0};

        S.q += p.q;
        S.position += p.position * p.q;
        S.velocity += p.velocity * p.q;
    }

    S.position = S.position / S.q;
    S.velocity = S.velocity / S.q;

    for (auto &p: particles) {
        p.position -= S.position;
        p.velocity -= S.velocity;
    }
}

void updateParticles() {
    for (auto &p: particles) {
        if (!p.active) continue;

        p.addTrace();

        p.position += p.velocity;
        p.force = {0, 0, 0};
    }

    for (size_t i = 0; i < particles.size(); ++i) {
        Particle &pi = particles[i];
        if (!pi.active) continue;
        for (size_t j = i + 1; j < particles.size(); ++j) {
            Particle &pj = particles[j];
            if (!pj.active) continue;

            cv::Vec3d delta = pj.position - pi.position;
            double distSq = delta.dot(delta);
            double dist = sqrt(distSq);

            cv::Vec3d normal = delta / dist;

            if (dist < sr(pi.q) + sr(pj.q)) {
                double v1n = pi.velocity.dot(normal);
                double v2n = pj.velocity.dot(normal);

                if (rand() % 2) { // Elastic collision
                    double m1 = pi.q;
                    double m2 = pj.q;
                    double v1nNew = (v1n * (m1 - m2) + 2 * m2 * v2n) / (m1 + m2);
                    double v2nNew = (v2n * (m2 - m1) + 2 * m1 * v1n) / (m1 + m2);

                    pi.velocity += normal * (v1nNew - v1n);
                    pj.velocity += normal * (v2nNew - v2n);
                } else { // Inelastic collision
                    double totalMass = pi.q + pj.q;
                    pi.velocity = (pi.velocity * pi.q + pj.velocity * pj.q) / totalMass;
                    pi.position = (pi.position * pi.q + pj.position * pj.q) / totalMass;
                    pi.q = totalMass;
                    pj.active = false; // Remove from interaction
                }
            } else {
                double F = pj.q / distSq;
                pi.force += normal * F;
                F = pi.q / distSq;
                pj.force -= normal * F;
            }
        }
    }

    for (auto &p: particles) {
        if (!p.active) continue;
        p.velocity += p.force;
    }
}

void renderScene(Mat &canvas) {
    canvas = Mat::zeros(HEIGHT, WIDTH, CV_8UC3);

    // Draw each particle's trace.
    for (const auto &p : particles) {
        if (!p.active || p.trace.empty()) continue;

        for (size_t i = 1; i < p.trace.size(); ++i) {
            const Vec3d &prev = p.trace[i - 1];
            const Vec3d &current = p.trace[i];
            Point p1((int)prev[0] + WIDTH / 2, (int)prev[1] + HEIGHT / 2);
            Point p2((int)current[0] + WIDTH / 2, (int)current[1] + HEIGHT / 2);

            // Fade color effect for older trace points.
            int intensity = (255 * i) / 10;
            line(canvas, p1, p2, Scalar(intensity, intensity, intensity), 1);
        }
    }

    // Draw the particles.
    for (const auto &p: particles) {
        if (!p.active) continue;
        int radius = (int) sr(p.q);
        int r = (int) ((p.position[2] / HEIGHT + 0.5) * 255.0);
        int g = (int) (p.q / maxQ * 255.0);
        int b = radius;
        circle(canvas, Point((int) p.position[0] + WIDTH / 2, (int) p.position[1] + HEIGHT / 2), radius,
               Scalar(b, g, r), FILLED);
    }
}

int main() {
    srand(time(0));
    initParticles();

    Mat canvas(HEIGHT, WIDTH, CV_8UC3);
    namedWindow("Simulation", WINDOW_AUTOSIZE);

    while (true) {
        updateParticles();
        renderScene(canvas);
        imshow("Simulation", canvas);
        if (waitKey(1000 / FPS) == 27) break; // Exit on ESC key press.
    }

    return 0;
}
