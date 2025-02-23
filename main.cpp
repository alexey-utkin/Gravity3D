/**
@author: Alexey Utkin
@email:  alexey.utkin@gmail.com

    My God, it's full of stars!
                      Arthur C.
*/

#include <omp.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cstdlib>
#include <deque>
#include <atomic>
#include <thread>
#include <iostream>

using namespace cv;
using namespace std;

constexpr int WIDTH = 1000;
constexpr int HEIGHT = 1000;
constexpr int cParticles = 2000;
constexpr int cBlackHole = 500;
constexpr int cTailSize = 10;
constexpr double maxQ = 1;
constexpr double maxRadius = 2;
constexpr double minDist = 1e-6;
constexpr double minQ = 1e-6;
int numThreads = 1;
const double radiusC = (exp(maxRadius) - 1) / maxQ;

double totalPotentialEnergy = 0.0;
double totalKineticEnergy = 0.0;
int inactiveCount = 0;

double rnd() { return rand() / 32767.0; }

struct Particle {
    atomic<int> lock{0};
    Vec3d position{};
    Vec3d velocity{};
    Vec3d force{};

    double realR{};
    double showR{};

    bool active = true;
    deque<Vec3d> trace;

    void addTrace() {
        trace.push_back(position);
        if (trace.size() > cTailSize) {
            trace.pop_front();
        }
    }

    void shiftTrace(const Vec3d &shift) {
       for (auto &t : trace) {
           t -= shift;
       }
    }

    void setQ(double q) {
        if (q < minQ) {
            q = minQ;
        }
        _q = q;
        showR = log(1 + radiusC * _q);
        realR = max(showR/2, minDist/2);
    }

    [[nodiscard]] double q() const { return _q; }
protected:
    double _q{};
};

alignas(64) Particle particles[cParticles];

struct Locker {
    Particle &p;

    explicit Locker(Particle &p1) : p(p1) {
        int expected = 0;
        while (!p.lock.compare_exchange_strong(expected, 1)) {
            expected = 0;
            this_thread::yield();
        }
    }

    ~Locker() { p.lock.store(0); }
};

struct CenterOfMass {
    Vec3d position{0,0,0};
    Vec3d impuls{0,0,0};
    double q{0};
};

void normalize() {
    // Compute the center of mass based on active particles
    vector<CenterOfMass> centerOfMasses(numThreads, {{0, 0, 0}, 0});
#pragma omp for schedule(static)
    for (auto i = 0; i < cParticles; ++i) {
        Particle &p = particles[i];
        if (!p.active) continue; // Skip inactive particles
        CenterOfMass &cm = centerOfMasses[omp_get_thread_num()];
        cm.position += p.position * p.q();
        cm.impuls += p.velocity * p.q();
        cm.q += p.q();
    }

#pragma omp barrier
    CenterOfMass cm{};
#pragma omp single
    {
        for (const auto &threadCM : centerOfMasses) {
            cm.position += threadCM.position;
            cm.impuls += threadCM.impuls;
            cm.q += threadCM.q;
        }

        if (cm.q <= 0) {
            cm.q -= minQ;
        }

        cm.position /= cm.q;
        cm.impuls /= cm.q;
    }

    // Apply offset to move the center of mass to the origin
#pragma omp barrier
#pragma omp for schedule(static)
    for (auto i = 0; i < cParticles; ++i) {
        Particle &p = particles[i];
        if (!p.active) continue; // Skip inactive particles
        p.position -= cm.position; // Shift position
        p.shiftTrace(cm.position);
        p.velocity -= cm.impuls;
    }
}

[[maybe_unused]] void initParticles_WithOldBlackHoles() {
    Particle S;

    int i = 0;
    for (auto &p : particles) {
        if (++i < cBlackHole)
            p.setQ(maxQ);
        else
            p.setQ(maxQ * exp(rnd() - 1.0));
        p.position = {WIDTH / 4 + (WIDTH * rnd() - WIDTH / 2) * 0.5,
                      HEIGHT / 4 + (HEIGHT * rnd() - HEIGHT / 2) * 0.5,
                      HEIGHT / 4 + (HEIGHT * rnd() - HEIGHT / 2) * 0.5};
        p.velocity = {0, 0, 0};
    }
}

[[maybe_unused]] void initParticles_3Centers() {
    double offsets[][3] = {{0, 0, 0},
                           {WIDTH / 2, HEIGHT / 2, HEIGHT / 2},
                           {0, HEIGHT / 2, HEIGHT / 2}};

    int i = 0;
    for (auto &p : particles) {
        ++i;
        p.setQ(maxQ * rnd());
        int z = i % 3;
        const double *offset = offsets[z];
        p.position = {offset[0] + (WIDTH * rnd() - WIDTH / 2) * 0.5,
                      offset[1] + (HEIGHT * rnd() - HEIGHT / 2) * 0.5,
                      offset[2] + (HEIGHT * rnd() - HEIGHT / 2) * 0.5};
        p.velocity = {rnd(), rnd(), rnd()};

    }
}


void processInteraction(Particle &pi, Particle &pj) {
    Locker lockI(pi);
    Locker lockJ(pj);
    if (!pi.active || !pj.active)
        return;

    Vec3d delta = pj.position - pi.position;
    double distSq = delta.dot(delta);
    if (distSq < minDist) {
        distSq = minDist;
    }
    double dist = sqrt(distSq);
    Vec3d normal = delta / dist;

    if (dist < (pi.realR + pj.realR)) {
        double v1n = pi.velocity.dot(normal);
        double v2n = pj.velocity.dot(normal);
        double totalMass = pi.q() + pj.q();
        if (rnd() < 0.05) {
            // Elastic collision
            double m1 = pi.q();
            double m2 = pj.q();
            double v1nNew = (v1n * (m1 - m2) + 2 * m2 * v2n) / totalMass;
            double v2nNew = (v2n * (m2 - m1) + 2 * m1 * v1n) / totalMass;

            pi.velocity += normal * (v1nNew - v1n);
            pj.velocity += normal * (v2nNew - v2n);
        } else { // Inelastic collision
            pi.velocity = (pi.velocity * pi.q() + pj.velocity * pj.q()) / totalMass;
            pi.position = (pi.position * pi.q() + pj.position * pj.q()) / totalMass;
            pi.setQ(totalMass);
            pj.active = false; // Mark the particle as inactive
        }
    } else {
        double qq = pj.q() * pi.q();
        totalPotentialEnergy += -qq / dist;

        // Calculate forces
        double F = qq / distSq;
        const Vec<double, 3> &vecF = normal * F;
        pi.force += vecF;
        pj.force -= vecF;
    }
}

void updateParticles() {
    // Updating positions and clearing forces
#pragma omp barrier
#pragma omp for reduction(+:totalKineticEnergy, inactiveCount) schedule(static)
    for (auto i = 0; i < cParticles; ++i) {
        Particle &p = particles[i];
        if (!p.active) {
            inactiveCount += 1;
            continue;
        }
        p.addTrace();
        totalKineticEnergy += p.velocity.dot(p.velocity) * p.q() * 0.5;
        p.position += p.velocity;
        p.force = {0, 0, 0};
    }

    // Update forces
    int totalInteractions = (cParticles * (cParticles - 1)) / 2;
#pragma omp barrier
#pragma omp for  reduction(+:totalPotentialEnergy) schedule(static) // Dynamic scheduling with chunk size 64
    for (auto index = 0; index < totalInteractions; ++index) {
        // Compute (i, j) indices from linear index
        int i = cParticles - 2 -
                (int) (std::sqrt(-8 * index + 4 * cParticles * (cParticles - 1) - 7) / 2.0 - 0.5);
        int j = index + i + 1 - (cParticles * (cParticles - 1)) / 2 +
                ((cParticles - i) * ((cParticles - i) - 1)) / 2;
        processInteraction(particles[i], particles[j]);
    }

    // Update velocities based on computed forces
#pragma omp barrier
#pragma omp for schedule(static)
    for (auto i = 0; i < cParticles; ++i) {
        Particle &p = particles[i];
        if (!p.active)
            continue;
        p.velocity += p.force / p.q();
    }
}

// Define the 8 corners of the cube (bounding box)
vector<Vec3d> cubeCorners = {
    {-WIDTH / 4, -HEIGHT / 4, -HEIGHT / 4}, // Bottom-back-left
    {WIDTH / 4, -HEIGHT / 4, -HEIGHT / 4},  // Bottom-back-right
    {WIDTH / 4, HEIGHT / 4, -HEIGHT / 4},   // Bottom-front-right
    {-WIDTH / 4, HEIGHT / 4, -HEIGHT / 4},  // Bottom-front-left
    {-WIDTH / 4, -HEIGHT / 4, HEIGHT / 4},  // Top-back-left
    {WIDTH / 4, -HEIGHT / 4, HEIGHT / 4},   // Top-back-right
    {WIDTH / 4, HEIGHT / 4, HEIGHT / 4},    // Top-front-right
    {-WIDTH / 4, HEIGHT / 4, HEIGHT / 4}    // Top-front-left
};

// Define the edges of the bounding cube (pairs of vertices)
vector<pair<int, int>> cubeEdges = {
    // @formatter:off
    {0, 1}, {1, 2}, {2, 3},
    {3, 0}, // Bottom face
    {4, 5}, {5, 6}, {6, 7},
    {7, 4}, // Top face
    {0, 4}, {1, 5}, {2, 6},
    {3, 7} // Vertical edges
    // @formatter:on
};

vector<Scalar> colors = {
    Scalar(0, 0, 255),   // Bright Red
    Scalar(0, 255, 0),   // Bright Green
    Scalar(255, 0, 0),   // Bright Blue
    Scalar(0, 255, 255), // Cyan
    Scalar(255, 255, 0), // Yellow
    Scalar(255, 0, 255), // Magenta
    Scalar(128, 0, 255), // Bright Purple
    Scalar(0, 128, 255), // Bright Orange
    Scalar(255, 128, 0), // Bright Pink
    Scalar(0, 255, 128), // Bright Teal
    Scalar(128, 255, 0), // Lime Green
    Scalar(255, 0, 128)  // Bright Cherry
};

// Camera control variables
Point lastMousePos;
bool rotating = false;
double cameraAngleX = 0.0, cameraAngleY = 0.0;
double zoom = 1.0;

void renderScene(Mat &canvas) {
    // Perspective projection parameters
    double f = 300 * zoom; // Focal length
    const double near =
        HEIGHT / 2; // Near clipping plane depth to avoid division by zero.

    double cosX = cos(cameraAngleX), sinX = sin(cameraAngleX);
    double cosY = cos(cameraAngleY), sinY = sin(cameraAngleY);

    auto projectPerspective = [&](const Vec3d &point) -> Point {
        double x = point[0], y = point[1], z = point[2];

        double newY = y * cosX - z * sinX;
        double newZ = y * sinX + z * cosX;
        double newX = x * cosY + newZ * sinY;
        newZ = -x * sinY + newZ * cosY;

        double adjustedZ = newZ + near;
        if (adjustedZ <= 0.1)
            adjustedZ = 0.1;

        int x_proj = static_cast<int>(f * newX / adjustedZ + WIDTH / 2);
        int y_proj = static_cast<int>(f * newY / adjustedZ + HEIGHT / 2);
        return {x_proj, y_proj};
    };

    // Draw cube edges
    for (size_t i = 0; i < cubeEdges.size(); ++i) {
        const auto &edge = cubeEdges[i];
        Point p1 = projectPerspective(cubeCorners[edge.first]);
        Point p2 = projectPerspective(cubeCorners[edge.second]);
        // Wrap color selection using modulo operation to avoid overflow
        const Scalar &color = colors[i % colors.size()];
        line(canvas, p1, p2, color, 1);
    }
#pragma omp for schedule(static)
    for (auto i = 0; i < cParticles; ++i) {
        Particle &p = particles[i];
        if (!p.active)
            continue;
        if (p.trace.size() > 1) {
            // Draw each particle's trace
            for (auto t = 1; t < p.trace.size(); ++t) {
                const Vec3d &prev = p.trace[t - 1];
                const Vec3d &current = p.trace[t];

                // Project 3D points to perspective 2D points
                Point p1 = projectPerspective(prev);
                Point p2 = projectPerspective(current);

                // Fade color effect for older trace points
                int intensity = (255 * t) / cTailSize;
                Scalar color(intensity, intensity,
                             intensity); // Grayscale based on trace age
#pragma omp critical(canvas)
                {
                    line(canvas, p1, p2, color, 1);
                }
            }
        }

        // Draw the particles.
        // Project 3D point to 2D perspective point
        Point center = projectPerspective(p.position);
        if (center.x < 0 || center.y < 0 || center.x >= WIDTH ||
            center.y >= HEIGHT) {
            // Skip rendering circles out of bounds.
            continue;
        }

        // Calculate visual attributes
        int radius = (int)(p.showR * f / (p.position[2] + near));
        if (radius <= 0 || radius > 50)
            continue;

        int r = (int)(p.q() * 255.0 / maxQ);
        int g = (int)((p.position[2] / HEIGHT + 0.5) * 255.0);
        int b = radius;

        // Draw particle
#pragma omp critical(canvas)
        {
            circle(canvas, center, radius, Scalar(b, g, r), FILLED);
        }
    }
}

// Mouse callback for rotation and zoom
void onMouse(int event, int x, int y, int flags, void *) {
    if (event == EVENT_LBUTTONDOWN) {
        rotating = true;
        lastMousePos = Point(x, y);
    } else if (event == EVENT_LBUTTONUP) {
        rotating = false;
    } else if (event == EVENT_MOUSEMOVE && rotating) {
        int dx = x - lastMousePos.x;
        int dy = y - lastMousePos.y;
        cameraAngleY += dx * 0.005;
        cameraAngleX += dy * 0.005;
        lastMousePos = Point(x, y);
    } else if (event == EVENT_MOUSEWHEEL) {
        zoom += (flags > 0) ? 0.1 : -0.1;
        zoom = max(0.1, zoom);
    }
}

int main() {
    numThreads = max(omp_get_max_threads() - 1, 1);
    // numThreads = 1;
    omp_set_num_threads(numThreads);
    cout << "Running with " << numThreads << " threads." << endl;

    srand(time(nullptr));
    // initParticles:
    initParticles_3Centers();
    // initParticles_WithOldBlackHoles();

    Mat canvas(HEIGHT, WIDTH, CV_8UC3);
    namedWindow("Simulation", WINDOW_AUTOSIZE);
    setMouseCallback("Simulation", onMouse, nullptr);

    while (true) {
        canvas = Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
        totalPotentialEnergy = 0.0;
        totalKineticEnergy = 0.0;
        inactiveCount = 0;
        auto start = chrono::high_resolution_clock::now();
#pragma omp parallel
        {
            normalize();
            updateParticles();
            renderScene(canvas);
        }
        auto duration = chrono::duration_cast<std::chrono::milliseconds>(
            chrono::high_resolution_clock::now() - start);
        std::cout << inactiveCount << " E: " << (totalKineticEnergy + totalPotentialEnergy) <<  ", " << totalKineticEnergy << ", " << totalPotentialEnergy << std::endl;
        // std::cout << "Execution time: " << duration.count() << " ms" << std::endl;
        imshow("Simulation", canvas);
        int key = waitKey(1);
        switch (key) {
        case ' ':
            cameraAngleX = 0.0;
            cameraAngleY = 0.0;
            zoom = 1.0;
            break;
        case 27: // ESC key
        case 'q':
            return 0;
        }
    }

    return 0;
}
