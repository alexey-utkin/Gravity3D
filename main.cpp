#include <omp.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cstdlib>

#define M_PI 3.14159265358979323846

using namespace cv;
using namespace std;

const int WIDTH = 1000;
const int HEIGHT = 1000;
const int FPS = 600;
const int cBody = 2000;
const int cBlackHole = 500;
const int cTailSize = 10;
const double maxQ = 1;
const double maxRadius = 2;
#undef RAND_MAX
double RAND_MAX = 32767.0;

struct Particle {
    atomic<int> lock{0};
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

    Particle() = default;

    // Delete copy constructor and copy assignment operator
    Particle(const Particle &) = delete;

    Particle &operator=(const Particle &) = delete;

    Particle(Particle &&other) = delete;

    Particle &operator=(Particle &&other) = delete;
};

alignas(64) Particle particles[cBody];

struct Locker {
    Particle &p;

    Locker(Particle &p1) : p(p1) {
        int expected = 0;
        while (!p.lock.compare_exchange_strong(expected, 1)) {
            expected = 0;
            std::this_thread::yield();// Reset expected value after a failed compare_exchange
        }
    }

    ~Locker() {
        p.lock.store(0);
    }
};

const double radiusC = (exp(maxRadius) - 1) / maxQ;

double sr(double q) {
    return log(1 + radiusC * q);
}

void initParticles() {
    Particle S;

    double offsets[][3] = {
            {0,         0,          0},
            {WIDTH / 2, HEIGHT / 2, HEIGHT / 2},
            {0,         HEIGHT / 2, HEIGHT / 2}
    };
    int i = 0;
    for (auto &p: particles) {
        // if (++i < cBlackHole)
        //     p.q = maxQ;
        // else
        //     p.q = maxQ * exp(rand() / RAND_MAX - 1.0);

        ++i;
        p.q = maxQ * rand() / RAND_MAX;
        int z = i % 3;
        const double *offset = offsets[z];
        p.position = {offset[0] + (WIDTH * (rand() / RAND_MAX) - WIDTH / 2) * 0.5,
                      offset[1] + (HEIGHT * (rand() / RAND_MAX) - HEIGHT / 2) * 0.5,
                      offset[2] + (HEIGHT * (rand() / RAND_MAX) - HEIGHT / 2) * 0.5};
        p.velocity = {rand() / RAND_MAX, rand() / RAND_MAX, rand() / RAND_MAX};

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

void initParticles1() {
    Particle S;

    int i = 0;
    for (auto &p: particles) {
        if (++i < cBlackHole)
            p.q = maxQ;
        else
            p.q = maxQ * exp(rand() / RAND_MAX - 1.0);

        p.position = {WIDTH / 4 + (WIDTH * (rand() / RAND_MAX) - WIDTH / 2) * 0.5,
                      HEIGHT / 4 + (HEIGHT * (rand() / RAND_MAX) - HEIGHT / 2) * 0.5,
                      HEIGHT / 4 + (HEIGHT * (rand() / RAND_MAX) - HEIGHT / 2) * 0.5};
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

void processInteraction(Particle &pi, Particle &pj) {
    Locker locki(pi);
    Locker lockj(pj);
    if (!pi.active || !pj.active) return;

    cv::Vec3d delta = pj.position - pi.position;
    double distSq = delta.dot(delta);
    double dist = sqrt(distSq);

    cv::Vec3d normal = delta / dist;

    if (dist < sr(pi.q) + sr(pj.q)) {
        double v1n = pi.velocity.dot(normal);
        double v2n = pj.velocity.dot(normal);
        {
            if (rand() % 5 != 1) { // Elastic collision
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
                pj.active = false; // Mark the particle as inactive
            }
        }
    } else {
        // Calculate forces
        double F = pj.q / distSq;
        {
            pi.force += normal * F;
            pj.force -= normal * F;
        }
    }
}

void updateParticles() {
    int numParticles = std::size(particles);
    // First loop: updating positions and clearing forces
#pragma omp for schedule(static)
    for (auto  i = 0; i < numParticles; ++i) {
        Particle &p = particles[i];
        if (!p.active) continue;
        p.addTrace();
        p.position += p.velocity;
        p.force = {0, 0, 0};
    }

    int totalInteractions = (numParticles * (numParticles - 1)) / 2;
#pragma omp for schedule(static)
    //schedule(dynamic, 64)
    for (auto index = 0; index < totalInteractions; ++index) {
        // Compute (i, j) indices from linear index
        int i = numParticles - 2 -
                (int) (std::sqrt(-8 * index + 4 * numParticles * (numParticles - 1) - 7) / 2.0 - 0.5);
        int j = index + i + 1 - (numParticles * (numParticles - 1)) / 2 +
                ((numParticles - i) * ((numParticles - i) - 1)) / 2;
        processInteraction(particles[i], particles[j]);
    }

    // Third loop: update velocities based on computed forces
#pragma omp for schedule(static)
    for (auto i = 0; i < numParticles; ++i) {
        Particle &p = particles[i];
        if (!p.active) continue;
        p.velocity += p.force;
    }
}

void renderScene(Mat &canvas) {
    canvas = Mat::zeros(HEIGHT, WIDTH, CV_8UC3);

    // Perspective projection parameters
    double f = 300; // Focal length
    const double near = HEIGHT / 2; // Near clipping plane depth to avoid division by zero.

    auto projectPerspective = [&](const Vec3d &point) -> Point {
        double z = point[2] + near; // Adjust for near clipping
        if (z <= 0.1) z = 0.1; // Prevents extreme scaling for very close objects
        double perspX = f * (point[0]) / z;
        double perspY = f * (point[1]) / z;

        // Step 3: Scale and center the projection on the canvas
        int x_proj = static_cast<int>(perspX + WIDTH / 2);
        int y_proj = static_cast<int>(perspY + HEIGHT / 2);
        return {x_proj, y_proj};
    };

    // Define the 8 corners of the cube (bounding box)
    vector<Vec3d> cubeCorners = {
            {-WIDTH / 4, -HEIGHT / 4, -HEIGHT / 4}, // Bottom-back-left
            {WIDTH / 4,  -HEIGHT / 4, -HEIGHT / 4}, // Bottom-back-right
            {WIDTH / 4,  HEIGHT / 4,  -HEIGHT / 4}, // Bottom-front-right
            {-WIDTH / 4, HEIGHT / 4,  -HEIGHT / 4}, // Bottom-front-left
            {-WIDTH / 4, -HEIGHT / 4, HEIGHT / 4}, // Top-back-left
            {WIDTH / 4,  -HEIGHT / 4, HEIGHT / 4}, // Top-back-right
            {WIDTH / 4,  HEIGHT / 4,  HEIGHT / 4}, // Top-front-right
            {-WIDTH / 4, HEIGHT / 4,  HEIGHT / 4}  // Top-front-left
    };

    // Define the edges of the bounding cube (pairs of vertices)
    vector<pair<int, int>> cubeEdges = {
            { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 0 }, // Bottom face
            { 4, 5 }, { 5, 6 }, { 6, 7 }, { 7, 4 }, // Top face
            { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 }  // Vertical edges
    };

    // Draw cube edges
    for (const auto &edge: cubeEdges) {
        Point p1 = projectPerspective(cubeCorners[edge.first]);
        Point p2 = projectPerspective(cubeCorners[edge.second]);
        line(canvas, p1, p2, Scalar(200, 200, 200), 1); // Light gray for cube edges
    }


    int numParticles = std::size(particles);
    #pragma omp for schedule(static)
    for (auto i = 0; i < numParticles; ++i) {
        Particle &p = particles[i];
        if (!p.active) continue;
        if (p.trace.size() > 1) {
            // Draw each particle's trace
            for (size_t i = 1; i < p.trace.size(); ++i) {
                const Vec3d &prev = p.trace[i - 1];
                const Vec3d &current = p.trace[i];

                // Project 3D points to perspective 2D points
                Point p1 = projectPerspective(prev);
                Point p2 = projectPerspective(current);

                // Fade color effect for older trace points
                int intensity = (255 * i) / cTailSize;
                Scalar color(intensity, intensity, intensity); // Grayscale based on trace age
#pragma omp critical
                {
                    line(canvas, p1, p2, color, 1);
                }
            }
        }

        // Draw the particles.
        // Project 3D point to 2D perspective point
        Point center = projectPerspective(p.position);
        if (center.x < 0 || center.y < 0 || center.x >= WIDTH || center.y >= HEIGHT) {
            // Skip rendering circles out of bounds.
            continue;
        }

        // Calculate visual attributes
        int radius = (int) (sr(p.q) * f / (p.position[2] + near));
        if (radius <= 0)
            continue;

        int r = (int) (p.q * 255.0 / maxQ);
        int g = (int) ((p.position[2] / HEIGHT + 0.5) * 255.0);
        int b = radius;

        // Draw particle
#pragma omp critical
        {
            circle(canvas, center, radius, Scalar(b, g, r), FILLED);
        }
    }
}

int main() {
    int numThreads = std::min(omp_get_max_threads(), 16); // Use at most 8 threads
    omp_set_num_threads(numThreads);
    std::cout << "Hello from thread " << numThreads << std::endl;

    srand(time(0));
    initParticles();

    Mat canvas(HEIGHT, WIDTH, CV_8UC3);
    namedWindow("Simulation", WINDOW_AUTOSIZE);

    while (true) {
#pragma omp parallel
        {
            updateParticles();
            renderScene(canvas);
        }
        imshow("Simulation", canvas);
        if (waitKey(1) == 27) break; // Exit on ESC key press.
    }
    return 0;
}
