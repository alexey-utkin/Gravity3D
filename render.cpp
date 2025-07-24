#include "render.h"

// Render Scene Function
void renderScene(Mat &canvas, Simulation &sim) {
    // Perspective projection parameters
    double f = 300 * zoom;                // Focal length
    const double near = windowHeight / 2; // Near clipping plane depth to avoid division by zero.

    double cosX = cos(cameraAngleX), sinX = sin(cameraAngleX);
    double cosY = cos(cameraAngleY), sinY = sin(cameraAngleY);
    
    int observerIndex = sim.getObserverIndex();
    Vec3d &observer = sim.getObserver();
    Particle *particles = sim.getParticles();
    
    if (observerIndex >= 0) {
        observer = particles[observerIndex].position;
    }

    auto projectPerspective = [&](const Vec3d &_point) -> Point {
        Vec3d point = _point - observer;
        double x = point[0], y = point[1], z = point[2];

        double newY = y * cosX - z * sinX;
        double newZ = y * sinX + z * cosX;
        double newX = x * cosY + newZ * sinY;
        newZ = -x * sinY + newZ * cosY;

        double adjustedZ = newZ + near;
        if (adjustedZ <= 0.1)
            adjustedZ = 0.1;

        int x_proj = static_cast<int>(f * newX / adjustedZ + windowWidth / 2);
        int y_proj = static_cast<int>(f * newY / adjustedZ + windowHeight / 2);
        return {x_proj, y_proj};
    };

    // Draw cube edges
    for (auto i = 0; i < cubeEdges.size(); ++i) {
        const auto &edge = cubeEdges[i];
        Point p1 = projectPerspective(cubeCorners[edge.first]);
        Point p2 = projectPerspective(cubeCorners[edge.second]);
        // Wrap color selection using modulo operation to avoid overflow
        const Scalar &color = colors[i % colors.size()];
        line(canvas, p1, p2, color, 1);
    }
#pragma omp for schedule(static)
    for (auto i = 0; i < Simulation::cParticles; ++i) {
        Particle &p = particles[i];
        if (!p.active)
            continue;

        // Draw the particles.
        // Project 3D point to 2D perspective point
        Point center = projectPerspective(p.position);
        if (center.x < 0 || center.y < 0 || center.x >= windowWidth || center.y >= windowHeight) {
            // Skip rendering circles out of bounds.
            continue;
        }
        // Track the maximum radius in screen space
        double radius = 0.0;
        for (const auto &dir : directions) {
            // Compute a surface point in 3D by moving along the current direction
            // Project the surface point into 2D
            Point surfacePoint2D = projectPerspective(p.position + (p.showR * dir));
            // Compute 2D distance between the center and the projected surface point
            // Update the maximum radius
            radius = std::max(maxRadius, norm(Vec2i(surfacePoint2D - center)));
        }
        if (radius <= 1 || radius > 50)
            continue;

        int r = static_cast<int>(p.q() * 255.0 / maxQ);
        int g = static_cast<int>((p.position[2] / HEIGHT + 0.5) * 255.0);
        int b = observerIndex == i ? 255 : 0;

        // Draw particle
#pragma omp critical(canvas)
        {
            circle(canvas, center, radius, Scalar(b, g, r), FILLED);
        }

        int sizeTrace = p.trace.size();
        if (sizeTrace > 1) {
            Point p1 = center;
            // Draw each particle's trace
            for (auto t = 0; t < sizeTrace; ++t) {
                // Project 3D points to perspective 2D points
                Point p2 = projectPerspective(p.trace[t]);
                // Fade color effect for older trace points
                const int intensity = 255 * (sizeTrace - t + 1) / (sizeTrace + 1);
                const Scalar color(intensity, intensity, intensity); // Grayscale based on trace age
#pragma omp critical(canvas)
                {
                    line(canvas, p1, p2, color, 1);
                }
                p1 = p2;
            }
        }
    }
}
