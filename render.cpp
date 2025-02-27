// Render Scene Function
inline void renderScene(Mat &canvas) {
    // Perspective projection parameters
    double f = 300 * zoom;                // Focal length
    const double near = windowHeight / 2; // Near clipping plane depth to avoid division by zero.

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
                Scalar color(intensity, intensity, intensity); // Grayscale based on trace age
#pragma omp critical(canvas)
                {
                    line(canvas, p1, p2, color, 1);
                }
            }
        }

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
            Point surfacePoint2D = projectPerspective(p.position + (p.realR * dir));
            // Compute 2D distance between the center and the projected surface point
            // Update the maximum radius
            radius = std::max(maxRadius, norm(Vec2i(surfacePoint2D - center)));
        }
        if (radius <= 1 || radius > 50)
            continue;

        int r = static_cast<int>(p.q() * 255.0 / maxQ);
        int g = static_cast<int>((p.position[2] / HEIGHT + 0.5) * 255.0);
        int b = 0;

        // Draw particle
#pragma omp critical(canvas)
        {
            circle(canvas, center, radius, Scalar(b, g, r), FILLED);
        }
    }
}
