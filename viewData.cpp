#include "viewData.h"
#include "simulation.h"

// Mouse callback for rotation and zoom
void Camera::_onMouse(int event, int x, int y, int flags) {
    if (event == EVENT_LBUTTONDOWN) {
        rotating = true;
        lastMousePos = Point(x, y);
    } else if (event == EVENT_LBUTTONUP) {
        rotating = false;
    } else if (event == EVENT_MOUSEMOVE && rotating) {
        int dx = x - lastMousePos.x;
        int dy = y - lastMousePos.y;
        angleY += dx * 0.005;
        angleX += dy * 0.005;
        lastMousePos = Point(x, y);
    } else if (event == EVENT_MOUSEWHEEL) {
        zoom *= (flags > 0) ? 1.1 : 0.9;
        zoom = max(1e-6, zoom);
    }
}

bool Simulation::inputProcessing() {
    char key = static_cast<char>(waitKey(1));
    switch (key) {
    default:
        break;
    case 'o':
        sortParticles();
        for (++observerIndex; !particles[observerIndex].active && observerIndex < cParticles; ++observerIndex)
            ;
        if (observerIndex >= cParticles) {
            observerIndex = -1;
        }
        break;
    case '+':
        camera.zoom *= 1.1; // Zoom in
        break;
    case '-':
        camera.zoom = max(1e-6, camera.zoom * 0.9); // Zoom out
        break;
    case '.':
    case '>':
        frameCountPerTrace *= 5;
        cTailSize = max(10, frameCountPerTrace/100);
        break;
    case ',':
    case '<':
        frameCountPerTrace /= 5;
        if (frameCountPerTrace < 1) {
            frameCountPerTrace = 1;
        }
        cTailSize = max(10, frameCountPerTrace/100);
        break;
    case ' ':
        camera.angleX = 0.0;
        camera.angleY = 0.0;
        camera.zoom = 1.0;
        frameCountPerTrace = 1;
        observer = {0, 0, 0};
        observerIndex = -1;
        break;
    case 'C':
        recenterAndZeroV(true);
        break;
    case 'c':
        recenterAndZeroV(false);
        break;
    case 27: // ESC key
    case 'q':
        return false;
    }
    return true;
}