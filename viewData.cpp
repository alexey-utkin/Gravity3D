#include "viewData.h"

// Define the 8 corners of the cube (bounding box)
const vector<Vec3d> cubeCorners = {
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
const vector<pair<int, int>> cubeEdges = {
    // @formatter:off
    {0, 1},
    {1, 2},
    {2, 3},
    {3, 0}, // Bottom face
    {4, 5},
    {5, 6},
    {6, 7},
    {7, 4}, // Top face
    {0, 4},
    {1, 5},
    {2, 6},
    {3, 7} // Vertical edges
    // @formatter:on
};

// Define orthogonal directions
const Vec3d directions[] = {
    Vec3d(1.0, 0.0, 0.0), // x-axis
    Vec3d(0.0, 1.0, 0.0), // y-axis
    Vec3d(0.0, 0.0, 1.0)  // z-axis
};

const vector<Scalar> colors = {
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

const char windowName[] = "Simulation";

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
        zoom *= (flags > 0) ? 1.1 : 0.9;
        zoom = max(1e-6, zoom);
    }
}

bool inputProcessing(Simulation &sim) {
    char key = static_cast<char>(waitKey(1));
    int localObserverIndex;
    
    switch (key) {
    default:
        break;
    case 'o':
        // Sort particles by mass
        sim.sortParticles();
        
        // Update observer index
        localObserverIndex = sim.getObserverIndex();
        localObserverIndex++;
        
        // Find next active particle
        while (localObserverIndex < Simulation::cParticles && !sim.getParticles()[localObserverIndex].active) {
            localObserverIndex++;
        }
        
        if (localObserverIndex >= Simulation::cParticles) {
            localObserverIndex = -1;
        }
        
        sim.setObserverIndex(localObserverIndex);
        break;
    case '+':
        zoom *= 1.1; // Zoom in
        break;
    case '-':
        zoom = max(1e-6, zoom * 0.9); // Zoom out
        break;
    case '.':
    case '>':
        sim.setFrameCountPerTrace(sim.getFrameCountPerTrace() * 5);
        sim.setTailSize(max(10, sim.getFrameCountPerTrace()/100));
        break;
    case ',':
    case '<':
        sim.setFrameCountPerTrace(sim.getFrameCountPerTrace() / 5);
        if (sim.getFrameCountPerTrace() < 1) {
            sim.setFrameCountPerTrace(1);
        }
        sim.setTailSize(max(10, sim.getFrameCountPerTrace()/100));
        break;
    case ' ':
        cameraAngleX = 0.0;
        cameraAngleY = 0.0;
        zoom = 1.0;
        sim.setFrameCountPerTrace(1);
        sim.getObserver() = {0, 0, 0};
        sim.setObserverIndex(-1);
        break;
    case 'C':
        sim.recenterAndZeroV(true);
        break;
    case 'c':
        sim.recenterAndZeroV(false);
        break;
    case 27: // ESC key
    case 'q':
        return false;
    }
    return true;
}