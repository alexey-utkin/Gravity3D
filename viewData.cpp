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
        zoom += (flags > 0) ? 0.1 : -0.1;
        zoom = max(0.1, zoom);
    }
}

bool inputProcessing() {
    char key = static_cast<char>(waitKey(1));
    switch (key) {
    default:
        break;
    case '+':
        zoom *= 1.1; // Zoom in
        break;
    case '-':
        zoom = max(1e-3, zoom*0.9); // Zoom out
        break;
    case '.':
    case '>':
        frameCountPerTrace *= 5;
        break;
    case ',':
    case '<' :
        frameCountPerTrace /= 5;
        if (frameCountPerTrace < 1) {
            frameCountPerTrace = 1;
        }
        break;
    case ' ':
        cameraAngleX = 0.0;
        cameraAngleY = 0.0;
        zoom = 1.0;
        frameCountPerTrace = 1;
        break;
    case 'c':
        recenterAndZeroV();
        break;
    case 27: // ESC key
    case 'q':
        return false;
    }
    return true;
}