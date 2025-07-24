#ifndef GRAVITY3D_VIEWDATA_H
#define GRAVITY3D_VIEWDATA_H

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// Camera structure to hold camera-related variables
struct Camera {
    Point lastMousePos;
    bool rotating = false;
    double angleX = 0.0;
    double angleY = 0.0;
    double zoom = 1.0;

    void _onMouse(int event, int x, int y, int flags);
    static void onMouse(int event, int x, int y, int flags, void *pCamera) {
        static_cast<Camera *>(pCamera)->_onMouse(event, x, y, flags);
    }
};

#endif // GRAVITY3D_VIEWDATA_H