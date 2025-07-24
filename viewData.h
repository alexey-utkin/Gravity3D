#ifndef GRAVITY3D_VIEWDATA_H
#define GRAVITY3D_VIEWDATA_H

#include "data.h"
#include <opencv2/opencv.hpp>

// External constants and variables
extern const std::vector<Vec3d> cubeCorners;
extern const std::vector<std::pair<int, int>> cubeEdges;
extern const Vec3d directions[3];
extern const std::vector<Scalar> colors;
extern Point lastMousePos;
extern bool rotating;
extern double cameraAngleX, cameraAngleY;
extern double zoom;
extern const char windowName[];

// Function declarations for user interaction
void onMouse(int event, int x, int y, int flags, void *);
bool inputProcessing(Simulation &sim);

#endif // GRAVITY3D_VIEWDATA_H