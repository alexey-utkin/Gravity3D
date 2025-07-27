#ifndef GRAVITY3D_DATA_SERIALIZATION_H
#define GRAVITY3D_DATA_SERIALIZATION_H

#include "data.h"
#include "viewData.h"
#include "simulation.h"
#include "external/nlohmann/json.hpp"

using json = nlohmann::json;

// Serialization for OpenCV types

// Vec3d serialization
inline void to_json(json& j, const cv::Vec3d& vec) {
    j = json{vec[0], vec[1], vec[2]};
}

inline void from_json(const json& j, cv::Vec3d& vec) {
    vec = cv::Vec3d(j[0], j[1], j[2]);
}

// Point serialization
inline void to_json(json& j, const cv::Point& point) {
    j = json{point.x, point.y};
}

inline void from_json(const json& j, cv::Point& point) {
    point.x = j[0];
    point.y = j[1];
}

// Matx33d serialization
inline void to_json(json& j, const cv::Matx33d& mat) {
    j = json{
        mat(0, 0), mat(0, 1), mat(0, 2),
        mat(1, 0), mat(1, 1), mat(1, 2),
        mat(2, 0), mat(2, 1), mat(2, 2)
    };
}

inline void from_json(const json& j, cv::Matx33d& mat) {
    mat(0, 0) = j[0]; mat(0, 1) = j[1]; mat(0, 2) = j[2];
    mat(1, 0) = j[3]; mat(1, 1) = j[4]; mat(1, 2) = j[5];
    mat(2, 0) = j[6]; mat(2, 1) = j[7]; mat(2, 2) = j[8];
}

// Custom types serialization

// SystemParams serialization
inline void to_json(json& j, const SystemParams& params) {
    // Create an empty object
    j = nlohmann::json::object();
    to_json(j["position"], params.position);
    to_json(j["impuls"], params.impuls);
    to_json(j["momentum"], params.momentum);
    to_json(j["inertialTensor"], params.inertialTensor);
    j["q"] = params.q;
}

inline void from_json(const json& j, SystemParams& params) {
    from_json(j["position"], params.position);
    from_json(j["impuls"], params.impuls);
    from_json(j["momentum"], params.momentum);
    from_json(j["inertialTensor"], params.inertialTensor);
    j["q"].get_to(params.q);
}

// Camera serialization
inline void to_json(json& j, const Camera& camera) {
    // Use a different approach for initializing the json object
    j = nlohmann::json::object();
    to_json(j["lastMousePos"],camera.lastMousePos);
    j["rotating"] = camera.rotating;
    j["angleX"] = camera.angleX;
    j["angleY"] = camera.angleY;
    j["zoom"] = camera.zoom;
}

inline void from_json(const json& j, Camera& camera) {
    // For complex types, we need to manually convert from json to the target type
    from_json(j["lastMousePos"], camera.lastMousePos);
    j["rotating"].get_to(camera.rotating);
    j["angleX"].get_to(camera.angleX);
    j["angleY"].get_to(camera.angleY);
    j["zoom"].get_to(camera.zoom);
}

// Particle serialization
inline void to_json(json& j, const Particle& particle) {
    // Use a different approach for initializing the json object
    j = nlohmann::json::object();
    
    // Add each field to the main json object
    to_json(j["position"] , particle.position);
    to_json(j["velocity"], particle.velocity);
    to_json(j["force"] , particle.force);
    j["realR"] = particle.realR;
    j["showR"] = particle.showR;
    j["active"] = particle.active;
    j["q"] = particle.q();
    j["trace"] = nlohmann::json::array();
}

inline void from_json(const json& j, Particle& particle) {
    from_json(j["position"] , particle.position);
    from_json(j["velocity"], particle.velocity);
    from_json(j["force"] , particle.force);
    j["realR"].get_to(particle.realR);
    j["showR"].get_to(particle.showR);
    j["active"].get_to(particle.active);
    double q_value;
    j["q"].get_to(q_value);
    particle.setQ(q_value);
}

// Simulation serialization
inline void to_json(json& j, const Simulation& sim) {
    // Use a different approach for initializing the json object
    j = nlohmann::json::object();
    
    // Add simple types directly
    j["numThreads"] = sim.numThreads;
    j["cTailSize"] = sim.cTailSize;
    j["totalPotentialEnergy"] = sim.totalPotentialEnergy;
    j["totalKineticEnergy"] = sim.totalKineticEnergy;
    j["inactiveCount"] = sim.inactiveCount;
    j["frameCount"] = sim.frameCount;
    j["frameCountPerTrace"] = sim.frameCountPerTrace;
    j["cubeScale"] = sim.cubeScale;
    
    // For complex types, create temporary jsons objects
    to_json(j["init"], sim.init);
    j["observerIndex"] = sim.observerIndex;
    to_json(j["observer"], sim.observer);
    to_json(j["camera"], sim.camera);

    to_json(j["particles"], sim.particles);
}

inline void from_json(const json& j, Simulation& sim) {
    // For simple types, we can use get_to
    j["numThreads"].get_to(sim.numThreads);
    j["cTailSize"].get_to(sim.cTailSize);
    j["totalPotentialEnergy"].get_to(sim.totalPotentialEnergy);
    j["totalKineticEnergy"].get_to(sim.totalKineticEnergy);
    j["inactiveCount"].get_to(sim.inactiveCount);
    j["frameCount"].get_to(sim.frameCount);
    j["frameCountPerTrace"].get_to(sim.frameCountPerTrace);
    j["cubeScale"].get_to(sim.cubeScale);

    printf("Simulation loaded.\n");
    
    // For complex types, we need to manually convert from json to the target type
    from_json(j["init"], sim.init);
    j["observerIndex"].get_to(sim.observerIndex);
    from_json(j["observer"], sim.observer);
    from_json(j["camera"], sim.camera);

    from_json(j["particles"], sim.particles);
}

#endif // GRAVITY3D_DATA_SERIALIZATION_H