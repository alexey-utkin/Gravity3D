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
    j = json{{"x", point.x}, {"y", point.y}};
}

inline void from_json(const json& j, cv::Point& point) {
    point.x = j["x"];
    point.y = j["y"];
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
    
    // Add each field individually
    // For complex types, we need to create temporary json objects
    nlohmann::json position_json = nlohmann::json::array({params.position[0], params.position[1], params.position[2]});
    nlohmann::json impuls_json = nlohmann::json::array({params.impuls[0], params.impuls[1], params.impuls[2]});
    nlohmann::json momentum_json = nlohmann::json::array({params.momentum[0], params.momentum[1], params.momentum[2]});
    
    // For the inertial tensor, we need to create a json array with all the elements
    nlohmann::json tensor_json = nlohmann::json::array({
        params.inertialTensor(0, 0), params.inertialTensor(0, 1), params.inertialTensor(0, 2),
        params.inertialTensor(1, 0), params.inertialTensor(1, 1), params.inertialTensor(1, 2),
        params.inertialTensor(2, 0), params.inertialTensor(2, 1), params.inertialTensor(2, 2)
    });
    
    // Add all fields to the main json object
    j["position"] = position_json;
    j["impuls"] = impuls_json;
    j["momentum"] = momentum_json;
    j["inertialTensor"] = tensor_json;
    j["q"] = params.q;
}

inline void from_json(const json& j, SystemParams& params) {
    // For complex types, we need to manually convert from json to the target type
    const auto& position_json = j.at("position");
    params.position = cv::Vec3d(position_json[0], position_json[1], position_json[2]);
    
    const auto& impuls_json = j.at("impuls");
    params.impuls = cv::Vec3d(impuls_json[0], impuls_json[1], impuls_json[2]);
    
    const auto& momentum_json = j.at("momentum");
    params.momentum = cv::Vec3d(momentum_json[0], momentum_json[1], momentum_json[2]);
    
    // For the inertial tensor, we need to manually set each element
    const auto& tensor_json = j.at("inertialTensor");
    params.inertialTensor(0, 0) = tensor_json[0];
    params.inertialTensor(0, 1) = tensor_json[1];
    params.inertialTensor(0, 2) = tensor_json[2];
    params.inertialTensor(1, 0) = tensor_json[3];
    params.inertialTensor(1, 1) = tensor_json[4];
    params.inertialTensor(1, 2) = tensor_json[5];
    params.inertialTensor(2, 0) = tensor_json[6];
    params.inertialTensor(2, 1) = tensor_json[7];
    params.inertialTensor(2, 2) = tensor_json[8];
    
    // For simple types, we can use get_to
    j.at("q").get_to(params.q);
}

// Camera serialization
inline void to_json(json& j, const Camera& camera) {
    // Use a different approach for initializing the json object
    j = nlohmann::json::object();
    
    // Create a temporary json object for the complex type
    nlohmann::json lastMousePos_json = nlohmann::json::object();
    lastMousePos_json["x"] = camera.lastMousePos.x;
    lastMousePos_json["y"] = camera.lastMousePos.y;
    
    // Add each field to the main json object
    j["lastMousePos"] = lastMousePos_json;
    j["rotating"] = camera.rotating;
    j["angleX"] = camera.angleX;
    j["angleY"] = camera.angleY;
    j["zoom"] = camera.zoom;
}

inline void from_json(const json& j, Camera& camera) {
    // For complex types, we need to manually convert from json to the target type
    const auto& lastMousePos_json = j.at("lastMousePos");
    camera.lastMousePos.x = lastMousePos_json["x"];
    camera.lastMousePos.y = lastMousePos_json["y"];
    
    // For simple types, we can use get_to
    j.at("rotating").get_to(camera.rotating);
    j.at("angleX").get_to(camera.angleX);
    j.at("angleY").get_to(camera.angleY);
    j.at("zoom").get_to(camera.zoom);
}

// Particle serialization
inline void to_json(json& j, const Particle& particle) {
    // Use a different approach for initializing the json object
    j = nlohmann::json::object();
    
    // Create temporary json objects for complex types
    nlohmann::json pos_json = nlohmann::json::array({particle.position[0], particle.position[1], particle.position[2]});
    nlohmann::json vel_json = nlohmann::json::array({particle.velocity[0], particle.velocity[1], particle.velocity[2]});
    nlohmann::json force_json = nlohmann::json::array({particle.force[0], particle.force[1], particle.force[2]});
    
    // Add each field to the main json object
    j["position"] = pos_json;
    j["velocity"] = vel_json;
    j["force"] = force_json;
    j["realR"] = particle.realR;
    j["showR"] = particle.showR;
    j["active"] = particle.active;
    j["q"] = particle.q();
    j["trace"] = nlohmann::json::array();
    
    // Serialize the trace
    for (const auto& point : particle.trace) {
        nlohmann::json point_json = nlohmann::json::array({point[0], point[1], point[2]});
        j["trace"].push_back(point_json);
    }
}

inline void from_json(const json& j, Particle& particle) {
    // For complex types, we need to manually convert from json to the target type
    const auto& position_json = j.at("position");
    particle.position = cv::Vec3d(position_json[0], position_json[1], position_json[2]);
    
    const auto& velocity_json = j.at("velocity");
    particle.velocity = cv::Vec3d(velocity_json[0], velocity_json[1], velocity_json[2]);
    
    const auto& force_json = j.at("force");
    particle.force = cv::Vec3d(force_json[0], force_json[1], force_json[2]);
    
    // For simple types, we can use get_to
    j.at("realR").get_to(particle.realR);
    j.at("showR").get_to(particle.showR);
    j.at("active").get_to(particle.active);
    
    double q_value;
    j.at("q").get_to(q_value);
    particle.setQ(q_value);
    
    // Deserialize the trace
    particle.trace.clear();
    for (const auto& point : j["trace"]) {
        // Each point is a json array with 3 elements
        cv::Vec3d vec(point[0], point[1], point[2]);
        particle.trace.push_back(vec);
    }
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
    
    // For complex types, create temporary json objects
    nlohmann::json init_json;
    to_json(init_json, sim.init);
    j["init"] = init_json;
    
    j["observerIndex"] = sim.observerIndex;
    
    nlohmann::json observer_json = nlohmann::json::array({sim.observer[0], sim.observer[1], sim.observer[2]});
    j["observer"] = observer_json;
    
    nlohmann::json camera_json;
    to_json(camera_json, sim.camera);
    j["camera"] = camera_json;
    
    j["particles"] = nlohmann::json::array();
    
    // Serialize the particles
    for (int i = 0; i < Simulation::cParticles; ++i) {
        nlohmann::json particle_json;
        to_json(particle_json, sim.particles[i]);
        j["particles"].push_back(particle_json);
    }
}

inline void from_json(const json& j, Simulation& sim) {
    // For simple types, we can use get_to
    j.at("numThreads").get_to(sim.numThreads);
    j.at("cTailSize").get_to(sim.cTailSize);
    j.at("totalPotentialEnergy").get_to(sim.totalPotentialEnergy);
    j.at("totalKineticEnergy").get_to(sim.totalKineticEnergy);
    j.at("inactiveCount").get_to(sim.inactiveCount);
    j.at("frameCount").get_to(sim.frameCount);
    j.at("frameCountPerTrace").get_to(sim.frameCountPerTrace);
    
    // For complex types, we need to manually convert from json to the target type
    from_json(j.at("init"), sim.init);
    j.at("observerIndex").get_to(sim.observerIndex);
    
    const auto& observer_json = j.at("observer");
    sim.observer = cv::Vec3d(observer_json[0], observer_json[1], observer_json[2]);
    
    from_json(j.at("camera"), sim.camera);
    
    // Deserialize the particles
    const auto& particles_json = j["particles"];
    for (int i = 0; i < Simulation::cParticles && i < particles_json.size(); ++i) {
        from_json(particles_json[i], sim.particles[i]);
    }
}

#endif // GRAVITY3D_DATA_SERIALIZATION_H