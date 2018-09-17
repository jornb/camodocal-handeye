#include "handeye.hpp"

#include "calib/HandEyeCalibration.h"

using namespace handeye;

RigidTransform RigidTransform::from_matrix(const Eigen::Matrix4d &other)
{
    Eigen::Affine3d a(other);

    Eigen::AngleAxisd r;
    r.fromRotationMatrix(a.rotation());

    Eigen::Translation3d t(a.translation());

    return { r, t };
}

RigidTransform RigidTransform::from_affine(const Eigen::Affine3d &other) {
    return from_matrix(other.matrix());
}

Eigen::Vector3d RigidTransform::get_rotation_vector() const {
    return rotation.axis() * rotation.angle(); 
}

Eigen::Vector3d RigidTransform::get_translation_vector() const {
    return translation.vector();
}

Eigen::Affine3d RigidTransform::get_affine_transform() const {
    Eigen::Affine3d X = Eigen::Affine3d::Identity();
    X.translate(get_translation_vector());
    X.rotate(rotation.matrix());
    return X;
}

bool handeye::estimate_hand_to_eye(
    const std::vector<RigidTransform> &hand_to_base,
    const std::vector<RigidTransform> &eye_to_world,
    RigidTransform &result,
    bool planar_motion)
{
    if (hand_to_base.size() != eye_to_world.size() || hand_to_base.size() < 3)
        return false;

    size_t N = hand_to_base.size();
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> rvecs1, rvecs2;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> tvecs1, tvecs2;

    for (size_t i = 1; i < N; i++) {
        auto h2b = RigidTransform::from_affine(hand_to_base[0].get_affine_transform().inverse() * hand_to_base[i].get_affine_transform());
        auto e2w = RigidTransform::from_affine(eye_to_world[0].get_affine_transform().inverse() * eye_to_world[i].get_affine_transform());

        rvecs1.push_back(h2b.get_rotation_vector());
        tvecs1.push_back(h2b.get_translation_vector());

        rvecs2.push_back(e2w.get_rotation_vector());
        tvecs2.push_back(e2w.get_translation_vector());
    }

    Eigen::Matrix4d tmp;
    camodocal::HandEyeCalibration::estimateHandEyeScrew(rvecs1, tvecs1, rvecs2, tvecs2, tmp, planar_motion);
    result = RigidTransform::from_matrix(tmp.inverse());
    return true;
}