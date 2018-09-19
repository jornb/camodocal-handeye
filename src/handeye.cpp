#include "handeye.hpp"

#include "calib/HandEyeCalibration.h"

using namespace handeye;

RigidTransform RigidTransform::from_matrix(const Eigen::Matrix4d &other) {
    Eigen::Matrix3d R = other.block(0, 0, 3, 3);

    Eigen::AngleAxisd r(R);
    Eigen::Translation3d t(other.block(0, 3, 3, 1));

    return { r, t };
}

RigidTransform RigidTransform::from_affine(const Eigen::Affine3d &other) {
    return from_matrix(other.matrix());
}

RigidTransform RigidTransform::from_pose(const Eigen::Matrix<double, 6, 1> &other) {
    Eigen::Vector3d t_vec = other.block<3, 1>(0, 0);
    Eigen::Vector3d r_vec = other.block<3, 1>(3, 0);

    Eigen::AngleAxisd r(r_vec.norm(), r_vec.normalized());
    Eigen::Translation3d t(t_vec);
    return { r, t };
}

Eigen::Vector3d RigidTransform::get_rotation_vector() const {
    return rotation.axis() * rotation.angle(); 
}

Eigen::Vector3d RigidTransform::get_translation_vector() const {
    return translation.vector();
}

Eigen::Affine3d RigidTransform::get_affine_transform() const {
    Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
    
    H.block<3, 3>(0, 0) = rotation.matrix();
    H.block<3, 1>(0, 3) = translation.vector();

    return Eigen::Affine3d(H);
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
