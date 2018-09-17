#include <vector>
#include <Eigen/Dense>

namespace handeye {
    struct RigidTransform {
        Eigen::AngleAxisd rotation;
        Eigen::Translation3d translation;

        static RigidTransform from_matrix(const Eigen::Matrix4d &other);
        static RigidTransform from_affine(const Eigen::Affine3d &other);

        Eigen::Vector3d get_rotation_vector() const;
        Eigen::Vector3d get_translation_vector() const;
        Eigen::Affine3d get_affine_transform() const;
    };

    /// Perform hand-eye calibration
    /// 
    /// Knowns:
    ///     * hand to base  (read by robot)
    ///     * eye to world  (estimated with checkerboard etc.)
    ///
    /// Unknowns:
    ///     * hand to eye
    ///     * (base to world)
    bool estimate_hand_to_eye(
        const std::vector<RigidTransform> &hand_to_base,
        const std::vector<RigidTransform> &eye_to_world,
        RigidTransform &result,
        bool planar_motion = false);
}
