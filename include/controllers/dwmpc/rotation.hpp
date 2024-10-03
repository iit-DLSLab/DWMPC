#include <Eigen/Dense>

inline Eigen::Matrix3d  quatToRotMat(const Eigen::Quaterniond & q) {
    Eigen::Matrix3d R;
    R(0, 0) = -1.0 + 2.0 * (q.w() * q.w()) + 2.0 * (q.x() * q.x());
    R(1, 1) = -1.0 + 2.0 * (q.w() * q.w()) + 2.0 * (q.y() * q.y());
    R(2, 2) = -1.0 + 2.0 * (q.w() * q.w()) + 2.0 * (q.z() * q.z());
    R(0, 1) = 2.0 * (q.x() * q.y() + q.w() * q.z());
    R(0, 2) = 2.0 * (q.x() * q.z() - q.w() * q.y());
    R(1, 0) = 2.0 * (q.x() * q.y() - q.w() * q.z());
    R(1, 2) = 2.0 * (q.y() * q.z() + q.w() * q.x());
    R(2, 0) = 2.0 * (q.x() * q.z() + q.w() * q.y());
    R(2, 1) = 2.0 * (q.y() * q.z() - q.w() * q.x());

    return R;
}
Eigen::Vector3d inline rotTorpy(Eigen::Matrix3d R)
{
    Eigen::Vector3d rpy;
    rpy(0) = atan2f((float) R(1,2), (float) R(2,2));
    rpy(1) = -asinf((float) R(0,2));
    rpy(2) = atan2f((float) R(0,1), (float) R(0,0));

    return rpy;
}
Eigen::Quaterniond inline rpyToquat(const Eigen::Vector3d& rpy) {
    Eigen::Quaterniond q;
    double roll, pitch, yaw;

    roll = rpy(0);
    pitch = rpy(1);
    yaw = rpy(2);

    q.w() = cos(roll/2.0)*cos(pitch/2.0)*cos(yaw/2.0)+sin(roll/2.0)*sin(pitch/2.0)*sin(yaw/2.0);
    q.x() = sin(roll/2.0)*cos(pitch/2.0)*cos(yaw/2.0)-cos(roll/2.0)*sin(pitch/2.0)*sin(yaw/2.0);
    q.y() = cos(roll/2.0)*sin(pitch/2.0)*cos(yaw/2.0)+sin(roll/2.0)*cos(pitch/2.0)*sin(yaw/2.0);
    q.z() = cos(roll/2.0)*cos(pitch/2.0)*sin(yaw/2.0)-sin(roll/2.0)*sin(pitch/2.0)*cos(yaw/2.0);


    return q;
}
inline Eigen::Vector3d quatToRPY(const Eigen::Quaterniond & q){
    Eigen::Vector3d rpy;
    rpy = rotTorpy(quatToRotMat(q));
    return rpy;
}