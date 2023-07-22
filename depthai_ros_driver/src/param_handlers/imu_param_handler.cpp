#include "depthai_ros_driver/param_handlers/imu_param_handler.hpp"

#include "depthai/pipeline/node/IMU.hpp"
#include "depthai-shared/properties/IMUProperties.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
ImuParamHandler::ImuParamHandler(rclcpp::Node* node, const std::string& name) : BaseParamHandler(node, name) {}
ImuParamHandler::~ImuParamHandler() = default;
void ImuParamHandler::declareParams(std::shared_ptr<dai::node::IMU> imu, const std::string& imuType) {
    imuSyncMethodMap = {
        {"COPY", dai::ros::ImuSyncMethod::COPY},
        {"LINEAR_INTERPOLATE_GYRO", dai::ros::ImuSyncMethod::LINEAR_INTERPOLATE_GYRO},
        {"LINEAR_INTERPOLATE_ACCEL", dai::ros::ImuSyncMethod::LINEAR_INTERPOLATE_ACCEL}};
    imuMessagetTypeMap = {
        {"IMU", imu::ImuMsgType::IMU},
        {"IMU_WITH_MAG", imu::ImuMsgType::IMU_WITH_MAG},
        {"IMU_WITH_MAG_SPLIT", imu::ImuMsgType::IMU_WITH_MAG_SPLIT}};
    imuAccelerometerModeMap = {
        {"RAW", dai::IMUSensor::ACCELEROMETER_RAW},
        {"CALIBRATED", dai::IMUSensor::ACCELEROMETER},
        {"LINEAR", dai::IMUSensor::LINEAR_ACCELERATION},
        {"GRAVITY", dai::IMUSensor::GRAVITY}};
    imuGyroscopeModeMap = {
        {"RAW", dai::IMUSensor::GYROSCOPE_RAW},
        {"CALIBRATED", dai::IMUSensor::GYROSCOPE_CALIBRATED},
        {"UNCALIBRATED", dai::IMUSensor::GYROSCOPE_UNCALIBRATED}};
    imuMagnetometerModeMap = {
        {"RAW", dai::IMUSensor::MAGNETOMETER_RAW},
        {"CALIBRATED", dai::IMUSensor::MAGNETOMETER_CALIBRATED},
        {"UNCALIBRATED", dai::IMUSensor::MAGNETOMETER_UNCALIBRATED}};
    imuRotationModeMap = {
        {"DEFAULT", dai::IMUSensor::ROTATION_VECTOR},
        {"GAME", dai::IMUSensor::GAME_ROTATION_VECTOR},
        {"GEOMAGNETIC", dai::IMUSensor::GEOMAGNETIC_ROTATION_VECTOR},
        {"ARVR_STABILIZED", dai::IMUSensor::ARVR_STABILIZED_ROTATION_VECTOR},
        {"ARVR_STABILIZED_GAME", dai::IMUSensor::ARVR_STABILIZED_GAME_ROTATION_VECTOR}};

    declareAndLogParam<bool>("i_get_base_device_timestamp", false);
    declareAndLogParam<std::string>("i_message_type", "IMU");
    declareAndLogParam<std::string>("i_sync_method", "LINEAR_INTERPOLATE_ACCEL");

    if (declareAndLogParam<bool>("i_enable_acc", true)) {
        const std::string accelerometerModeName =
            utils::getUpperCaseStr(declareAndLogParam<std::string>("i_acc_mode", "raw"));
        const dai::IMUSensor accelerometerMode =
            utils::getValFromMap(accelerometerModeName, imuAccelerometerModeMap);
        const int accelerometerFreq = declareAndLogParam<int>("i_acc_freq", 400);
        declareAndLogParam<float>("i_acc_cov", 0.0);

        imu->enableIMUSensor(accelerometerMode, accelerometerFreq);
    }

    if (declareAndLogParam<bool>("i_enable_gyro", true)) {
        const std::string gyroscopeModeName =
            utils::getUpperCaseStr(declareAndLogParam<std::string>("i_gyro_mode", "raw"));
        const dai::IMUSensor gyroscopeMode =
            utils::getValFromMap(gyroscopeModeName, imuGyroscopeModeMap);
        const int gyroscopeFreq = declareAndLogParam<int>("i_gyro_freq", 400);
        declareAndLogParam<float>("i_gyro_cov", 0.0);

        imu->enableIMUSensor(gyroscopeMode, gyroscopeFreq);
    }

    const bool magnetometerAvailable = imuType == "BNO086";
    if (declareAndLogParam<bool>("i_enable_mag", magnetometerAvailable)) {
        if (magnetometerAvailable) {
            const std::string magnetometerModeName =
                utils::getUpperCaseStr(declareAndLogParam<std::string>("i_mag_mode", "raw"));
            const dai::IMUSensor magnetometerMode =
                utils::getValFromMap(magnetometerModeName, imuMagnetometerModeMap);
            const int magnetometerFreq = declareAndLogParam<int>("i_mag_freq", 100);
            declareAndLogParam<float>("i_mag_cov", 0.0);

            imu->enableIMUSensor(magnetometerMode, magnetometerFreq);
        } else {
            RCLCPP_ERROR(getROSNode()->get_logger(), "Magnetometer enabled but not available with current sensor");
            declareAndLogParam<bool>("i_enable_mag", false, true);
        }
    }

    const bool rotationAvailable = imuType == "BNO086";
    if (declareAndLogParam<bool>("i_enable_rotation", false)) {
        if (rotationAvailable) {
            const std::string rotationModeName =
                utils::getUpperCaseStr(declareAndLogParam<std::string>("i_rot_mode", "default"));
            const dai::IMUSensor rotationMode =
                utils::getValFromMap(rotationModeName, imuRotationModeMap);
            const int rotationFreq = declareAndLogParam<int>("i_rot_freq", 400);
            declareAndLogParam<float>("i_rot_cov", -1.0);

            imu->enableIMUSensor(rotationMode, rotationFreq);
        } else {
            RCLCPP_ERROR(getROSNode()->get_logger(), "Rotation enabled but not available with current sensor");
            declareAndLogParam<bool>("i_enable_rotation", false, true);
        }
    }

    imu->setBatchReportThreshold(declareAndLogParam<int>("i_batch_report_threshold", 1));
    imu->setMaxBatchReports(declareAndLogParam<int>("i_max_batch_reports", 10));
}

dai::ros::ImuSyncMethod ImuParamHandler::getSyncMethod() {
    return utils::getValFromMap(utils::getUpperCaseStr(getParam<std::string>("i_sync_method")), imuSyncMethodMap);
}

imu::ImuMsgType ImuParamHandler::getMsgType() {
    return utils::getValFromMap(utils::getUpperCaseStr(getParam<std::string>("i_message_type")), imuMessagetTypeMap);
}

dai::CameraControl ImuParamHandler::setRuntimeParams(const std::vector<rclcpp::Parameter>& /*params*/) {
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver
