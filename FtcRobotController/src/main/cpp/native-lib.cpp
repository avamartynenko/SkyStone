#include <jni.h>
#include <cmath>
#include <mutex>
#include <thread>
#include <iomanip>
#include <android/log.h>

#include "librealsense2/rs.hpp"

#define TAG "SM native_example"

static const float meters2inches = 39.3701;

// https://cs.stanford.edu/~acoates/quaternion.h

#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR,    TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN,     TAG, __VA_ARGS__)
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,     TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG,    TAG, __VA_ARGS__)

//rs2::context ctx;

extern "C" JNIEXPORT jstring JNICALL
Java_org_firstinspires_ftc_robotcontroller_internal_FtcRobotControllerActivity_nGetLibrealsenseVersionFromJNI(JNIEnv *env, jclass type) {
    return (*env).NewStringUTF(RS2_API_VERSION_STR);
}

extern "C" JNIEXPORT jint JNICALL
Java_com_example_realsense_1native_1example_MainActivity_nGetCamerasCountFromJNI(JNIEnv *env, jclass type) {
    rs2::context ctx;
    int sensorsCount = ctx.query_devices().size();
    return sensorsCount; //ctx.query_devices().size();
}

extern "C" JNIEXPORT jint JNICALL
Java_com_example_realsense_1native_1example_MainActivity_nGetSensorsCountFromJNI(JNIEnv *env, jclass type) {
    rs2::context ctx;
    return ctx.query_all_sensors().size();
}

std::mutex g_data_mutex;

rs2::pipeline pipe;
rs2::config cfg;

extern "C" JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_robotcontroller_internal_FtcRobotControllerActivity_nStartStream(JNIEnv *env, jclass type) {
    rs2::context ctx;
    if(ctx.query_devices().size() == 0) {
        LOGE("No pose devices available");
        return;
    }

    cfg.disable_all_streams();
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

    // Start pipeline with chosen configuration
    pipe.start(cfg);
}

extern "C" JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_robotcontroller_internal_FtcRobotControllerActivity_nStopStream(JNIEnv *env, jclass type) {
    try {
        LOGI("Stopping stream");
        pipe.stop();
        cfg.disable_all_streams();
    }
    catch(...) {
        LOGE("Unable to stop stream");
    }
}

// https://programmer.help/blogs/android-jni-array-operation.html
extern "C" JNIEXPORT jfloatArray  JNICALL
Java_org_firstinspires_ftc_robotcontroller_internal_FtcRobotControllerActivity_nGetCameraPoseData(JNIEnv *env, jclass type) {
    rs2::context ctx;
    if(ctx.query_devices().size() == 0) {
        LOGE("nGetCameraPoseData: No pose devices available");
        return NULL;
    }

    try {
        rs2::frameset frames;
    //auto frames = pipe.wait_for_frames(&fs);
        if(!pipe.try_wait_for_frames(&frames)) {
            LOGE("nGetCameraPoseData: try_wait_for_frames fail");
            return NULL;
        }
    // Get a frame from the pose stream
    auto f = frames.first_or_default(RS2_STREAM_POSE);
    // Cast the frame to pose_frame and get its data
    rs2_pose pose_data;

    {
//        std::lock_guard<std::mutex> lock(g_data_mutex);
        pose_data = f.as<rs2::pose_frame>().get_pose_data();
    }

    jfloatArray result;
    float pose_data_array[] = {
            pose_data.translation.x,    // 0
            pose_data.translation.y,
            pose_data.translation.z,

            pose_data.velocity.x,       // 3
            pose_data.velocity.y,
            pose_data.velocity.z,

            pose_data.acceleration.x,   // 6
            pose_data.acceleration.y,
            pose_data.acceleration.z,

            pose_data.rotation.x,       // 9
            pose_data.rotation.y,
            pose_data.rotation.z,
            pose_data.rotation.w,

            pose_data.angular_velocity.x, // 13
            pose_data.angular_velocity.y,
            pose_data.angular_velocity.z,

            pose_data.angular_acceleration.x, //16
            pose_data.angular_acceleration.y,
            pose_data.angular_acceleration.z,

            // actual values are unsigned int, we will use float to simplify passing data back and forth
            (float) pose_data.tracker_confidence, //19
            (float) pose_data.mapper_confidence,
            (float) f.get_frame_number()
    };
    int pose_elements = sizeof(pose_data_array)/ sizeof(pose_data_array[0]);
    result = env->NewFloatArray(pose_elements);
    if (result == NULL) {
        return NULL; /* out of memory error thrown */
    }

    env->SetFloatArrayRegion(result, 0, pose_elements, pose_data_array);
    return result;
    }
    catch(const rs2::wrong_api_call_sequence_error& ex) {
        LOGE("Unable to get frames");
        return NULL;
    }
}

extern "C" JNIEXPORT jstring JNICALL
Java_org_firstinspires_ftc_robotcontroller_internal_FtcRobotControllerActivity_nGetCameraPose(JNIEnv *env, jclass type) {

    rs2::context ctx;
    if(ctx.query_devices().size() == 0) {
        LOGE("nGetCameraPose: No pose devices available");
        return NULL;
    }

    //auto frames = pipe.wait_for_frames();

    rs2::frameset frames;
    try {
        if(!pipe.try_wait_for_frames(&frames)) {
            LOGE("nGetCameraPose: try_wait_for_frames fail");
            return NULL;
        }
    }
    catch(...) {
        LOGE("nGetCameraPose: try_wait_for_frames exception");
        return NULL;
    }

    // Get a frame from the pose stream
    auto f = frames.first_or_default(RS2_STREAM_POSE);
    // Cast the frame to pose_frame and get its data
    rs2_pose pose_data;

    {
//        std::lock_guard<std::mutex> lock(g_data_mutex);
        pose_data = f.as<rs2::pose_frame>().get_pose_data();
    }

    std::stringstream strPose;
    strPose << "Device Position (meters): " << std::endl << std::setprecision(3) << std::fixed;
    strPose << "\tx=" << pose_data.translation.x << std::endl;
    strPose << "\ty=" << pose_data.translation.y << std::endl;
    strPose << "\tz=" << pose_data.translation.z << std::endl;

    strPose << "Rotation: " << std::endl;
    strPose << "\tx=" << pose_data.rotation.x << std::endl;
    strPose << "\ty=" << pose_data.rotation.y << std::endl;
    strPose << "\tz=" << pose_data.rotation.z << std::endl;
    strPose << "\tw=" << pose_data.rotation.w << std::endl;

    strPose << "Velocity (m/s): " << std::endl;
    strPose << "\tx=" << pose_data.velocity.x << std::endl;
    strPose << "\ty=" << pose_data.velocity.y << std::endl;
    strPose << "\tz=" << pose_data.velocity.z << std::endl;
    strPose << "\tglobal=" << sqrt(pow(pose_data.velocity.x, 2) + pow(pose_data.velocity.y, 2) + pow(pose_data.velocity.z, 2)) << std::endl;

    strPose << "Confidence: " << std::endl;
    strPose << "\tmapper=" << pose_data.mapper_confidence;
    strPose << "\ttracker=" << pose_data.tracker_confidence;

    return (*env).NewStringUTF(strPose.str().c_str());
}

/*
 * return robot positon in float array
 * 0 - robot x (inches)
 * 1 - robot y (inches)
 * 2 - robot heading (-pi .. +pi)
 */
extern "C" JNIEXPORT jfloatArray JNICALL
Java_org_firstinspires_ftc_robotcontroller_internal_FtcRobotControllerActivity_nGetCameraPoseXYYaw(JNIEnv *env, jclass type) {

    rs2::context ctx;
    if(ctx.query_devices().size() == 0) {
        LOGE("nGetCameraPoseXYYaw: No pose devices available");
        return NULL;
    }

    auto frames = pipe.wait_for_frames();
    // Get a frame from the pose stream
    auto f = frames.first_or_default(RS2_STREAM_POSE);
    // Cast the frame to pose_frame and get its data
    rs2_pose pose_data;

    {
//        std::lock_guard<std::mutex> lock(g_data_mutex);
        pose_data = f.as<rs2::pose_frame>().get_pose_data();
    }

    float yaw = atan2(pose_data.rotation.y, pose_data.rotation.w) * 2;
    while (yaw < -M_PI) yaw += 2 * M_PI;
    while (yaw > M_PI)  yaw -= 2 * M_PI;

    jfloatArray result;

    /*
     * https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md
     * https://github.com/IntelRealSense/librealsense/raw/master/doc/img/T265_sensor_extrinsics.png
     * Positive X direction is towards right imager
     * Positive Y direction is upwards toward the top of the device
     * Positive Z direction is inwards toward the back of the device
     *
     * Assuming that camera is mounted on the front of the robot and faces
     * in "forward" direction of the robot:
     *
     *  Device -X -> robot's +Y
     *  Device  Y -> ignored (robot's height)
     *  Device -Z -> robot's +X
     *
     *
     *  |Y      _L__
     *  |    B |____| F
     *  |        R
     *  |______________X
     *
     *
     */
    float pose_data_array[] = {
            -pose_data.translation.z * meters2inches, // camera's z matches robot's x
            -pose_data.translation.x * meters2inches, // camera's x matches robot's y
            yaw,
            // actual values are unsigned int, we will use float to simplify passing data back and forth
            (float) pose_data.tracker_confidence,
            (float) pose_data.mapper_confidence,
            (float) f.get_frame_number()
    };

    int pose_elements = sizeof(pose_data_array)/ sizeof(pose_data_array[0]);
    result = env->NewFloatArray(pose_elements);
    if (result == NULL) {
        return NULL; /* out of memory error thrown */
    }

    env->SetFloatArrayRegion(result, 0, pose_elements, pose_data_array);
    return result;
}