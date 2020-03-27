package org.firstinspires.ftc.teamcode.util;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

@Config
public class T265Localizer implements Localizer {
    private static final String TAG = "T265Localizer";

    T265Wrapper localizer = new T265Wrapper();
    private Pose2d _poseEstimate = new Pose2d();
    private Pose2d _poseEstimateCorrection = new Pose2d();

    public T265Localizer() {
        setPoseEstimate(new Pose2d(0, 0,0));
    }

    @Override
    public void update() {
        localizer.refreshPoseData();
        _poseEstimate = new Pose2d( localizer.getRobotX() + _poseEstimateCorrection.getX(),
                                    localizer.getRobotY() + _poseEstimateCorrection.getY(),
                                    localizer.getYaw() + _poseEstimateCorrection.getHeading());
        Log.d(TAG, String.format("update: pose set to x=%.2f y=%.2f yaw=%.2f", _poseEstimate.getX(), _poseEstimate.getY(), _poseEstimate.getHeading()));
    }

    @Override
    public void setPoseEstimate(Pose2d newPose) {
        // TODO: investigate when/how this method is used and decide if it should be incorporated into pose correction
        Log.wtf(TAG, String.format("setPoseEstimate: x=%.2f y=%.2f yaw=%.2f", newPose.getX(), newPose.getY(), newPose.getHeading()));
        localizer.refreshPoseData();
        _poseEstimateCorrection = new Pose2d(localizer.getRobotX() - newPose.getX(), localizer.getRobotY() - newPose.getY(), localizer.getYaw() - newPose.getHeading());
    }

    @Override
    public Pose2d getPoseEstimate() {
        Log.d(TAG, String.format("getPoseEstimate: x=%.2f y=%.2f yaw=%.2f", _poseEstimate.getX(), _poseEstimate.getY(), _poseEstimate.getHeading()));
        return _poseEstimate;
    }
}
