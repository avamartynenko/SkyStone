package org.firstinspires.ftc.teamcode.util;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.jetbrains.annotations.Nullable;

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
        //Log.d(TAG, String.format("update: _poseEstimateCorrection ix x=%.2f y=%.2f yaw=%.1f°", _poseEstimateCorrection.getX(), _poseEstimateCorrection.getY(), Math.toDegrees(_poseEstimateCorrection.getHeading())));
        //Log.d(TAG, String.format("update: localizer robot ix x=%.2f y=%.2f yaw=%.1f°", localizer.getRobotX(), localizer.getRobotY(), Math.toDegrees(localizer.getYaw())));

        float xDiff = localizer.getRobotX() - (float) _poseEstimateCorrection.getX();
        float yDiff = localizer.getRobotY() - (float) _poseEstimateCorrection.getY();
        float yawDiff = (float) _poseEstimateCorrection.getHeading();

        //Log.d(TAG, String.format("update: localizer robot ix xDiff=%.2f yDiff=%.2f yawDiff=%.1f°", xDiff, yDiff, Math.toDegrees(yawDiff)));

        _poseEstimate = new Pose2d( xDiff * Math.cos(yawDiff) + yDiff * Math.sin(yawDiff),
                -xDiff * Math.sin(yawDiff) + yDiff * Math.cos(yawDiff),
                normalizeYaw(localizer.getYaw() - _poseEstimateCorrection.getHeading()));

        //Log.d(TAG, String.format("update: pose set to x=%.2f y=%.2f yaw=%.1f°", _poseEstimate.getX(), _poseEstimate.getY(), Math.toDegrees(_poseEstimate.getHeading())));
    }

    @Override
    public void setPoseEstimate(Pose2d newPose) {
        if(newPose.getHeading() != 0) {
            Log.wtf(TAG, String.format("setPoseEstimate: x=%.2f y=%.2f yaw=%.1f°", newPose.getX(), newPose.getY(), newPose.getHeading()));
            throw new IllegalArgumentException("newPose with non zero heading is not supported");
        }

        localizer.refreshPoseData();
        //Log.d(TAG, String.format("setPoseEstimate: localizer robot ix x=%.2f y=%.2f yaw=%.1f°", localizer.getRobotX(), localizer.getRobotY(), Math.toDegrees(localizer.getYaw())));
        _poseEstimateCorrection = new Pose2d(localizer.getRobotX() - newPose.getX(), localizer.getRobotY() - newPose.getY(), localizer.getYaw());
    }

    @Override
    public Pose2d getPoseEstimate() {
        //Log.d(TAG, String.format("getPoseEstimate: x=%.2f y=%.2f yaw=%.1f°", _poseEstimate.getX(), _poseEstimate.getY(), Math.toDegrees(_poseEstimate.getHeading())));
        return _poseEstimate;
    }

    private double normalizeYaw(double Yaw) {
        while(Yaw > Math.PI)
            Yaw -= Math.PI *2;
        return Yaw;
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }
}
