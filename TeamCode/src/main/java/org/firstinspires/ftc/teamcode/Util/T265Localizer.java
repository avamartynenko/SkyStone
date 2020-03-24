package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

@Config
public class T265Localizer implements Localizer {
    T265Wrapper localizer = new T265Wrapper();
    private Pose2d _poseEstimate = new Pose2d();

    @Override
    public void update() {
        localizer.refreshPoseData();
        _poseEstimate = new Pose2d(localizer.getRobotX(), localizer.getRobotY(), localizer.getYaw());
    }

    @Override
    public void setPoseEstimate(Pose2d newPose) {
        // TODO: investigate when/how this method is used and decide if it should be incorporated into pose correction
        _poseEstimate = newPose;
    }

    @Override
    public Pose2d getPoseEstimate() {
        return _poseEstimate;
    }
}