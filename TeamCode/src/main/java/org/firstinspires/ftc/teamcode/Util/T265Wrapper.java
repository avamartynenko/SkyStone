package org.firstinspires.ftc.teamcode.Util;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

public class T265Wrapper {
    private float[] cameraPose;
    public void startStream() {
        //FtcRobotControllerActivity.StartStream();
    }

    public void stopStream() {
        FtcRobotControllerActivity.StopStream();
    }

    public float[] refreshPoseData() {
        cameraPose = FtcRobotControllerActivity.GetCameraPoseXYYaw();
        return cameraPose;
    }

    public float getX() {
        return cameraPose[0];
    }

    public float getY() {
        return cameraPose[1];
    }

    public float getYaw() {
        return cameraPose[2];
    }
}
