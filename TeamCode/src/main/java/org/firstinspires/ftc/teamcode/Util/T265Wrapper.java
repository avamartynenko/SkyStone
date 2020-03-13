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
        // read time averages around 5ms, with rare spikes up to 12ms
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

    public int getTrackerConfidence() {
        return (int) cameraPose[3];
    }

    public int getMapperConfidence() {
        return (int) cameraPose[4];
    }

    public String getTrackerConfidenceText() {
        switch (getTrackerConfidence()) {
            case 3:
                return "High";
            case 2:
                return "Medium";
            case 1:
                return "Low";
            default:
                return "Fail";
        }
    }

    public String getMapperConfidenceText() {
        switch (getMapperConfidence()) {
            case 3:
                return "High";
            case 2:
                return "Medium";
            case 1:
                return "Low";
            default:
                return "Fail";
        }
    }
}
