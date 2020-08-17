package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import static java.lang.Math.*;
import static java.lang.Thread.sleep;

public class T265Wrapper {
    private float[] cameraPose;

    // camera offset relative to robot's center
    private float offsetX, offsetY;

    // distance from robot center to camera center
    private float offsetRadius;

    private float alignmentError, rotationCos, rotationSin, offsetYaw;

    // robot current coordinates relative to robot center
    private float robotX, robotY;

    // robot starting coordinates relative to camera (accounting to mounting error)
    private float startX, startY;

    final public static int ixX = 0;
    final public static int ixY = 1;
    final public static int ixYaw = 2;
    final public static int ixFrameNumber = 5;

    public T265Wrapper() {
        offsetX = 7.64f; // robot center relative to camera
        offsetY = 6.22f;
        offsetRadius = (float) sqrt(pow(offsetX, 2) + pow(offsetY, 2));
        offsetYaw = (float) Math.atan2(-offsetY, -offsetX);

        // enter coordinates reported by camera censor when traveled in the straight line along robots X axis
        // .5 degree error in camera alignment results in about 1 inch tracking error travelling across the field
        alignmentError = -(float) Math.atan2(1, 110);

        // adjust robot center direction angle from camera to mounting error
        offsetYaw -= alignmentError;

        // calculate start coordinates
        startX = offsetRadius * (float) cos(offsetYaw);
        startY = offsetRadius * (float) sin(offsetYaw);

        // calculate rotational cos and sin for future use
        rotationCos = (float) cos(alignmentError);
        rotationSin = (float) sin(alignmentError);
    }

    public void startStream() {
        FtcRobotControllerActivity.StartStream();
    }

    public void stopStream() {
        FtcRobotControllerActivity.StopStream();
    }

    public float[] refreshPoseData() {
        // read time averages around 5ms, with rare spikes up to 12ms
        cameraPose = FtcRobotControllerActivity.GetCameraPoseXYYaw();

        float rotationX = cameraPose[ixX] + offsetRadius * (float) cos(offsetYaw + cameraPose[ixYaw]);
        float rotationY = cameraPose[ixY] + offsetRadius * (float) sin(offsetYaw + cameraPose[ixYaw]);

        // adjust to robot starting position
        // now we have coordinates of the robot before adjusting to angle between coordinate planes
        rotationX -= startX;
        rotationY -= startY;

        robotX = rotationX * rotationCos - rotationY * rotationSin;
        robotY = rotationX * rotationSin + rotationY * rotationCos;

        return cameraPose;
    }

    public float getCameraX() {
        return cameraPose[ixX];
    }

    public float getRobotX() {
        return robotX;
    }

    public float getRobotY() { return robotY; }

    public float getCameraY() {
        return cameraPose[ixY];
    }

    public float getYaw() {
        return cameraPose[ixYaw];
    }

    public int getTrackerConfidence() {
        return (int) cameraPose[3];
    }

    public int getMapperConfidence() {
        return (int) cameraPose[4];
    }

    public long getFrameNumber() {
        return (long) cameraPose[ixFrameNumber];
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

    public void restart() throws InterruptedException {
        stopStream();
        sleep(1000);
        startStream();
    }
}
