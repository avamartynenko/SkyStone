package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Test util mode to measure robot's dimentions
 * Instructions:
 * Step 1: turn robot on and move around until it reaches high tracking confidence
 * Step 2: restart op mode to reset camera location to 0, rotate robot around its center
 * until if faces opposite direction. write down camera x and y. repeat step several times to get
 * average measurement, this will give you X and Y offset (doubled) from robot center to camera
 *
 */

@Config
@TeleOp(name="Util: Camera Position", group="Util")
public class T265TestOpMode extends LinearOpMode {

    public static final String TAG = "T265TestOpMode";

    public static double ORBITAL_FREQUENCY = 0.05;
    public static double SPIN_FREQUENCY = 0.25;

    public static double ORBITAL_RADIUS = 50;
    public static double SIDE_LENGTH = 10;

    private static void rotatePoints(double[] xPoints, double[] yPoints, double angle) {
        for (int i = 0; i < xPoints.length; i++) {
            double x = xPoints[i];
            double y = yPoints[i];
            xPoints[i] = x * Math.cos(angle) - y * Math.sin(angle);
            yPoints[i] = x * Math.sin(angle) + y * Math.cos(angle);
        }
    }

    @Override
    public void runOpMode()  {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        T265Wrapper localizer = new T265Wrapper();
        
        float[] location = localizer.refreshPoseData();

        telemetry.addData("CameraX", location[T265Wrapper.ixX]);
        telemetry.addData("CameraY", location[T265Wrapper.ixY]);
        telemetry.addData("RobotX", localizer.getRobotX());
        telemetry.addData("RobotY", localizer.getRobotY());
        telemetry.addData("Yaw", location[T265Wrapper.ixYaw]);

        waitForStart();

        if (isStopRequested())
            return;

        localizer.stopStream();
        sleep(250);
        localizer.startStream();
        sleep(250);

        while (opModeIsActive()) {
            location = localizer.refreshPoseData();
            
            /*double bx = ORBITAL_RADIUS * Math.cos(2 * Math.PI * ORBITAL_FREQUENCY * time);
            double by = ORBITAL_RADIUS * Math.sin(2 * Math.PI * ORBITAL_FREQUENCY * time);*/

            telemetry.addData("CameraX", "%.2f", location[T265Wrapper.ixX]);
            telemetry.addData("CameraY", "%.2f", location[T265Wrapper.ixY]);
            telemetry.addData("RobotX", "%.2f", localizer.getRobotX());
            telemetry.addData("RobotY", "%.2f", localizer.getRobotY());

            telemetry.addData("Yaw", "%.2f", location[T265Wrapper.ixYaw]);

            telemetry.addData("Tracker Confidence", localizer.getTrackerConfidenceText());
            telemetry.update();

            double bx = location[0];
            double by = location[1];

            double l = SIDE_LENGTH / 2;

            double[] bxPoints = { l, -l, -l, l };
            double[] byPoints = { l, l, -l, -l };
            rotatePoints(bxPoints, byPoints, location[2]);
            for (int i = 0; i < 4; i++) {
                bxPoints[i] += bx;
                byPoints[i] += by;
            }

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay()
                    .setStrokeWidth(1)
                    .setStroke("goldenrod")
                    .strokeCircle(bx, by, ORBITAL_RADIUS)
                    .setFill("black")
                    .fillPolygon(bxPoints, byPoints);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
