package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        T265Wrapper localizer = new T265Wrapper();
        
        float[] location = localizer.refreshPoseData();

        telemetry.addData("X", location[0]);
        telemetry.addData("Y", location[1]);
        telemetry.addData("Yaw", location[2]);
        waitForStart();

        if (isStopRequested()) return;

        ElapsedTime et = new ElapsedTime();
        double minRead = 1000;
        double maxRead = 0;
        long loopNumber = 0;
        double totalReadTime = 0;

        while (opModeIsActive()) {
            loopNumber ++;
            double time = getRuntime();
            //Log.i(TAG, "Sending request to get pose data... time=" + time);
            //et.reset();
            location = localizer.refreshPoseData();
            /*double readTime = et.milliseconds();
            totalReadTime += readTime;
            if(readTime > maxRead) maxRead = readTime;
            if(readTime < minRead) minRead = readTime;
            telemetry.addData("maxRead", maxRead);
            telemetry.addData("minRead", minRead);
            telemetry.addData("avgRead", totalReadTime/loopNumber);
            telemetry.update();*/

            /*double bx = ORBITAL_RADIUS * Math.cos(2 * Math.PI * ORBITAL_FREQUENCY * time);
            double by = ORBITAL_RADIUS * Math.sin(2 * Math.PI * ORBITAL_FREQUENCY * time);*/

            telemetry.addData("X", "%.2f", location[0]);
            telemetry.addData("Y", "%.2f", location[1]);
            telemetry.addData("Yaw", "%.2f", location[2]);
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
