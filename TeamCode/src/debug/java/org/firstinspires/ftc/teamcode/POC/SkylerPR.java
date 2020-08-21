package org.firstinspires.ftc.teamcode.POC;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.SimpleMotorEx;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.T265Localizer;
import org.firstinspires.ftc.teamcode.util.T265Wrapper;

import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;

@Config
@Autonomous(name="POC: Pure Pursuit", group="POC")
public class SkylerPR extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        telemetry = dashboard.getTelemetry();
        T265Localizer localizer = new T265Localizer();

        SimpleMotorEx fL = new SimpleMotorEx("frontLeft", hardwareMap, 435);
        SimpleMotorEx fR = new SimpleMotorEx("frontRight", hardwareMap, 435);
        SimpleMotorEx bL = new SimpleMotorEx("backLeft", hardwareMap, 435);
        SimpleMotorEx bR = new SimpleMotorEx("backRight", hardwareMap, 435);

        bL.setInverted(true);
        fL.setInverted(true);

        MecanumDrive dt = new MecanumDrive(false, fL, fR, bL, bR);

        localizer.update();
        float startX = 0;
        float startY = 0;

        waitForStart();

        if (isStopRequested()) return;

        double movSpeed = .4;
        double turnSpeed = .5;

        // With X and Y coordinates.
        Waypoint p1 = new StartWaypoint(0, 0);
        //Waypoint p2 = new GeneralWaypoint(10, startY + 3, movSpeed, turnSpeed, 3);
        /*Waypoint p3 = new GeneralWaypoint(startX + 20, startY - 3, movSpeed, turnSpeed, 3);
        Waypoint p4 = new GeneralWaypoint(startX + 30, startY + 3, movSpeed, turnSpeed, 3);*/
        Waypoint p5 = new EndWaypoint(startX + 40, startY, 0, movSpeed, turnSpeed, 3, 3, .5);

        if(true) {

            // we are using the waypoints we made in the above examples
            Path m_path = new Path(p1, p5);
            m_path.init(); // initialize the path

            while (opModeIsActive()) {
                while (!m_path.isFinished() && opModeIsActive()) {
                    packet = new TelemetryPacket();
                    Canvas fieldOverlay = packet.fieldOverlay();

                    if (m_path.timedOut())
                        throw new InterruptedException("Timed out");

                    localizer.update();
                    Pose2d currentPose = localizer.getPoseEstimate();

                    packet.put("x", currentPose.getX());
                    packet.put("y", currentPose.getY());
                    double speeds[] = m_path.loop(currentPose.getX(), currentPose.getY(), currentPose.getHeading());

                    packet.put("speed 0", speeds[0]);
                    packet.put("speed 1", speeds[1]);
                    packet.put("speed 2", speeds[2]);

                    dashboard.sendTelemetryPacket(packet);

                    // return the motor speeds

                    dt.driveRobotCentric(speeds[0], speeds[1], speeds[2]);

                    DashboardUtil.drawRobot(fieldOverlay, currentPose);
                }

                dt.stop();
            }
        }
    }
}
