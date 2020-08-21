package org.firstinspires.ftc.teamcode.POC;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;

import java.util.ArrayList;
import java.util.List;

@Config
@Disabled
@Autonomous(name="POC: Path Drawing", group="POC")
public class DrawPath extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        telemetry = dashboard.getTelemetry();
        float startX = 0;
        float startY = 0;

        waitForStart();

        if (isStopRequested()) return;

        double movSpeed = .4;
        double turnSpeed = .5;

        // With X and Y coordinates.
        Waypoint p1 = new StartWaypoint(0, 0);
        Waypoint p2 = new GeneralWaypoint(10, startY + 3, movSpeed, turnSpeed, 3);
        Waypoint p3 = new GeneralWaypoint(startX + 20, startY - 3, movSpeed, turnSpeed, 3);
        Waypoint p4 = new GeneralWaypoint(startX + 30, startY + 3, movSpeed, turnSpeed, 3);
        Waypoint p5 = new EndWaypoint(startX + 40, startY, 0, movSpeed, turnSpeed, 3, 3, .5);

        // we are using the waypoints we made in the above examples
        Path m_path = new Path(p1, p2, p3, p4, p5);
        m_path.init(); // initialize the path

        List<Pose2d> poseHistory = new ArrayList<>();

        for(int i=0; i<20; i++)
            poseHistory.add(new Pose2d(i, i, 0));

        while (opModeIsActive()) {
            packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            Pose2d currentPose = new Pose2d(10, 10, 1);
            DashboardUtil.drawRobot(fieldOverlay, currentPose);
            DashboardUtil.drawPath(fieldOverlay, m_path);
            DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);
            dashboard.sendTelemetryPacket(packet);
            sleep(50);
        }
    }
}
