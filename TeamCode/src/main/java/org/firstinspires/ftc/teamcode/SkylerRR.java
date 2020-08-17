package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.SimpleMotionConstraints;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.AssetsTrajectoryManager;
import org.jetbrains.annotations.NotNull;

import java.io.IOException;

@Autonomous(name="1 POC: RR Test", group = "POC")
public class SkylerRR extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
      SkylerDrive drive = new SkylerDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        try {
            // Trajectory trajLoad =

            TrajectoryConfig tc = AssetsTrajectoryManager.loadConfig("skystone");
            //drive.trajectoryBuilder(tc);


/*            Trajectory trajLeft = drive.trajectoryBuilder(new Pose2d())
//                .lineTo(new Vector2d(0, -   20))
//                .lineTo(new Vector2d(0, -   29))
//                .strafeRight(29)
                    .splineTo(new Pose2d(0, -29, 90))
                    .build();*/

      //      drive.followTrajectory(trajLoad);

/*        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(new Pose2d(0, -29, 90))
                .build()
        );*/

/*        Trajectory trajS = drive.trajectoryBuilder()
                .splineTo(new Pose2d(0, -29, 0))
                .build();

        drive.followTrajectory(trajS);*/

            sleep(2000);
        }
        catch (Exception ex) {

        }
    }
}
