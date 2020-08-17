package org.firstinspires.ftc.teamcode.POC;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@TeleOp(name="Diag: Back and Forth Test", group ="Util")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getLocalizer().getPoseEstimate(), true)
                        .splineTo(new Vector2d(-5, 0), Math.toRadians(180))
                        .build()
        );

        for(int i=0; i<10; i++) {
/*
            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getLocalizer().getPoseEstimate(), true)
                            .splineTo(new Vector2d(60, 3), Math.toRadians(180))
                            .build()
            );

            sleep(1000);

            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getLocalizer().getPoseEstimate(), true)
                            .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                            .build()
            );

            sleep(1000);*/
        }

    }
}
