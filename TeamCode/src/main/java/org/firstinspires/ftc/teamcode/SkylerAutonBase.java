package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorImplEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.T265Wrapper;

@Config
@Autonomous(name="POC: Drive to XY test", group="Util")
public class SkylerAutonBase extends LinearOpMode {
    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        telemetry = dashboard.getTelemetry();
        T265Wrapper localizer = new T265Wrapper();

        MotorImplEx fL = new MotorImplEx(hardwareMap, "frontLeft", 435);
        MotorImplEx fR = new MotorImplEx(hardwareMap,"frontRight", 435);
        MotorImplEx bL = new MotorImplEx(hardwareMap,"backLeft", 435);
        MotorImplEx bR = new MotorImplEx(hardwareMap,"backRight", 435);

        bL.setInverted(true);
        bR.setInverted(true);

        MecanumDrive dt = new MecanumDrive(false, fL, fR, bL, bR);

        float location[] = localizer.refreshPoseData();
        float startX = localizer.getCameraX();
        float startY = localizer.getCameraY();

        packet.put("x", localizer.getCameraX() - startX);
        packet.put("y", localizer.getCameraY() - startY);
        dashboard.sendTelemetryPacket(packet);

        waitForStart();

        if (isStopRequested()) return;
        ElapsedTime et = new ElapsedTime();

        while (opModeIsActive()) {
            while (/*et.milliseconds() < 1000*/ location[0] - startX < 50) {
                location = localizer.refreshPoseData();

                packet.put("x", location[0] - startX);
                packet.put("y", location[1] - startY);
                dashboard.sendTelemetryPacket(packet);
                dt.driveRobotCentric(0, 1, 0);
            }

            et.reset();
            while (/*et.milliseconds() < 1000*/ location[0] - startX > 0) {
                location = localizer.refreshPoseData();
                telemetry.addData("x", location[0] - startX);
                telemetry.addData("y", location[1] - startY);
                telemetry.update();
                dt.driveRobotCentric(0, -1, 0);
            }

            while (/*et.milliseconds() < 1000*/ location[0] - startX < 0) {
                location = localizer.refreshPoseData();
                telemetry.addData("x", location[0] - startX);
                telemetry.addData("y", location[1] - startY);
                telemetry.update();
                dt.driveRobotCentric(0, 1, 0);
            }

            dt.driveRobotCentric(0, 0, 0);

            location = localizer.refreshPoseData();
            telemetry.addData("x", location[0] - startX);
            telemetry.addData("y", location[1] - startY);
            telemetry.update();

            sleep(5000);

            /*dt.driveRobotCentric(0, 0, 0);
            sleep(100);

            dt.driveRobotCentric(0, -1, 0);
            sleep(1000);
            dt.driveRobotCentric(0, 0, 0);
            sleep(100);

            dt.driveRobotCentric(.3, 0, 0);
            sleep(500);
            dt.driveRobotCentric(0, 0, 0);
            sleep(100);

            dt.driveRobotCentric(-.3, 0, 0);
            sleep(500);
            dt.driveRobotCentric(0, 0, 0);
            sleep(100);*/

            return;
        }
    }
}
