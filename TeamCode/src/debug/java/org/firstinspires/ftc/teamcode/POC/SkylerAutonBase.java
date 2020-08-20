package org.firstinspires.ftc.teamcode.POC;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.SimpleMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.T265Wrapper;

@Config
@Disabled
@Autonomous(name="POC: Drive to XY test - Spin Motors", group="Util")
public class SkylerAutonBase extends LinearOpMode {
    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        telemetry = dashboard.getTelemetry();
        T265Wrapper localizer = new T265Wrapper();


        SimpleMotorEx fL = new SimpleMotorEx("frontLeft", hardwareMap, 435);
        SimpleMotorEx fR = new SimpleMotorEx("frontRight", hardwareMap, 435);
        SimpleMotorEx bL = new SimpleMotorEx("backLeft", hardwareMap, 435);
        SimpleMotorEx bR = new SimpleMotorEx("backRight", hardwareMap, 435);

        bL.setInverted(true);
        fL.setInverted(true);

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

            bR.set(.0);
            fL.set(.25);
            sleep(500);
            fL.set(.0);
            fR.set(.25);
            sleep(500);
            fR.set(.0);
            bL.set(.25);
            sleep(500);
            bL.set(.0);
            bR.set(.25);
            sleep(500);

/*            fL.set(-.25);
            fR.set(-.25);
            bL.set(-.25);
            bR.set(-.25);

            sleep(500);*/

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

        }
    }
}
