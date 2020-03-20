package org.firstinspires.ftc.teamcode.POC;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This OP Mode tests all motors rotation
 * TODO: rewrite to query motors names from HW Map instead of hardcoding Skyler's configuration
 */

@TeleOp(name="Diag: Motor Spin Test", group ="Util")
public class MotorTest extends LinearOpMode {

    @Override public void runOpMode() {

        while(!opModeIsActive()) {
        }

        //waitForStart();
        while(opModeIsActive()) {

            DcMotor testMotor;
            String freeSpinMotors[] = {"backLeft", "backRight", "frontLeft", "frontRight", "intakeLeft", "intakeRight"};
            for(String motor : freeSpinMotors) {
                testMotor = hardwareMap.get(DcMotor.class, motor);
                telemetry.addLine(motor + " spin forward");
                telemetry.update();
                testMotor.setPower(1);
                sleep(1000);

                telemetry.addLine(motor + " stop");
                telemetry.update();
                testMotor.setPower(0);
                sleep(250);

                telemetry.addLine(motor + " spin backward");
                telemetry.update();
                testMotor.setPower(-1);
                sleep(1000);

                telemetry.addLine(motor + " stop");
                telemetry.update();
                testMotor.setPower(0);
                sleep(1000);
            }

            String limitedRangeMotors[] = {"liftElevator", "frogTongue"};

            for(String motor : limitedRangeMotors) {
                testMotor = hardwareMap.get(DcMotor.class, motor);
                telemetry.addLine(motor + " spin forward");
                telemetry.update();
                testMotor.setPower(.3);
                sleep(500);

                telemetry.addLine(motor + " stop");
                telemetry.update();
                testMotor.setPower(0);
                sleep(250);

                telemetry.addLine(motor + " spin backward");
                telemetry.update();
                testMotor.setPower(-.3);
                sleep(500);

                telemetry.addLine(motor + " stop");
                telemetry.update();
                testMotor.setPower(0);
                sleep(250);
            }
        }
    }

}
