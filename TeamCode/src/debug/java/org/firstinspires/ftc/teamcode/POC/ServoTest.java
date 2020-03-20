package org.firstinspires.ftc.teamcode.POC;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This OP Mode tests all servos rotation
 * TODO: rewrite to query servos names from HW Map instead of hardcoding Skyler's configuration
 */

@TeleOp(name="Diag: Servo Spin Test", group ="Util")
public class ServoTest extends LinearOpMode {

    @Override public void runOpMode() {

        while(!opModeIsActive()) {
        }

        //waitForStart();
        while(opModeIsActive()) {

            CRServo crServo;
            String crServos[] = {"dropServo4", "dropServo3", "collectServo4", "collectServo3"};
            for(String servo : crServos) {
                crServo = hardwareMap.get(CRServo.class, servo);

                telemetry.addLine(servo + " set power to .9");
                telemetry.update();
                crServo.setPower(.9);
                sleep(500);

                telemetry.addLine(servo + " set power to -.9");
                telemetry.update();
                crServo.setPower(-.9);
                sleep(500);

                telemetry.addLine(servo + " stop");
                telemetry.update();
                crServo.setPower(0);
                sleep(250);

                crServo.resetDeviceConfigurationForOpMode();
                crServo.close();
            }

            String simpleServos[] = {"twister", "grabber", "hookRight", "hookLeft"};
            Servo simpleServo;
            for(String servo : simpleServos) {
                simpleServo = hardwareMap.get(Servo.class, servo);

                telemetry.addLine(servo + " move to .9");
                telemetry.update();
                simpleServo.setPosition(.9);
                sleep(500);

                telemetry.addLine(servo + " move to .1");
                telemetry.update();
                simpleServo.setPosition(.1);
                sleep(500);

                simpleServo.resetDeviceConfigurationForOpMode();
                simpleServo.close();
            }
        }
    }
}
