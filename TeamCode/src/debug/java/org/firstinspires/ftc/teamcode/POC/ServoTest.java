package org.firstinspires.ftc.teamcode.POC;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This OP Mode tests all servos rotation
 * TODO: rewrite to query servos names from HW Map instead of hardcoding Skyler's configuration
 */

@Disabled
@TeleOp(name="Diag: Servo Spin Test", group ="Util")
public class ServoTest extends LinearOpMode {

    @Override public void runOpMode() {
        waitForStart();

        if(isStopRequested())
            return;

        while(opModeIsActive()) {

            for(CRServo crServo : hardwareMap.getAll(CRServo.class)) {
                telemetry.addLine(crServo.getDeviceName() + " set power to .75");
                telemetry.update();
                crServo.setPower(.75);
                sleep(3000);

                telemetry.addLine(crServo.getDeviceName() + " set power to 0");
                telemetry.update();
                crServo.setPower(0);
                sleep(3000);

                telemetry.addLine(crServo.getDeviceName() + " set power to -.75");
                telemetry.update();
                crServo.setPower(-.75);
                sleep(3000);

                crServo.resetDeviceConfigurationForOpMode();
            }

            String simpleServos[] = { "dropServo3", "dropServo4", "collectServo4", "collectServo3"};// {/*"twister", "grabber",*/  "hookLeft", "hookRight"};
            Servo simpleServo;
            if(true) for(String servo : simpleServos) {
                simpleServo = hardwareMap.get(Servo.class, servo);

                telemetry.addLine(servo + " move to .6");
                telemetry.update();
                simpleServo.setPosition(.6);
                sleep(3000);

                telemetry.addLine(servo + " move to .5");
                telemetry.update();
                simpleServo.setPosition(.5);
                sleep(3000);

                telemetry.addLine(" Wait for next...");
                telemetry.update();

                sleep(5000);

                simpleServo.resetDeviceConfigurationForOpMode();
//                simpleServo.close();
            }
        }
    }
}
