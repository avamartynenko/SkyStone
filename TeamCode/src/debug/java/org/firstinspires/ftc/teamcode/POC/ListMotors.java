package org.firstinspires.ftc.teamcode.POC;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This OP Mode tests all motors rotation
 */

@Disabled
@TeleOp(name="Diag: List all motors", group ="Util")
public class ListMotors extends LinearOpMode {

    @Override public void runOpMode() {

        while(!opModeIsActive()) {
        }

        //waitForStart();
        while(opModeIsActive()) {
            for(DcMotor motor : hardwareMap.getAll(DcMotor.class)) {
                telemetry.addLine(motor.getDeviceName() + " spin forward");
                telemetry.update();
                motor.setPower(.3);
                sleep(1000);

                telemetry.addLine(motor.getDeviceName() + " stop");
                telemetry.update();
                motor.setPower(0);
                sleep(250);

                telemetry.addLine(motor.getDeviceName() + " spin backward");
                telemetry.update();
                motor.setPower(-.3);
                sleep(1000);

                telemetry.addLine(motor.getDeviceName() + " stop");
                telemetry.update();
                motor.setPower(0);
                sleep(1000);

                motor.resetDeviceConfigurationForOpMode();
                motor.close();
            }
        }
    }
}
