package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Util teleop mode to identify and fine tune proper angles for arms movements
 */

@Config
@TeleOp(name="Util: Arms Positions", group ="Util")
public class ArmsPositions extends LinearOpMode {
    public static double collectServo3Position = .4;
    public static double collectServo4Position = .4;
    public static double dropServo3Position = .4;
    public static double dropServo4Position = .4;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        waitForStart();

        if (isStopRequested()) return;

        // front 3, back 4
        String simpleServos[] = { "dropServo3", "dropServo4", "collectServo4", "collectServo3"};
        Servo dropServo3 = hardwareMap.get(Servo.class, "dropServo3");
        Servo dropServo4 = hardwareMap.get(Servo.class, "dropServo4");
        Servo collectServo3 = hardwareMap.get(Servo.class, "collectServo3");
        Servo collectServo4 = hardwareMap.get(Servo.class, "collectServo4");

        while (opModeIsActive()) {
            dropServo3.setPosition(dropServo3Position);
            dropServo4.setPosition(dropServo4Position);
            collectServo3.setPosition(collectServo3Position);
            collectServo4.setPosition(collectServo4Position);

            /*telemetry.addData("Battery", "%.2f", (float) getBatteryVoltage());
            telemetry.update();*/
        }
    }

    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}
