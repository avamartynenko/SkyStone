package org.firstinspires.ftc.teamcode.roadrunner.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/*
 * Constants shared between multiple drive types.
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 * fields may also be edited through the dashboard (connect to the robot's WiFi network and
 * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */
@Config
public class DriveConstants {
    /*
     * These are motor constants that should be listed online for your motors.
     */
    public static final double TICKS_PER_REV = 383.6;
    public static final double MAX_RPM = 435;

    /*
     * Set the first flag appropriately. If using the built-in motor velocity PID, update
     * MOTOR_VELO_PID with the tuned coefficients from DriveVelocityPIDTuner.
     */
    public static final boolean RUN_USING_ENCODER = true;
    public static final PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(20 , 5, 1);

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public static double WHEEL_RADIUS = 1.9685;
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    // 7.17 (SE = 2.276) @13.8
    // Effective track width = 5.02 (SE = 0.425)
    // Effective track width = 4.17 (SE = 0.522) @13.76
    // Effective track width = 4.18 (SE = 0.522) @13.66
    // Effective track width = 3.40 (SE = 0.352)
    // Effective track width = 4.59 (SE = 0.371) @ 12.68
    // Effective track width = 5.23 (SE = 1.713) @ 12.83
    // Effective track width = 6.24 (SE = 1.350)
    // Effective track width = 3.82 (SE = 0.438)
    public static double TRACK_WIDTH = 17; //  (SE = 0.318)

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     *

@13.82 kV = 0.01206, kStatic = 0.10769 (R^2 = 0.95), kA = 0.00005 (R^2 = 0.90)
@13.98 kV = 0.01168, kStatic = 0.11582 (R^2 = 0.95), kA = 0.00004 (R^2 = 0.79)
@14.12 kV = 0.01219, kStatic = 0.09977 (R^2 = 0.95), kA = 0.00004 (R^2 = 0.89)
@14.16 kV = 0.01129, kStatic = 0.12059 (R^2 = 0.93), kA = 0.00004 (R^2 = 0.89)
@14.16 kV = 0.01189, kStatic = 0.10821 (R^2 = 0.95), kA = 0.00007 (R^2 = 0.63)
@14.35 kV = 0.01216, kStatic = 0.10086 (R^2 = 0.95), kA = 0.00005 (R^2 = 0.87)

     */
    // tested at 13.4
/*    public static double kV = 0.00550;
    public static double kA = 0.000045;
    public static double kStatic = 0.24; // @ 14.1*/

    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. The velocity and
     * acceleration values are required, and the jerk values are optional (setting a jerk of 0.0
     * forces acceleration-limited profiling).
     *
     * SM: we wil run robot at 100% of actual capabilities
     * vaxVel = maxAccel = MAX_RPM * WHELL_RADIUS * Diameter / 60 (assuming in/s and in/s^2)
     * TODO: find falues for maxJerk and maxAnglJerk
     */
    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            45.0, 45.0, 2.5,
            Math.toRadians(180.0), Math.toRadians(180.0), 2.0
    );


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF() {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 * 60.0 / (MAX_RPM * TICKS_PER_REV);
    }
}
