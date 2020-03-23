package org.firstinspires.ftc.teamcode.util.localizer;

import android.graphics.PointF;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Util.T265Wrapper;

/**
 * Use only with robots which can rotate around its center (not skyler)
 * LinearOpMode to measure distance from rotot center to camera reference module
 * This value is used in later steps to calculate cameras offset*
 */

@Config
@Disabled
@TeleOp(name="Util: Localizer - Step 1 - Radius", group ="Util")
public class LocalizerS1Radius extends LinearOpMode {
    // time to sleep between measurements
    public static int measureInterval = 500;
    public static double rotationPower = .2;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        DcMotor fL = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor fR = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor bL = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor bR = hardwareMap.get(DcMotor.class, "backRight");

        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        T265Wrapper localizer = new T265Wrapper();
        float location[];

        while(!opModeIsActive()) {
            localizer.refreshPoseData();

            telemetry.addData("X", "%.01f", (float) localizer.getX());
            telemetry.addData("Y", "%.01f", (float) localizer.getY());

            telemetry.addLine("Move robot around with your hands to reach High tracker confidence");
            telemetry.addData("Tracker Confidence", localizer.getTrackerConfidenceText());
            telemetry.update();
        }

        if(isStopRequested())
            return;

        PointF points[] = new PointF[3];

        while(opModeIsActive()) {

            fL.setPower(rotationPower);
            fR.setPower(rotationPower);
            bL.setPower(rotationPower);
            bR.setPower(rotationPower);

            sleep(measureInterval);
            localizer.refreshPoseData();
            telemetry.addData("X", "%.01f", (float) localizer.getX());
            telemetry.addData("Y", "%.01f", (float) localizer.getY());

            points[0] = points[1];
            points[1] = points[2];
            points[2] = new PointF(localizer.getX(), localizer.getY());

            float radius = 0;

            try {
                radius = calculateRadius(points);
            }
            catch (Exception ex) {
                telemetry.addLine("Unable to calculate radius");
            };

            telemetry.addData("Radius", "%.02f", radius);
            telemetry.addData("TrackerConfidence", "%.0f", (float) localizer.getTrackerConfidence());
            telemetry.update();
        }
    }

    // https://www.geeksforgeeks.org/equation-of-circle-when-three-points-on-the-circle-are-given/
    static float calculateRadius(PointF[] points) {
        float   x1 = points[0].x, x2 = points[1].x, x3 = points[2].x,
                y1 = points[0].y, y2 = points[1].y, y3 = points[2].y;

        float x12 = x1 - x2;
        float x13 = x1 - x3;

        float y12 = y1 - y2;
        float y13 = y1 - y3;

        float y31 = y3 - y1;
        float y21 = y2 - y1;

        float x31 = x3 - x1;
        float x21 = x2 - x1;

        // x1^2 - x3^2
        float sx13 = (float) (Math.pow(x1, 2) - Math.pow(x3, 2));

        // y1^2 - y3^2
        float sy13 = (float) (Math.pow(y1, 2) - Math.pow(y3, 2));

        float sx21 = (float) (Math.pow(x2, 2) - Math.pow(x1, 2));

        float sy21 = (int)(Math.pow(y2, 2) - Math.pow(y1, 2));

        float f = ((sx13) * (x12)
                    + (sy13) * (x12)
                    + (sx21) * (x13)
                    + (sy21) * (x13))
                    / (2 * ((y31) * (x12) - (y21) * (x13)));

        float g = ((sx13) * (y12)
                    + (sy13) * (y12)
                    + (sx21) * (y13)
                    + (sy21) * (y13))
                    / (2 * ((x31) * (y12) - (x21) * (y13)));

        float c = -(int)Math.pow(x1, 2) - (int)Math.pow(y1, 2) -
                    2 * g * x1 - 2 * f * y1;

        // eqn of circle be x^2 + y^2 + 2*g*x + 2*f*y + c = 0
        // where centre is (h = -g, k = -f) and radius r
        // as r^2 = h^2 + k^2 - c
        float h = -g;
        float k = -f;
        float sqr_of_r = h * h + k * k - c;

        // r is the radius
        float r = (float) Math.sqrt(sqr_of_r);

            // DecimalFormat df = new DecimalFormat("#.#####");
            // System.out.println("Centre = (" + h + "," + k + ")");
            // System.out.println("Radius = " + df.format(r));
        return r;
    }
}
