package teamcode;

import virtual_robot.controller.LinearOpMode;
import virtual_robot.hardware.*;
import virtual_robot.hardware.bno055.BNO055IMU;
import virtual_robot.util.color.Color;
import virtual_robot.util.navigation.*;
import virtual_robot.util.time.ElapsedTime;
import virtual_robot.util.navigation.AngleUtils;

/**
 * Example Autonomous Opmode
 *
 * Uses Line-following two drive around the tape at the perimeter of the lander.
 *
 * Requires mechanum bot configuration.
 *
 * Start with bot in center of lander, facing top of screen.
 *
 */
public class MechBotAutoDemo extends LinearOpMode {

    DcMotor m1, m2, m3, m4;
    //GyroSensor gyro;
    BNO055IMU imu;
    ColorSensor colorSensor;
    Servo backServo;

    public void runOpMode(){

        m1 = hardwareMap.dcMotor.get("back_left_motor");
        m2 = hardwareMap.dcMotor.get("front_left_motor");
        m3 = hardwareMap.dcMotor.get("front_right_motor");
        m4 = hardwareMap.dcMotor.get("back_right_motor");
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());

        colorSensor = hardwareMap.colorSensor.get("color_sensor");
        backServo = hardwareMap.servo.get("back_servo");

        Orientation orientation;

        ElapsedTime waitTime = new ElapsedTime();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Seconds since init","%d. Press start when ready.", (int)waitTime.seconds());
            telemetry.update();
        }

        //Turn 45 degrees.
        setPower(0, 0, 0.5f);
        //while (opModeIsActive() && gyro.getHeading() < 45) continue;
        while (opModeIsActive()){
            orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if (orientation.firstAngle >= 45) break;
        }
        setPower(0, 0, 0);

        float[] hsv = new float[3];

        //Drive forward until color sensor touches tape.
        setPower(0,0.5f,0);
        while (opModeIsActive()) {
            Color.RGBtoHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsv);
            if (hsv[1] > 0.75) break;
        }

        //Keep driving forward until color sensor is off of tape.
        while (opModeIsActive()) {
            Color.RGBtoHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsv);
            if (hsv[1] < 0.25) break;
        }

        float currentHeading = 45;
        ElapsedTime et = new ElapsedTime();

        //Main loop; once per trip around the lander
        while (opModeIsActive()){

            //Turn 90 degrees.
            setPower(0,0,0.5f);
            backServo.setPosition(0.333 * (currentHeading - 45.0f)/90.0f);
            telemetry.addData("Turning 90 degrees","");
            telemetry.update();

            //while (opModeIsActive() && AngleUtils.normalizeDegrees(gyro.getHeading() - currentHeading) < 90) continue;

            while (opModeIsActive()){
                orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if (AngleUtils.normalizeDegrees(orientation.firstAngle - currentHeading) >= 90) break;
            }

            currentHeading = (float)AngleUtils.normalizeDegrees360(currentHeading + 90);

            //Follow outside edge of tape until color sensor has not seen tape for 0.5 seconds.
            telemetry.addData("Line Following","");
            telemetry.update();
            et.reset();
            while (opModeIsActive() && et.seconds() < 0.5) {
                Color.RGBtoHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsv);
                if (hsv[1] > 0.25) et.reset();
                setPower( 2.0f * (hsv[1] - 0.5f), 0.5f, 0);
                String s1 = String.format("%d", colorSensor.red())+", "+String.format("%d", colorSensor.green())+", "+String.format("%d", colorSensor.blue());
                telemetry.addData("Color sensor RGB:", s1);
                String s2 = String.format("%.2f", hsv[0])+", "+String.format("%.2f", hsv[1])+", "+String.format("%.2f", hsv[2]);
                telemetry.addData("Color sensor HSV:", s2);
                telemetry.update();
            }

        }

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }

    void setPower(float px, float py, float pa){
        double p1 = -px + py - pa;
        double p2 = px + py + -pa;
        double p3 = -px + py + pa;
        double p4 = px + py + pa;
        double max = Math.max(1.0, Math.abs(p1));
        max = Math.max(max, Math.abs(p2));
        max = Math.max(max, Math.abs(p3));
        max = Math.max(max, Math.abs(p4));
        p1 /= max;
        p2 /= max;
        p3 /= max;
        p4 /= max;
        m1.setPower(p1);
        m2.setPower(p2);
        m3.setPower(p3);
        m4.setPower(p4);
    }
}
