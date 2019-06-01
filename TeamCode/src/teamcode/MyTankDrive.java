package teamcode;

import virtual_robot.controller.OpMode;
import virtual_robot.hardware.ColorSensor;
import virtual_robot.hardware.DcMotor;
import virtual_robot.hardware.DistanceSensor;
import virtual_robot.hardware.GyroSensor;
import virtual_robot.hardware.Servo;
import virtual_robot.hardware.bno055.BNO055IMU;
import virtual_robot.util.navigation.AngleUnit;
import virtual_robot.util.navigation.AxesOrder;
import virtual_robot.util.navigation.AxesReference;
import virtual_robot.util.navigation.DistanceUnit;
import virtual_robot.util.navigation.Orientation;

public class MyTankDrive extends OpMode {

    DcMotor m1, m2, m3,m4;
    BNO055IMU imu;
    Servo backServo;
    DistanceSensor frontDistance, leftDistance, rightDistance, backDistance;
    ColorSensor colorSensor;

    public void init() {
        m1 = hardwareMap.dcMotor.get("back_left_motor");
        m2 = hardwareMap.dcMotor.get("front_left_motor");
        m3 = hardwareMap.dcMotor.get("front_right_motor");
        m4 = hardwareMap.dcMotor.get("back_right_motor");
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        backServo = hardwareMap.servo.get("back_servo");

        frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";
        imu.initialize(parameters);

        colorSensor = hardwareMap.colorSensor.get("color_sensor");

        telemetry.addData("Tank drive - two sticks Y", "");
        telemetry.addData("plus left/right slide on X", "");
        telemetry.addData("Press Start When Ready", "");
    }

    public void loop()
    {
        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.
        float left = -gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        left =  (float)scaleInput(left);
        right = (float)scaleInput(right);

        // squirrely drive is controlled by the x axis of both sticks
        float xLeft = gamepad1.left_stick_x;
        float xRight = gamepad1.right_stick_x;

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        xLeft =  (float)scaleInput(xLeft);
        xRight = (float)scaleInput(xRight);

        // combine turning and squirrely drive inputs and
        double fr = clip(right+xRight, -1, 1);
        double br = clip(right-xRight, -1, 1);
        double fl = clip(left+xLeft, -1, 1);
        double bl = clip(left-xLeft, -1, 1);

        // write the values to the motors
        m1.setPower(bl);
        m2.setPower(fl);
        m3.setPower(fr);
        m4.setPower(br);

        telemetry.addData("Squirrely Tank Drive", "*** v2.1 ***");
        telemetry.addData("front left/right power:", "%.2f %.2f", fl, fr);
        telemetry.addData("back left/right power:", "%.2f %.2f", bl, br);

        telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
        //telemetry.addData("Heading"," %.1f", gyro.getHeading());
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        telemetry.addData("Heading", " %.1f", orientation.firstAngle * 180.0 / Math.PI);

        telemetry.addData("Front Distance", " %.1f", frontDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Left Distance", " %.1f", leftDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Right Distance", " %.1f", rightDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Back Distance", " %.1f", backDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(),
                m3.getCurrentPosition(), m4.getCurrentPosition());

    }

    public void stop()
    {
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds. Raising to an odd power (3)
     * preserves the sign of the input.
     */
    double scaleInput(double dVal)  {
        return dVal*dVal*dVal;		// maps {-1,1} -> {-1,1}
    }

    // clip value to range
    double clip(float v, float a, float b)
    {
        if (v < a)
            v = a;
        if (v > b)
            v = b;
        return v;
    }
}
