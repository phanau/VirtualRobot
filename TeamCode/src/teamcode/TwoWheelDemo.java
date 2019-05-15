package teamcode;

import virtual_robot.hardware.*;
import virtual_robot.controller.LinearOpMode;
import virtual_robot.util.navigation.DistanceUnit;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
public class TwoWheelDemo extends LinearOpMode {

    public void runOpMode(){
        DcMotor left = hardwareMap.dcMotor.get("left_motor");
        DcMotor right = hardwareMap.dcMotor.get("right_motor");
        left.setDirection(DcMotor.Direction.REVERSE);
        GyroSensor gyro = hardwareMap.gyroSensor.get("gyro_sensor");
        Servo backServo = hardwareMap.servo.get("back_servo");
        gyro.init();
        ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");
        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        DistanceSensor backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");
        DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");

        telemetry.addData("Press Start to Continue","");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a){
                telemetry.addData("a pressed","");
                left.setPower(-.5);
                right.setPower(-.5);
            } else if (gamepad1.y) {
                telemetry.addData("y pressed", "");
                left.setPower(0.5);
                right.setPower(0.5);
            } else if (gamepad1.b){
                telemetry.addData("b pressed", "");
                left.setPower(0.5);
                right.setPower(-0.5);
            } else if (gamepad1.x){
                telemetry.addData("x pressed", "");
                left.setPower(-0.5);
                right.setPower(0.5);
            }
            else {
                left.setPower(0);
                right.setPower(0);
            }
            backServo.setPosition(0.5 - 0.5* gamepad1.left_stick_y);
            telemetry.addData("Press", "Y-fwd, A-rev, B-Rt, X-Lt");
            telemetry.addData("Left Gamepad stick controls back servo","");
            telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
            telemetry.addData("Heading"," %.1f", gyro.getHeading());
            telemetry.addData("Encoders","Left %d  Right %d", left.getCurrentPosition(), right.getCurrentPosition());
            telemetry.addData("Distance", " Fr %.1f  Lt %.1f  Rt %.1f  Bk %.1f  ",
                    frontDistance.getDistance(DistanceUnit.CM), leftDistance.getDistance(DistanceUnit.CM),
                    rightDistance.getDistance(DistanceUnit.CM), backDistance.getDistance(DistanceUnit.CM)
                    );
            telemetry.update();
        }
        left.setPower(0);
        right.setPower(0);
    }
}
