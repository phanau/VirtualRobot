package virtual_robot.controller;

import virtual_robot.controller.LinearOpMode;

public class OpMode extends LinearOpMode {

    public OpMode(){}

    // implementations of OpMode override these
    public void init() {}
    public void loop() {}
    public void stop() {}

    public void runOpMode()
    {
        init();
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            loop();
            telemetry.update();
        }
        stop();
    }


}
