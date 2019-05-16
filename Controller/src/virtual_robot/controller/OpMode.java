package virtual_robot.controller;

import java.util.concurrent.TimeUnit;

import virtual_robot.controller.LinearOpMode;
import virtual_robot.hardware.HardwareMap;
import virtual_robot.hardware.Telemetry;

public class OpMode extends LinearOpMode {

    // internal time tracking
    private long startTime = 0; // in nanoseconds

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

    /**
     * Get the number of seconds this op mode has been running
     * <p>
     * This method has sub millisecond accuracy.
     * @return number of seconds this op mode has been running
     */
    public double getRuntime() {
        final double NANOSECONDS_PER_SECOND = TimeUnit.SECONDS.toNanos(1);
        return (System.nanoTime() - startTime) / NANOSECONDS_PER_SECOND;
    }

    /**
     * Reset the start time to zero.
     */
    public void resetStartTime() {
        startTime = System.nanoTime();
    }

}
