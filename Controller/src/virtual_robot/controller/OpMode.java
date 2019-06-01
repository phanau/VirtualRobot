package virtual_robot.controller;

import java.util.concurrent.TimeUnit;

import virtual_robot.controller.LinearOpMode;
import virtual_robot.hardware.HardwareMap;
import virtual_robot.hardware.Telemetry;

public class OpMode extends LinearOpMode {

    // internal time tracking
    private long _startTime = 0;        // in nanoseconds
    private long _suspendedTime = 0;    // in nanoseconds
    private long _prevTick = 0;         // in nanoseconds

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

    public void runOpMode(BooleanObject bSuspend)
    {
        init();
        telemetry.update();
        waitForStart();
        resetStartTime();
        while (opModeIsActive()) {
            long tick = System.nanoTime();
            long deltaT = tick-_prevTick;
            _prevTick = tick;
            if (bSuspend.value) {
                _suspendedTime += deltaT;
            }
            else {
                loop();
                telemetry.update();
            }
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
        double elapsedTime = (System.nanoTime()-_startTime-_suspendedTime) / NANOSECONDS_PER_SECOND;
        return elapsedTime;
    }

    /**
     * Reset the start time to zero.
     */
    public void resetStartTime() {
        _startTime = _prevTick = System.nanoTime();
        _suspendedTime = 0;
    }

}
