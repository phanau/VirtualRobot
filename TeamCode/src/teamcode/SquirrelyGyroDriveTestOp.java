package teamcode;

import java.util.ArrayList;

import virtual_robot.controller.OpMode;
import virtual_robot.controller.XDriveBot;
import virtual_robot.hardware.DcMotor;
import virtual_robot.util._Libs.AutoLib;
import virtual_robot.util._Libs.HeadingSensor;
import virtual_robot.util._Libs.SensorLib;



/**
 * simple example of using a Step that makes a bot with "squirrely wheels" drive along a given course
 * Created by phanau on 10/31/16.
 */


// simple example sequence that tests time based "squirrely wheel" drive steps to drive along a prescribed path
// while stabilizing robot orientation with gyro inputs
//@Autonomous(name="Test: Squirrely Gyro Drive Test 1", group ="Test")
//@Disabled
public class SquirrelyGyroDriveTestOp extends OpMode {

    AutoLib.Sequence mSequence;             // the root of the sequence tree
    boolean bDone;                          // true when the programmed sequence is done
    DcMotor mMotors[];                      // motors in assumed order fr, br, fl, bl
    HeadingSensor mGyro;                    // gyro used to maintain robot orientation while "squirreling" around
    RobotHardware rh;                       // standard hardware set for these tests

    @Override
    public void init() {
        // tell AutoLib about this client OpMode
        AutoLib.mOpMode = this;

        // get hardware
        rh = new RobotHardware();
        rh.init(this);
        mMotors = rh.mMotors;
        mGyro = rh.mIMU;

        // create an autonomous sequence with the steps to drive
        // several legs of a polygonal course ---
        float power = 1.0f;

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

        // add a bunch of timed "legs" to the sequence - use Gyro heading convention of positive degrees CCW from initial heading
        float leg =  3.0f;  // time along each leg of the polygon
        boolean isXdrive = (this.virtualBot.getClass() == XDriveBot.class);  // handle X-drive too ...
        if (isXdrive)
            leg /= Math.sqrt(2);   // each wheel rotation moves the bot further with X-drive

        SensorLib.PID pid = null;         // use default PID provided by the step

        // move to better starting position (assume we start at the center of the field)
        mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(this, 180, 0, mGyro, pid, mMotors, power, leg/2, false));

        // drive a square while maintaining constant orientation (0)
        mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(-90, 0, mGyro, pid, mMotors, power, leg/2, false));
        mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(0, 0, mGyro, pid, mMotors, power, leg, false));
        mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(90, 0, mGyro, pid, mMotors, power, leg, false));
        mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(180, 0, mGyro, pid, mMotors, power, leg, false));
        mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(270, 0, mGyro, pid, mMotors, power, leg/2, false));

        // ... and then a diamond
        mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(-45, 0, mGyro, pid, mMotors, power, leg, false));
        mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(45, 0, mGyro, pid, mMotors, power, leg, false));
        mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(135, 0, mGyro, pid, mMotors, power, leg, false));
        mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(225, 0, mGyro, pid, mMotors, power, leg, false));

        // ... and then sort of a polygonal circle
        int n = 60;     // number of sides
        for (int i=0; i<=n; i++) {
            float heading = 360*i/n - 90;
            boolean stop = (i==n);
            int t = (i==0 || i==n) ? 2 : 4;
            mSequence.add(new AutoLib.SquirrelyGyroTimedDriveStep(heading, 0, mGyro, pid, mMotors, power, t*leg/n, stop));
        }

        // start out not-done
        bDone = false;

        telemetry.addData("Autonomous test of Meccanum/X-drive", "");
        telemetry.addData("using SquirrelyGyroTimedDriveSteps", "");
        telemetry.addData("navigate square/diamond/circle", "");

    }

    @Override
    public void loop() {
        // report elapsed time to test Suspend/Resume
        telemetry.addData("elapsed time", this.getRuntime());

        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
        else
            telemetry.addData("sequence finished", "");
    }

    @Override
    public void stop() {
        super.stop();
    }
}

