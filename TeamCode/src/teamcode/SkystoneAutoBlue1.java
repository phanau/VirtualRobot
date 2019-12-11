package teamcode;

import virtual_robot.controller.MechanumBot;
import virtual_robot.controller.OpMode;
import virtual_robot.controller.XDriveBot;
import virtual_robot.hardware.DcMotor;
import virtual_robot.util._Libs.AutoLib;
import virtual_robot.util._Libs.HeadingSensor;
import virtual_robot.util._Libs.SensorLib;
import virtual_robot.util.navigation.DistanceUnit;
import virtual_robot.util.navigation.Position;

import java.util.ArrayList;


/**
 * simple example of using a Step that uses encoder and gyro input to drive to given field positions
 * assuming a "squirrely" drive (Meccanum wheels or X-drive) that can move in any direction independent
 * of the direction the bot is facing.
 * Created by phanau on 5/29/19
 */

// simple example sequence that tests encoder/gyro-based position integration to drive along a given path
//@Autonomous(name="Test: Pos Int Drive Test", group ="Test")
//@Disabled
public class SkystoneAutoBlue1 extends OpMode {

    // return done when we're within tolerance distance of target position
    class PositionTerminatorStep extends AutoLib.MotorGuideStep {

        OpMode mOpMode;
        SensorLib.PositionIntegrator mPosInt;
        Position mTarget;
        double mTol;
        double mPrevDist;
        boolean mStop;
        ArrayList<AutoLib.SetPower> mMotorsteps;

        public PositionTerminatorStep(SensorLib.PositionIntegrator posInt, Position target, double tol,  boolean stop) {
            mOpMode = AutoLib.mOpMode;
            mPosInt = posInt;
            mTarget = target;
            mTol = tol;
            mStop = stop;
            mPrevDist = 1e6;    // infinity
        }

        public void set(ArrayList<AutoLib.SetPower> motorsteps){
            mMotorsteps = motorsteps;
        }

        @Override
        public boolean loop() {
            super.loop();
            Position current = mPosInt.getPosition();
            double dist = Math.sqrt((mTarget.x-current.x)*(mTarget.x-current.x) + (mTarget.y-current.y)*(mTarget.y-current.y));
            if (mOpMode != null) {
                mOpMode.telemetry.addData("PTS target", String.format("%.2f", mTarget.x) + ", " + String.format("%.2f", mTarget.y));
                mOpMode.telemetry.addData("PTS current", String.format("%.2f", current.x) + ", " + String.format("%.2f", current.y));
                mOpMode.telemetry.addData("PTS dist", String.format("%.2f", dist));
            }
            boolean bDone = (dist < mTol);

            // try to deal with "circling the drain" problem -- when we're close to the tolerance
            // circle, but we can't turn sharply enough to get into it, we circle endlessly --
            // if we detect that situation, just give up and move on.
            // simple test: if we're close but the distance to the target increases, we've missed it.
            if (dist < mTol*4 && dist > mPrevDist)
                bDone = true;
            mPrevDist = dist;

            // optionally stop the motors when we're done
            if (bDone && mStop) {
                for (AutoLib.SetPower m : mMotorsteps) {
                    m.setPower(0);
                }
            }

            return bDone;
        }
    }

    // guide step that uses a gyro and a position integrator to determine how to guide the robot to the target
    class SqGyroPosIntGuideStep extends AutoLib.SquirrelyGyroGuideStep {

        OpMode mOpMode;
        Position mTarget;
        SensorLib.EncoderGyroPosInt mPosInt;
        double mTol;
        float mMaxPower;
        float mMinPower = 0.25f;
        float mSgnPower;

        public SqGyroPosIntGuideStep(OpMode opmode, SensorLib.EncoderGyroPosInt posInt, Position target, float heading,
                                     SensorLib.PID pid, ArrayList<AutoLib.SetPower> motorsteps, float power, double tol) {
            // this.preAdd(new SquirrelyGyroGuideStep(mode, direction, heading, gyro, pid, steps, power));
            super(opmode, 0, heading, posInt.getGyro(), pid, motorsteps, power);
            mOpMode = opmode;
            mTarget = target;
            mPosInt = posInt;
            mTol = tol;
            mMaxPower = Math.abs(power);
            mSgnPower = (power > 0) ? 1 : -1;
        }

        public boolean loop() {
            // run the EncoderGyroPosInt to update its position based on encoders and gyro
            mPosInt.loop();

            // update the SquirrelyGyroGuideStep direction to continue heading for the target
            // while maintaining the heading (orientation) given for this Step
            float direction = (float) HeadingToTarget(mTarget, mPosInt.getPosition());
            super.setDirection(direction);

            // when we're close to the target, reduce speed so we don't overshoot
            Position current = mPosInt.getPosition();
            float dist = (float)Math.sqrt((mTarget.x-current.x)*(mTarget.x-current.x) + (mTarget.y-current.y)*(mTarget.y-current.y));
            float brakeDist = (float)mTol * 5;  // empirical ...
            if (dist < brakeDist) {
                float power = mSgnPower * (mMinPower + (mMaxPower-mMinPower)*(dist/brakeDist));
                super.setMaxPower(power);
            }

            // run the underlying GyroGuideStep and return what it returns for "done" -
            // currently, it leaves it up to the terminating step to end the Step
            return super.loop();
        }

        private double HeadingToTarget(Position target, Position current) {
            double headingXrad = Math.atan2((target.y - current.y), (target.x - current.x));        // pos CCW from X-axis
            double headingYdeg = SensorLib.Utils.wrapAngle(Math.toDegrees(headingXrad) - 90.0);     // pos CCW from Y-axis
            if (mOpMode != null) {
                mOpMode.telemetry.addData("GPIGS.HTT target", String.format("%.2f", target.x) + ", " + String.format("%.2f", target.y));
                mOpMode.telemetry.addData("GPIGS.HTT current", String.format("%.2f", current.x) + ", " + String.format("%.2f", current.y));
                mOpMode.telemetry.addData("GPIGS.HTT heading", String.format("%.2f", headingYdeg));
            }
            return headingYdeg;
        }
    }

    // Step: drive to given absolute field position while facing in the given direction using given EncoderGyroPosInt
    class SqPosIntDriveToStep extends AutoLib.GuidedTerminatedDriveStep {

        SensorLib.EncoderGyroPosInt mPosInt;
        Position mTarget;
        AutoLib.GyroGuideStep mGuideStep;
        PositionTerminatorStep mTerminatorStep;

        public SqPosIntDriveToStep(OpMode opmode, SensorLib.EncoderGyroPosInt posInt, DcMotor[] motors,
                                 float power, SensorLib.PID pid, Position target, float heading, double tolerance, boolean stop)
        {
            super(new SqGyroPosIntGuideStep(opmode, posInt, target, heading, pid, null, power, tolerance),
                    new PositionTerminatorStep(posInt, target, tolerance, stop),
                    motors);

            mPosInt = posInt;
            mTarget = target;
        }

    }

    // Step: use Vuforia to locate the Skystone image and set the given position to where it is so we can go there
    class FindSkystoneStep extends AutoLib.LogTimeStep {

        Position mCurrPos;      // current position -- stone is relative to this
        Position mTarget;       // target position we'll update if we see Skystone

        public FindSkystoneStep(OpMode opmode, Position currPos, Position target, float timeout)
        {
            super(opmode, "FindSkystoneStep", timeout);
            mCurrPos = currPos;
            mTarget = target;
        }

        public boolean loop() {

            mTarget.x = mCurrPos.x - 8;  // pretend we found the Stone nearest the audience

            // run the base Step and return what it returns for "done" - i.e. have we timed out?
            // note that we're running this code AFTER trying to determine target with Vuforia data ...
            return super.loop();
        }
    }


    // Step: log the given Position variable to the console so we can see if/when it changes due to Vuforia hit ...
    class LogPosition extends AutoLib.LogTimeStep {

        OpMode mOpMode;
        Position mPosition;
        String mName;

        public LogPosition(OpMode opMode, String name, Position position, double seconds) {
            super(opMode, name, seconds);
            mOpMode = opMode;
            mName = name;
            mPosition = position;
        }

        public boolean loop()
        {
            mOpMode.telemetry.addData(mName, mPosition);
            return super.loop();
        }
    }

    // a simple Step that computes a new Position (c) by adding an offset (b) to Position a.
    // this happens during sequence run-time in loop(), not when the sequence is constructed in init().
    class ComputePositionStep extends AutoLib.Step {

        Position mA, mB, mC;

        public ComputePositionStep(Position a, Position b, Position c)
        {
           mA = a;  mB = b;  mC = c;
        }

        public boolean loop()
        {
            super.loop();
            if (firstLoopCall()) {
                mC.x = mA.x + mB.x;
                mC.y = mA.y + mB.y;
                mC.z = mA.z + mB.z;
            }
            return true;
        }

    }

    AutoLib.Sequence mSequence;             // the root of the sequence tree
    boolean bDone;                          // true when the programmed sequence is done
    boolean bSetup;                         // true when we're in "setup mode" where joysticks tweak parameters
    SensorLib.PID mPid;                     // PID controller for the sequence
    SensorLib.EncoderGyroPosInt mPosInt;    // Encoder/gyro-based position integrator to keep track of where we are
    RobotHardware rh;                       // standard hardware set for these tests

    // some robot dimensions we'll need to e.g. position the front of the bot where we want it relative to
    // the centroid of the bot, which is where we track its position on the field
    final float ROBOT_LENGTH = 18.0f;
    final float MM_TO_INCH = 25.4f;

    @Override
    public void init() {
        // tell AutoLib about this client OpMode
        AutoLib.mOpMode = this;

        // get hardware
        rh = new RobotHardware();
        rh.init(this);

        // post instructions to console
        telemetry.addData("PosIntDriveTestOp", "");
        telemetry.addData("requires Meccanum or X-drive", "");
        telemetry.addData("", "autonomous point to point");
        telemetry.addData("", "navigation using PositionIntegrator");
        telemetry.addData("", "driven by motor encoders");

        // create a PID controller for the sequence
        // parameters of the PID controller for this sequence - assumes 20-gear motors (fast)
        float Kp = 0.02f;        // motor power proportional term correction per degree of deviation
        float Ki = 0.025f;         // ... integrator term
        float Kd = 0;             // ... derivative term
        float KiCutoff = 10.0f;    // maximum angle error for which we update integrator
        mPid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);    // make the object that implements PID control algorithm

        // create Encoder/gyro-based PositionIntegrator to keep track of where we are on the field
        int countsPerRev = 28*40;		// for 40:1 gearbox motor @ 28 counts/motorRev
        double wheelDiam = 4.0;		    // wheel diameter (in)

        // initial position and orientation of bot is along Blue wall near the red loading zone facing the Red side
        rh.mIMU.setHeadingOffset(180);  // initially bot is facing in (Vuforia) field -Y direction, where (for us) +Y is bearing zero
        Position initialPosn = new Position(DistanceUnit.INCH, -36.0, 63.0, 0.0, 0);
        SensorLib.EncoderGyroPosInt.DriveType dt =
                (this.virtualBot.getClass() == XDriveBot.class) ? SensorLib.EncoderGyroPosInt.DriveType.XDRIVE :
                        (this.virtualBot.getClass() == MechanumBot.class) ? SensorLib.EncoderGyroPosInt.DriveType.MECANUM :
                                SensorLib.EncoderGyroPosInt.DriveType.NORMAL;  // handle X-drive too ...
        mPosInt = new SensorLib.EncoderGyroPosInt(dt,this, rh.mIMU, rh.mMotors, countsPerRev, wheelDiam, initialPosn);

        // create an autonomous sequence with the steps to drive
        // several legs of a polygonal course ---
        float movePower = 1.0f;
        float turnPower = 1.0f;

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();
        float tol = 1.0f;   // tolerance in inches

        // add a bunch of position integrator "legs" to the sequence -- uses absolute field coordinate system
        // corresponding to Vuforia convention of +X to the rear and +Y to the Blue side
        // Vuforia convention is bearing zero = +X while our code uses bearing zero = +Y, so there's an offset
        final float boff = -90;  // bearing offset of our convention (+Y to rear of field) vs. Vuforia's (+Y to Blue side)
        // coordinates and bearings below are in Vuforia terms to be compatible with Vuforia position updates if we use them.

        // get closer to the Skystones so Vuforia can see the Skystone image better
        Position lookLoc = new Position(DistanceUnit.INCH, -36, 40+ROBOT_LENGTH/2, 0., 0);
        mSequence.add(new SqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid, lookLoc, -90+boff, tol, true));

        // look for the Skystone with the image and update the target Position for the next move to go to it
        // default to the middle Stone if we don't see one ...
        Position skyLoc = new Position(DistanceUnit.INCH, -36, 24+ROBOT_LENGTH/2, 0., 0);
        AutoLib.ConcurrentSequence cs1 = new AutoLib.ConcurrentSequence();
        mSequence.add(cs1);

        cs1.add(new FindSkystoneStep(this, lookLoc, skyLoc, 3.0f));         // look for SkyStone ...
        cs1.add(new LogPosition(this, "skyLoc", skyLoc,0.0f));       // ... and report target position while searching

        // drive to the SkyStone if we found it, otherwise to the default (middle) stone.
        mSequence.add(new SqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid, skyLoc, -90 + boff, tol, true));

        // grab the Skystone
        // mSequence.add(new AutoLib.ServoStep());
        mSequence.add(new AutoLib.LogTimeStep(this, "grab Stone", 2));

        // back up a bit to pull the stone out of the line of stones
        Position pullLoc = new Position(DistanceUnit.INCH, 0, 0, 0., 0);
        mSequence.add(new ComputePositionStep(skyLoc, new Position(DistanceUnit.INCH, 0, 12, 0, 0), pullLoc));
        mSequence.add(new SqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid, pullLoc, -90 + boff, tol, false));

        // go to the Blue Foundation
        mSequence.add(new SqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 48, 32, 0., 0), -90+boff, tol, true));

        // do the next two Steps in parallel
        AutoLib.ConcurrentSequence cs2 = new AutoLib.ConcurrentSequence();
        mSequence.add(cs2);

        // drop the Skystone
        // cs2.add(new AutoLib.ServoStep());
        cs2.add(new AutoLib.LogTimeStep(this, "drop Stone", 2));

        // grab the Foundation
        // cs2.add(new AutoLib.ServoStep());
        cs2.add(new AutoLib.LogTimeStep(this, "grab Foundation", 2));

        // drag the Foundation to the Building Area
        mSequence.add(new SqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 48, 62, 0., 0), -90+boff, tol, true));

        // release the Foundation
        // mSequence.add(new AutoLib.ServoStep());
        mSequence.add(new AutoLib.LogTimeStep(this, "release Foundation", 2));

        // slide out of the corridor left by positioning the Foundation
        mSequence.add(new SqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 62, 0., 0), -90+boff, tol, false));
        mSequence.add(new SqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 48, 0., 0), -90+boff, tol, false));

        // return to the quarry for a second SkyStone
        mSequence.add(new SqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, -60, 32, 0., 0), -90+boff, tol, true));

        // grab the Skystone
        // mSequence.add(new AutoLib.ServoStep());
        mSequence.add(new AutoLib.LogTimeStep(this, "grab Stone", 2));

        // back up a bit to pull the stone out of the line of stones
        mSequence.add(new SqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, -60, 40, 0., 0), -90+boff, tol, false));

        // bring it to the audience end of the Foundation via the Blue Skybridge
        mSequence.add(new SqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 48, 0., 0), -90+boff, tol, false));
        mSequence.add(new SqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 24, 60, 0., 0), 0+boff, tol, true));

        // drop the Skystone
        // cs2.add(new AutoLib.ServoStep());
        mSequence.add(new AutoLib.LogTimeStep(this, "drop Stone", 2));

        // park under the SkyBridge
        mSequence.add(new SqPosIntDriveToStep(this, mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 48, 0., 0), 0+boff, tol, true));

        // start out not-done
        bDone = false;
    }

    public void loop() {
        // report elapsed time to test Suspend/Resume
        telemetry.addData("elapsed time", this.getRuntime());

        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
        else
            telemetry.addData("sequence finished", "");
    }

    public void stop() {
        super.stop();
    }
}

