package teamcode;

import virtual_robot.controller.OpMode;
import virtual_robot.controller.XDriveBot;
import virtual_robot.hardware.DcMotor;
import virtual_robot.util.navigation.DistanceUnit;
import virtual_robot.util.navigation.Position;

import virtual_robot.util._Libs.AutoLib;
import virtual_robot.util._Libs.HeadingSensor;
import virtual_robot.util._Libs.SensorLib;

import java.util.ArrayList;


/**
 * simple example of using a Step that uses encoder and gyro input to drive to given field positions.
 * Created by phanau on 12/15/18
 */

// simple example sequence that tests encoder/gyro-based position integration to drive along a given path
//@Autonomous(name="Test: Pos Int Drive Test", group ="Test")
//@Disabled
public class PosIntDriveTestOp extends OpMode {

    // use motor encoders and gyro to track absolute field position
    class EncoderGyroPosInt extends SensorLib.PositionIntegrator {
        OpMode mOpMode;
        HeadingSensor mGyro;
        DcMotor[] mEncoderMotor;

        int mEncoderPrev[];		// previous reading of motor encoder
        boolean mFirstLoop;

        int mCountsPerRev;
        double mWheelDiam;

        public EncoderGyroPosInt(OpMode opmode, HeadingSensor gyro, DcMotor[] encoderMotor, int countsPerRev, double wheelDiam, Position initialPosn)
        {
            super(initialPosn);
            mOpMode = opmode;
            mGyro = gyro;
            mEncoderMotor = encoderMotor;
            mFirstLoop = true;
            mCountsPerRev = countsPerRev;
            mWheelDiam = wheelDiam;
            mEncoderPrev = new int[encoderMotor.length];
        }

        public boolean loop() {
            // get initial encoder values
            if (mFirstLoop) {
                for (int i=0; i<mEncoderMotor.length; i++)
                    mEncoderPrev[i] = mEncoderMotor[i].getCurrentPosition();
                mFirstLoop = false;
            }

            // get current encoder values and compute average delta since last read
            int encoderDist = 0;
            for (int i=0; i<mEncoderMotor.length; i++) {
                int encoder = mEncoderMotor[i].getCurrentPosition();
                encoderDist += encoder - mEncoderPrev[i];
                mEncoderPrev[i] = mEncoderMotor[i].getCurrentPosition();
            }
            encoderDist /= mEncoderMotor.length;

            // get bearing from IMU gyro
            double imuBearingDeg = mGyro.getHeading();

            // update accumulated field position
            double dist = (encoderDist * mWheelDiam * Math.PI)/mCountsPerRev;
            boolean isXdrive = (mOpMode.virtualBot.getClass() == XDriveBot.class);  // handle X-drive too ...
            if (isXdrive)
                dist *= Math.sqrt(2);   // each wheel rotation moves the bot further with X-drive
            this.move(dist, imuBearingDeg);

            if (mOpMode != null) {
                mOpMode.telemetry.addData("EGPI position", String.format("%.2f", this.getX()) + ", " + String.format("%.2f", this.getY()));
                Position simPos = mOpMode.virtualBot.getPosition();     // get the "actual" position from the VirtualBot to see how well our PosInt is tracking ...
                mOpMode.telemetry.addData("Vbot position", String.format("%.2f", simPos.x) + ", " + String.format("%.2f", simPos.y));
            }

            return true;
        }

        public HeadingSensor getGyro() {
            return mGyro;
        }
    }

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
    class GyroPosIntGuideStep extends AutoLib.GyroGuideStep {

        OpMode mOpMode;
        Position mTarget;
        EncoderGyroPosInt mPosInt;
        double mTol;
        float mMaxPower;
        float mMinPower = 0.25f;
        float mSgnPower;

        public GyroPosIntGuideStep(EncoderGyroPosInt posInt, Position target,
                                   SensorLib.PID pid, ArrayList<AutoLib.SetPower> motorsteps, float power, double tol) {
            super(0, posInt.getGyro(), pid, motorsteps, power);
            mOpMode = AutoLib.mOpMode;
            mTarget = target;
            mPosInt = posInt;
            mTol = tol;
            mMaxPower = Math.abs(power);
            mSgnPower = (power > 0) ? 1 : -1;
        }

        public boolean loop() {
            // run the EncoderGyroPosInt to update its position based on encoders and gyro
            mPosInt.loop();

            // update the GyroGuideStep heading to continue heading for the target
            float direction = (float) HeadingToTarget(mTarget, mPosInt.getPosition());
            super.setHeading(direction);

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

    // Step: drive to given absolute field position using given EncoderGyroPosInt
    class PosIntDriveToStep extends AutoLib.GuidedTerminatedDriveStep {

        EncoderGyroPosInt mPosInt;
        Position mTarget;
        AutoLib.GyroGuideStep mGuideStep;
        PositionTerminatorStep mTerminatorStep;

        public PosIntDriveToStep(EncoderGyroPosInt posInt, DcMotor[] motors,
                                 float power, SensorLib.PID pid, Position target, double tolerance, boolean stop)
        {
            super(new GyroPosIntGuideStep(posInt, target, pid, null, power, tolerance),
                    new PositionTerminatorStep(posInt, target, tolerance, stop),
                    motors);

            mPosInt = posInt;
            mTarget = target;
        }

    }



    AutoLib.Sequence mSequence;             // the root of the sequence tree
    boolean bDone;                          // true when the programmed sequence is done
    boolean bSetup;                         // true when we're in "setup mode" where joysticks tweak parameters
    SensorLib.PID mPid;                     // PID controller for the sequence
    EncoderGyroPosInt mPosInt;              // Encoder/gyro-based position integrator to keep track of where we are
    SensorLib.PIDAdjuster mPidAdjuster;     // for interactive adjustment of PID parameters
    RobotHardware rh;                       // standard hardware set for these tests

    @Override
    public void init() {
        // tell AutoLib about this client OpMode
        AutoLib.mOpMode = this;

        // get hardware
        rh = new RobotHardware();
        rh.init(this);

        // post instructions to console
        telemetry.addData("PosIntDriveTestOp", "");
        telemetry.addData("", "autonomous point to point");
        telemetry.addData("", "navigation using PositionIntegrator");
        telemetry.addData("", "driven by motor encoders");
        telemetry.addData("", "works with all drive types.");

        // create a PID controller for the sequence
        // parameters of the PID controller for this sequence - assumes 20-gear motors (fast)
        float Kp = 0.1f;        // motor power proportional term correction per degree of deviation
        float Ki = 0.025f;         // ... integrator term
        float Kd = 0;             // ... derivative term
        float KiCutoff = 10.0f;    // maximum angle error for which we update integrator
        mPid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);    // make the object that implements PID control algorithm

        // create a PID adjuster for interactive tweaking (see loop() below)
        mPidAdjuster = new SensorLib.PIDAdjuster(this, mPid, gamepad1);

        // create Encoder/gyro-based PositionIntegrator to keep track of where we are on the field
        int countsPerRev = 28*40;		// for 40:1 gearbox motor @ 28 counts/motorRev
        double wheelDiam = 4.0;		    // wheel diameter (in)
        Position initialPosn = new Position(DistanceUnit.INCH, 0.0, 0.0, 0.0, 0);
        // example starting position: at origin of field
        mPosInt = new EncoderGyroPosInt(this, rh.mIMU, rh.mMotors, countsPerRev, wheelDiam, initialPosn);


        // create an autonomous sequence with the steps to drive
        // several legs of a polygonal course ---
        float movePower = 1.0f;
        float turnPower = 1.0f;

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

        // add a bunch of timed "legs" to the sequence - use Gyro heading convention of positive degrees CW from initial heading
        float tol = 1.0f;   // tolerance in inches
        float timeout = 2.0f;   // seconds

        // add a bunch of position integrator "legs" to the sequence -- uses absolute field coordinate system in inches
        mSequence.add(new PosIntDriveToStep(mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 36, 0., 0), tol, false));
        mSequence.add(new PosIntDriveToStep(mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 36, 36, 0., 0), tol, false));
        mSequence.add(new PosIntDriveToStep(mPosInt, rh.mMotors, movePower, mPid,                   // do this move backwards!
                new Position(DistanceUnit.INCH, 36, 0, 0., 0), tol, false));
        //mSequence.add(new PosIntDriveToStep(mPosInt, rh.mMotors, movePower, mPid,
        //        new Position(DistanceUnit.INCH, -48, -48, 0., 0), tol, false));
        mSequence.add(new PosIntDriveToStep(mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 0, 0., 0), tol, true));

        mSequence.add(new AutoLib.LogTimeStep("stopped", 3));

        mSequence.add(new PosIntDriveToStep(mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 36, 0., 0), tol, false));
        mSequence.add(new PosIntDriveToStep(mPosInt, rh.mMotors, -movePower, mPid,
                new Position(DistanceUnit.INCH, 36, 36, 0., 0), tol, false));
        mSequence.add(new PosIntDriveToStep(mPosInt, rh.mMotors, -movePower, mPid,                   // do this move backwards!
                new Position(DistanceUnit.INCH, 36, 0, 0., 0), tol, false));
        //mSequence.add(new PosIntDriveToStep(mPosInt, rh.mMotors, -movePower, mPid,                   // do this move backwards!
        //        new Position(DistanceUnit.INCH, -48, -48, 0., 0), tol, false));
        mSequence.add(new PosIntDriveToStep(mPosInt, rh.mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 0, 0., 0), tol, true));

        mSequence.add(new AutoLib.LogTimeStep("stopped", 3));

        // turn to heading zero to finish up
        mSequence.add(new AutoLib.AzimuthTolerancedTurnStep(0, rh.mIMU, mPid, rh.mMotors, turnPower, tol, 10));
        mSequence.add(new AutoLib.MoveByTimeStep(rh.mMotors, 0, 0, true));     // stop all motors

        // start out not-done
        bDone = false;
    }

    public void loop() {
        // report elapsed time to test Suspend/Resume
        telemetry.addData("elapsed time", this.getRuntime());

        if (gamepad1.y)
            bSetup = true;      // Y button: enter "setup mode" using controller inputs to set Kp and Ki
        if (gamepad1.x)
            bSetup = false;     // X button: exit "setup mode"
        if (bSetup) {           // "setup mode"
            mPidAdjuster.loop();
            return;
        }

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

