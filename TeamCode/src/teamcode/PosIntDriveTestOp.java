package teamcode;

import virtual_robot.controller.OpMode;
import virtual_robot.hardware.ColorSensor;
import virtual_robot.hardware.DcMotor;
import virtual_robot.hardware.DistanceSensor;
import virtual_robot.hardware.GyroSensor;
import virtual_robot.hardware.Servo;
import virtual_robot.hardware.Telemetry;
import virtual_robot.hardware.bno055.BNO055IMU;
import virtual_robot.util._Libs.BNO055IMUHeadingSensor;
import virtual_robot.util.navigation.AngleUnit;
import virtual_robot.util.navigation.AxesOrder;
import virtual_robot.util.navigation.AxesReference;
import virtual_robot.util.navigation.DistanceUnit;
import virtual_robot.util.navigation.Orientation;

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

    // use a single motor encoder and gyro to track absolute field position
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
            // get initial encoder value
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
            this.move(dist, imuBearingDeg);

            if (mOpMode != null)
                mOpMode.telemetry.addData("EGPI position", String.format("%.2f", this.getX())+", " + String.format("%.2f", this.getY()));

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

        public PositionTerminatorStep(OpMode opmode, SensorLib.PositionIntegrator posInt, Position target, double tol) {
            mOpMode = opmode;
            mPosInt = posInt;
            mTarget = target;
            mTol = tol;
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
            return bDone;
        }
    }

    // guide step that uses a gyro and a position integrator to determine how to guide the robot to the target
    class GyroPosIntGuideStep extends AutoLib.GyroGuideStep {

        OpMode mOpMode;
        Position mTarget;
        EncoderGyroPosInt mPosInt;

        public GyroPosIntGuideStep(OpMode opmode, EncoderGyroPosInt posInt, Position target,
                                   SensorLib.PID pid, ArrayList<AutoLib.SetPower> motorsteps, float power) {
            super(opmode, 0, posInt.getGyro(), pid, motorsteps, power);
            mOpMode = opmode;
            mTarget = target;
            mPosInt = posInt;
        }

        public boolean loop() {
            // run the EncoderGyroPosInt to update its position based on encoders and gyro
            mPosInt.loop();

            // update the GyroGuideStep heading to continue heading for the target
            super.setHeading((float) HeadingToTarget(mTarget, mPosInt.getPosition()));

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

        OpMode mOpMode;
        EncoderGyroPosInt mPosInt;
        Position mTarget;
        AutoLib.GyroGuideStep mGuideStep;
        PositionTerminatorStep mTerminatorStep;

        public PosIntDriveToStep(OpMode opmode, EncoderGyroPosInt posInt, DcMotor[] motors,
                                 float power, SensorLib.PID pid, Position target, double tolerance, boolean stop)
        {
            super(opmode,
                    new GyroPosIntGuideStep(opmode, posInt, target, pid, null, power),
                    new PositionTerminatorStep(opmode, posInt, target, tolerance),
                    motors);

            mOpMode = opmode;
            mPosInt = posInt;
            mTarget = target;
        }

    }



    AutoLib.Sequence mSequence;             // the root of the sequence tree
    boolean bDone;                          // true when the programmed sequence is done
    DcMotor mMotors[];                      // motors, some of which can be null: assumed order is fr, br, fl, bl
    BNO055IMUHeadingSensor mGyro;           // gyro to use for heading information
    boolean bSetup;                         // true when we're in "setup mode" where joysticks tweak parameters
    SensorLib.PID mPid;                     // PID controller for the sequence
    EncoderGyroPosInt mPosInt;              // Encoder/gyro-based position integrator to keep track of where we are
    SensorLib.PIDAdjuster mPidAdjuster;     // for interactive adjustment of PID parameters

    @Override
    public void init() {
        bSetup = false;      // start out in Kp/Ki setup mode
        AutoLib.HardwareFactory mf = null;
        final boolean debug = false;
        //if (debug)
        //    mf = new AutoLib.TestHardwareFactory(this);
        //else
            mf = new AutoLib.RealHardwareFactory(this);

        // get the motors: depending on the factory we created above, these may be
        // either dummy motors that just log data or real ones that drive the hardware
        // assumed order is fr, br, fl, bl
        mMotors = new DcMotor[4];
        mMotors[0] = mf.getDcMotor("front_right_motor");
        mMotors[1] = mf.getDcMotor("back_right_motor");
        (mMotors[2] = mf.getDcMotor("front_left_motor")).setDirection(DcMotor.Direction.REVERSE);
        (mMotors[3] = mf.getDcMotor("back_left_motor")).setDirection(DcMotor.Direction.REVERSE);

        // get hardware IMU and wrap gyro in HeadingSensor object usable below
        mGyro = new BNO055IMUHeadingSensor(hardwareMap.get(BNO055IMU.class, "imu"));
        mGyro.init(7);  // orientation of REV hub in my ratbot
        mGyro.setDegreesPerTurn(355.0f);     // appears that's what my IMU does ...
        mGyro.setHeadingOffset(0.0f);        // example: facing due north along field Y-axis (positive CCW)

        // create a PID controller for the sequence
        // parameters of the PID controller for this sequence - assumes 20-gear motors (fast)
        float Kp = 0.02f;        // motor power proportional term correction per degree of deviation
        float Ki = 0.025f;         // ... integrator term
        float Kd = 0;             // ... derivative term
        float KiCutoff = 10.0f;    // maximum angle error for which we update integrator
        mPid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);    // make the object that implements PID control algorithm

        // create a PID adjuster for interactive tweaking (see loop() below)
        mPidAdjuster = new SensorLib.PIDAdjuster(this, mPid, gamepad1);

        // create Encoder/gyro-based PositionIntegrator to keep track of where we are on the field
        int countsPerRev = 28*20;		// for 20:1 gearbox motor @ 28 counts/motorRev
        double wheelDiam = 4.0;		    // wheel diameter (in)
        Position initialPosn = new Position(DistanceUnit.INCH, 0.0, 0.0, 0.0, 0);
        // example starting position: at origin of field
        mPosInt = new EncoderGyroPosInt(this, mGyro, mMotors, countsPerRev, wheelDiam, initialPosn);


        // create an autonomous sequence with the steps to drive
        // several legs of a polygonal course ---
        float movePower = 0.20f;
        float turnPower = 0.25f;

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

        // add a bunch of timed "legs" to the sequence - use Gyro heading convention of positive degrees CW from initial heading
        float tol = 3.0f;   // tolerance in inches
        float timeout = 2.0f;   // seconds

        // add a bunch of position integrator "legs" to the sequence -- uses absolute field coordinate system in inches
        mSequence.add(new PosIntDriveToStep(this, mPosInt, mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 36, 0., 0), tol, false));
        mSequence.add(new PosIntDriveToStep(this, mPosInt, mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 36, 36, 0., 0), tol, false));
        mSequence.add(new PosIntDriveToStep(this, mPosInt, mMotors, movePower, mPid,                   // do this move backwards!
                new Position(DistanceUnit.INCH, 36, 0, 0., 0), tol, false));
        mSequence.add(new PosIntDriveToStep(this, mPosInt, mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 0, 0., 0), tol, false));

        mSequence.add(new PosIntDriveToStep(this, mPosInt, mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 36, 0., 0), tol, false));
        mSequence.add(new PosIntDriveToStep(this, mPosInt, mMotors, -movePower, mPid,
                new Position(DistanceUnit.INCH, 36, 36, 0., 0), tol, false));
        mSequence.add(new PosIntDriveToStep(this, mPosInt, mMotors, -movePower, mPid,                   // do this move backwards!
                new Position(DistanceUnit.INCH, 36, 0, 0., 0), tol, false));
        mSequence.add(new PosIntDriveToStep(this, mPosInt, mMotors, movePower, mPid,
                new Position(DistanceUnit.INCH, 0, 0, 0., 0), tol, false));

        // turn to heading zero to finish up
        mSequence.add(new AutoLib.AzimuthTolerancedTurnStep(this, 0, mGyro, mPid, mMotors, turnPower, tol, timeout));
        mSequence.add(new AutoLib.MoveByTimeStep(mMotors, 0, 0, true));     // stop all motors

        // start out not-done
        bDone = false;
    }

    public void loop() {

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

