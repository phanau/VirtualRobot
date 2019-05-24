/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Gonna push this file.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package teamcode;

import virtual_robot.controller.OpMode;
import virtual_robot.controller.XDriveBot;
import virtual_robot.hardware.DcMotor;
import virtual_robot.util._Libs.AutoLib;
import virtual_robot.util._Libs.Range;
import virtual_robot.hardware.bno055.BNO055IMU;
import virtual_robot.util._Libs.BNO055IMUHeadingSensor;
import virtual_robot.util._Libs.SensorLib;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 * Uses position integration to estimate where we are on the field
 */

//@TeleOp(name="TankDrive PosInt Encoder", group="Test")  // @Autonomous(...) is the other common choice
//@Disabled
public class TankDrivePosInt extends OpMode {

	DcMotor mMotors[];

	SensorLib.PositionIntegrator mPosInt;	// position integrator
	BNO055IMUHeadingSensor mGyro;           // gyro to use for heading information

	DcMotor[] mEncoderMotor;
	int mEncoderPrev[];		// previous reading of motor encoder

	boolean bDebug = false;
	boolean bFirstLoop = true;

	/**
	 * Constructor
	 */
	public TankDrivePosInt() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {
		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */
		
		try {
			// get the motors:
			// assumed order is fr, br, fl, bl
			AutoLib.HardwareFactory mf = new AutoLib.RealHardwareFactory(this);
			mMotors = new DcMotor[4];
			mMotors[0] = mf.getDcMotor("front_right_motor");
			if (mMotors[0] != null) {
				mMotors[1] = mf.getDcMotor("back_right_motor");
				(mMotors[2] = mf.getDcMotor("front_left_motor")).setDirection(DcMotor.Direction.REVERSE);
				(mMotors[3] = mf.getDcMotor("back_left_motor")).setDirection(DcMotor.Direction.REVERSE);
			}
			else {  // assume we're using the 2-wheel bot simulation
				mMotors[0] = mMotors[1] = mf.getDcMotor("right_motor");
				(mMotors[2] = mf.getDcMotor("left_motor")).setDirection(DcMotor.Direction.REVERSE);
				(mMotors[3] = mf.getDcMotor("left_motor")).setDirection(DcMotor.Direction.REVERSE);
			}
		}
		catch (IllegalArgumentException iax) {
			bDebug = true;
		}

		// create position integrator
		mPosInt = new SensorLib.PositionIntegrator();

		// get hardware IMU and wrap gyro in HeadingSensor object usable below
		mGyro = new BNO055IMUHeadingSensor(hardwareMap.get(BNO055IMU.class, "imu"));
		mGyro.init(7);  // orientation of REV hub in my ratbot
		mGyro.setDegreesPerTurn(355.0f);     // appears that's what my IMU does ...

		bFirstLoop = true;
		mEncoderMotor = mMotors;
		mEncoderPrev = new int[4];
	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		// get initial encoder value
		if (bFirstLoop) {
			for (int i=0; i<mEncoderMotor.length; i++)
				mEncoderPrev[i] = mEncoderMotor[i].getCurrentPosition();
			bFirstLoop = false;
		}

		// tank drive
		// note that if y equal -1 then joystick is pushed all of the way forward.
		float left = -gamepad1.left_stick_y;
		float right = -gamepad1.right_stick_y;

		// clip the right/left values so that the values never exceed +/- 1
		left = Range.clip(left, -1, 1);
		right = Range.clip(right, -1, 1);

		// scale the joystick value to make it easier to control
		// the robot more precisely at slower speeds.
		final float scale = 0.75f;
		left =  (float)scaleInput(left) * scale;
		right = (float)scaleInput(right) * scale;

		// write the values to the motors - for now, front and back motors on each side are set the same
		if (!bDebug) {
			mMotors[0].setPower(right);
			mMotors[1].setPower(right);
			mMotors[2].setPower(left);
			mMotors[3].setPower(left);
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
		final int countsPerRev = 28*40;		// for 40:1 gearbox motor @ 28 counts/motorRev
		final double wheelDiam = 4.0;		// wheel diameter (in)
		double dist = (encoderDist * wheelDiam * Math.PI)/countsPerRev;
		boolean isXdrive = (this.virtualBot.getClass() == XDriveBot.class);  // handle X-drive too ...
		if (isXdrive)
			dist *= Math.sqrt(2);   // each wheel rotation moves the bot further with X-drive
		mPosInt.move(dist, imuBearingDeg);

		/*
		 * Send telemetry data back to driver station.
		 */
		telemetry.addData("Test", "*** Position Integration ***");
		telemetry.addData("left pwr", String.format("%.2f", left));
		telemetry.addData("right pwr", String.format("%.2f", right));
		telemetry.addData("gamepad1", gamepad1);
		//telemetry.addData("gamepad2", gamepad2);
		telemetry.addData("position", String.format("%.2f", mPosInt.getX())+", " + String.format("%.2f", mPosInt.getY()));
	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

	}
	
	/*
	 * This method scales the joystick input so for low joystick values, the 
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	double scaleInput(double dVal)  {
		return dVal*dVal*dVal;		// maps {-1,1} -> {-1,1}
	}

}
