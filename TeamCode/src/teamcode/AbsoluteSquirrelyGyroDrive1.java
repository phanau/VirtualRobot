/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

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
import virtual_robot.hardware.DcMotor;
import virtual_robot.hardware.bno055.BNO055IMU;
import virtual_robot.util._Libs.BNO055IMUHeadingSensor;
import virtual_robot.util._Libs.AutoLib;

/*
 * TeleOp Mode
 * Enables control of the robot via the gamepad such that the robot moves in the
 * absolute direction and speed indicated by the left joystick, assuming the game console is
 * aligned with the robot when the mode is initiated, and the robot maintains an orientation
 * aligned with the direction of the right joystick..
 */

//@TeleOp(name="AbsoluteSquirrelyGyroDrive1", group="Test")  // @Autonomous(...) is the other common choice
//@Disabled
public class AbsoluteSquirrelyGyroDrive1 extends OpMode {

	AutoLib.SquirrelyGyroTimedDriveStep mStep;
	RobotHardware rh;

	/**
	 * Constructor
	 */
	public AbsoluteSquirrelyGyroDrive1() {

	}

	@Override
	public void init() {

		// get hardware
		rh = new RobotHardware();
		rh.init(this);

		// create a Step that we will use in teleop mode
		mStep = new AutoLib.SquirrelyGyroTimedDriveStep(this, 0, 0, rh.mIMU, null, rh.mMotors, 0, 10000, false);
	}


	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		// motion direction is on the right stick
		float dx = gamepad1.right_stick_x;
		float dy = -gamepad1.right_stick_y;	// y is reversed :(

		// power is the magnitude of the direction vector
		double power = Math.sqrt(dx*dx + dy*dy);
		mStep.setPower((float) power);

		// make sure we can rotate even if we're not moving
		mStep.setMaxPower((float) 1.0);

		// we don't have a valid direction when inputs are "zero"
		final double MIN_INPUT = 0.1;
		if (power > MIN_INPUT) {
			// direction angle of stick >> the direction we want to move
			double direction = Math.atan2(-dx, dy);    // stick angle: zero = +y, positive CCW, range +-pi
			direction *= 180.0 / Math.PI;        // radians to degrees
			mStep.setDirection((float) direction);
		}

		// vehicle heading (orientation) is on the left stick (near the dpad, which also controls heading)
		float hx = gamepad1.left_stick_x;
		float hy = -gamepad1.left_stick_y;    // y is reversed :(

		double heading = 0;
		boolean setHeading = false;
		double hMag = Math.sqrt(hx*hx + hy*hy);
		if (hMag > MIN_INPUT) {
			// direction angle of stick >> the direction we want to face
			heading = Math.atan2(-hx, hy);    // stick angle: zero = +y, positive CCW, range +-pi
			heading *= 180.0 / Math.PI;        // radians to degrees
			setHeading = true;
		}

		// also allow inputting of orientation on 8-way pad
		if (gamepad1.dpad_up) { heading = 0; setHeading = true; }
		if (gamepad1.dpad_right) { heading = -90; setHeading = true; }
		if (gamepad1.dpad_down) { heading = 180; setHeading = true; }
		if (gamepad1.dpad_left) { heading = 90; setHeading = true; }
		if (gamepad1.dpad_up && gamepad1.dpad_right) { heading = -45; setHeading = true; }
		if (gamepad1.dpad_down && gamepad1.dpad_right) { heading = -135; setHeading = true; }
		if (gamepad1.dpad_down && gamepad1.dpad_left) { heading = 135; setHeading = true; }
		if (gamepad1.dpad_up && gamepad1.dpad_left) { heading = 45; setHeading = true; }

		if (setHeading)
			mStep.setHeading((float) heading);

		// run the control step
		mStep.loop();
	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {
	}

}
