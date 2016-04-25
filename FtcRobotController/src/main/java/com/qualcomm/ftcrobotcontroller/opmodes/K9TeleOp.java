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

package com.qualcomm.ftcrobotcontroller.opmodes;

import android.text.StaticLayout;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class K9TeleOp extends OpMode {

	/*
	 * Note: the configuration of the servos is such that
	 * as the rightArm servo approaches 0, the rightArm position moves up (away from the floor).
	 * Also, as the plow servo approaches 0, the plow opens up (drops the game element).
	 */
	// TETRIX VALUES.
	//final static double RIGHT_ARM_MIN_RANGE  = 0.50;
	//final static double RIGHT_ARM_MAX_RANGE  = 1.0;
	// final static double LEFT_ARM_MIN_RANGE = 0.40;
	// final static double LEFT_ARM_MAX_RANGE = 1.0;
	final static double PLOW_MIN_RANGE  = 0.0;
	final static double PLOW_MAX_RANGE  = 1.0;
	final static double CRANE_MIN_RANGE = 0.20;
	final static double CRANE_MAX_RANGE = 1.0;



	//position of the crane
	double cranePosition;

	//amount to change the crane servo position
	double craneDelta = 0.1;

	// position of the rightArm servo.
	//double rightArmPosition;

	// position of the leftArm servo.
	//   double leftArmPosition;

	// amount to change the rightArm servo position.
	//double rightArmDelta = 0.1;

	//amount to change the leftArm servo position
	// double leftArmDelta = 0.1;

	// position of the plow servo
	double plowPosition;

	// amount to change the plow servo position by
	double plowDelta = 0.1;

	DcMotor backMotorRight;
	DcMotor backMotorLeft;
	DcMotor frontMotorRight;
	DcMotor frontMotorLeft;
	Servo plow;
	//Servo rightArm;
	//Servo leftArm;
	Servo crane;


	/**
	 * Constructor
	 */
	public K9TeleOp() {
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
		
		/*
		 * For the demo Tetrix K9 bot we assume the following,
		 *   There are two motors "motor_1" and "motor_2"
		 *   "motor_1" is on the right side of the bot.
		 *   "motor_2" is on the left side of the bot and reversed.
		 *   
		 * We also assume that there are two servos "servo_1" and "servo_6"
		 *    "servo_1" controls the rightArm joint of the manipulator.
		 *    "servo_6" controls the plow joint of the manipulator.
		 */
		backMotorRight = hardwareMap.dcMotor.get("back_right_drive");
		backMotorLeft = hardwareMap.dcMotor.get("back_left_drive");
		frontMotorRight = hardwareMap.dcMotor.get("front_right_drive");
		frontMotorLeft = hardwareMap.dcMotor.get("front_left_drive");

		backMotorLeft.setDirection(DcMotor.Direction.REVERSE);
		frontMotorLeft.setDirection(DcMotor.Direction.REVERSE);

		crane = hardwareMap.servo.get("crane"); //servo 6
		//leftArm = hardwareMap.servo.get("left_arm"); //servo 3
		//rightArm = hardwareMap.servo.get("right_arm"); //servo 2
		plow = hardwareMap.servo.get("plow"); //servo 1

		// assign the starting position of the wrist and plow
		//rightArmPosition = 1.0;
		//leftArmPosition = 0.0;
		cranePosition = 1.0;
		plowPosition = 0.52;
	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		/*
		 * Gamepad 1
		 * 
		 * Gamepad 1 controls the motors via the left stick, and Gamepad 2 controls the
		 * plow via the dpad up/down buttons
		 * and the arms by the x,y,a,b buttons
		 * and the crane by the left bumper and the right bumper.
		 */

		// throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
		// 1 is full down
		// direction: left_stick_x ranges from -1 to 1, where -1 is full left
		// and 1 is full right
		float throttle = gamepad1.left_stick_y;
		float direction = -gamepad1.left_stick_x;
		float right = throttle - direction;
		float left = throttle + direction;

		// clip the right/left values so that the values never exceed +/- 1
		right = Range.clip(right, -1, 1);
		left = Range.clip(left, -1, 1);

		// scale the joystick value to make it easier to control
		// the robot more precisely at slower speeds.
		right = (float)scaleInput(right);
		left =  (float)scaleInput(left);

		// write the values to the motors
		backMotorRight.setPower(-right);
		frontMotorRight.setPower(-right);
		backMotorLeft.setPower(-left);
		frontMotorLeft.setPower(-left);

		// update the position of the rightArm.
		if (gamepad2.a) {
			// if the A button is pushed on gamepad1, increment the position of
			// the rightArm servo.
			//rightArmPosition += rightArmDelta;
		}

		if (gamepad2.b) {
			// if the Y button is pushed on gamepad1, decrease the position of
			// the rightArm servo.
			//rightArmPosition -= rightArmDelta;
		}

		//update the position of the left arm
		if (gamepad2.y)
		{
			// leftArmPosition += leftArmDelta;
		}
		if (gamepad2.x)
		{
			// leftArmPosition -= leftArmDelta;
		}

		// update the position of the plow
		if (gamepad2.dpad_up) {
			plowPosition += plowDelta;
		}

		if (gamepad2.dpad_down) {
			plowPosition = 0.52;
		}

		//update the position of the crane
		if ((gamepad2.left_bumper))
		{
			cranePosition += craneDelta;
		}
		if (gamepad2.right_bumper)
		{
			cranePosition -= craneDelta;
		}

		//drop the plow to the floor
		if(gamepad1.left_bumper)
		{
			plowPosition = 0.0;
		}
		//raise it al the way up.
		if (gamepad1.right_bumper)
		{
			plowPosition = 1.0;
		}


		// clip the position values so that they never exceed their allowed range.
		// rightArmPosition = Range.clip(rightArmPosition, RIGHT_ARM_MIN_RANGE, RIGHT_ARM_MAX_RANGE);
		// leftArmPosition = Range.clip(leftArmPosition, LEFT_ARM_MIN_RANGE, LEFT_ARM_MAX_RANGE);
		plowPosition = Range.clip(plowPosition, PLOW_MIN_RANGE, PLOW_MAX_RANGE);
		cranePosition = Range.clip(cranePosition, CRANE_MIN_RANGE,CRANE_MAX_RANGE);

		// write position values to the wrist and plow servo
		//	rightArm.setPosition(rightArmPosition);
		//  leftArm.setPosition(leftArmPosition);
		plow.setPosition(plowPosition);
		crane.setPosition(cranePosition);



		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        /*telemetry.addData("Text", "*** Robot Data***");
      //  telemetry.addData("rightArm", "rightArm:  " + String.format("%.2f", rightArmPosition));
       // telemetry.addData("leftArm", "leftArm:  " + String.format("%.2f",leftArmPosition));
        telemetry.addData("plow", "plow:  " + String.format("%.2f", plowPosition));
        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
        telemetry.addData("crane", "crane: " + String.format("%.2f", cranePosition));*/

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
		double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);

		// index should be positive.
		if (index < 0) {
			index = -index;
		}

		// index cannot exceed size of array minus 1.
		if (index > 16) {
			index = 16;
		}

		// get value from the array.
		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		// return scaled value.
		return dScale;
	}

}
