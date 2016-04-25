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

public class JoshVManual extends OpMode {

    final static double PLOW_MIN_RANGE  = 0.0;
    final static double PLOW_MAX_RANGE  = 1.0;
    final static double CRANE_MIN_RANGE = 0.20;
    final static double CRANE_MAX_RANGE = 1.0;

    double cranePosition;
    double craneDelta = 0.1;
    double plowPosition;
    double plowDelta = 0.1;

    DcMotor backMotorRight;
    DcMotor backMotorLeft;
    DcMotor frontMotorRight;
    DcMotor frontMotorLeft;
    Servo plow;
    Servo crane;


    public JoshVManual()
    {

    }

    @Override
    public void init()
    {
        backMotorRight = hardwareMap.dcMotor.get("back_right_drive");
        backMotorLeft = hardwareMap.dcMotor.get("back_left_drive");
        frontMotorRight = hardwareMap.dcMotor.get("front_right_drive");
        frontMotorLeft = hardwareMap.dcMotor.get("front_left_drive");

        backMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        frontMotorLeft.setDirection(DcMotor.Direction.REVERSE);

        crane = hardwareMap.servo.get("crane"); //servo 6
        plow = hardwareMap.servo.get("plow"); //servo 1

        cranePosition = 1.0;
        plowPosition = 0.52;
    }

    @Override
    public void loop()
    {
        float throttle = gamepad1.left_stick_y;
        float direction = gamepad1.left_stick_x;

        float rightRotation = throttle + direction;
        float leftRotation = throttle - direction;

        rightRotation = Range.clip(rightRotation, -1, 1);
        leftRotation = Range.clip(rightRotation, -1, 1);

        rightRotation = (float)scaleInput(leftRotation);
        leftRotation =  (float)scaleInput(leftRotation);

        backMotorRight.setPower(-rightRotation);
        frontMotorRight.setPower(-rightRotation);
        backMotorLeft.setPower(-leftRotation);
        frontMotorLeft.setPower(-leftRotation);

        if (gamepad2.dpad_up) {
            plowPosition += plowDelta;
        }

        if (gamepad2.dpad_down) {
            plowPosition = 0.52;
        }

        if ((gamepad2.left_bumper))
        {
            cranePosition += craneDelta;
        }
        if (gamepad2.right_bumper)
        {
            cranePosition -= craneDelta;
        }

        if(gamepad1.left_bumper)
        {
            plowPosition = 0.0;
        }

        if (gamepad1.right_bumper)
        {
            plowPosition = 1.0;
        }

        plowPosition = Range.clip(plowPosition, PLOW_MIN_RANGE, PLOW_MAX_RANGE);
        cranePosition = Range.clip(cranePosition, CRANE_MIN_RANGE,CRANE_MAX_RANGE);

        plow.setPosition(plowPosition);
        crane.setPosition(cranePosition);
    }

    @Override
    public void stop()
    {

    }

    double scaleInput(double dVal)
    {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        int index = (int) (dVal * 16.0);

        if (index < 0) {
            index = -index;
        }

        if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }

}
