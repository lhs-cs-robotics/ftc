/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="WolfBot Supreme", group="Pushbot")
//@Disabled
public class PushbotTeleopTank_Iterative extends OpMode{

    /* Declare OpMode members. */
    org.firstinspires.ftc.teamcode.HardwarePushbot robot = new org.firstinspires.ftc.teamcode.HardwarePushbot(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    //double          clawBarrierOffset  = -1 ; //Each servo you want to move seperately
    //double          clawRotationOffset = 1;   //has to have its own variable offset
    //double          clawLiftOffset = -1;
    //final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
    double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
    double rightX = gamepad1.right_stick_x;
    final double v1 = r * Math.cos(robotAngle) + rightX;
    final double v2 = r * Math.sin(robotAngle) - rightX;
    final double v3 = r * Math.sin(robotAngle) + rightX;
    final double v4 = r * Math.cos(robotAngle) - rightX;

    leftFront.setPower(v1);
    rightFront.setPower(v2);
    leftRear.setPower(v3)
            rightRear.setPower(v4);

    @Override
    public void loop() {
        double left;
        double right;
/*
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;
        if(gamepad1.dpad_left)
        {
            robot.frontLeftDrive.setPower(.4);
            robot.frontRightDrive.setPower(-.4);
            robot.backLeftDrive.setPower(-.6);
            robot.backRightDrive.setPower(.6);
        }
        else if(gamepad1.dpad_right)
        {
            robot.frontLeftDrive.setPower(-.4);
            robot.frontRightDrive.setPower(.4);
            robot.backLeftDrive.setPower(.6);
            robot.backRightDrive.setPower(-.6);
        }
        else
        {
            robot.frontLeftDrive.setPower(left);
            robot.frontRightDrive.setPower(right);
            robot.backLeftDrive.setPower(left);
            robot.backRightDrive.setPower(right);
        }
*/
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        robot.frontLeftDrive.setPower(v1);
        robot.frontRightDrive.setPower(v2);
        robot.backLeftDrive.setPower(v3);
        robot.backRightDrive.setPower(v4);

        if(gamepad1.right_trigger > 0)
        {
            robot.frontLeftDrive.setPower(-1);
            robot.frontRightDrive.setPower(1);
            robot.backLeftDrive.setPower(-1);
            robot.backRightDrive.setPower(1);
        }
        else if(gamepad1.left_trigger > 0)
        {
            robot.frontLeftDrive.setPower(1);
            robot.frontRightDrive.setPower(-1);
            robot.backLeftDrive.setPower(1);
            robot.backRightDrive.setPower(-1);
        }


        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad1.right_bumper)
        {
            robot.leftArm.setPosition(.9);
            robot.rightArm.setPosition(0);
            //clawRotationOffset -= CLAW_SPEED; //Changes servo position in code
        }
        else if (gamepad1.a)
        {
            robot.rightArm.setPosition(.9);
            robot.leftArm.setPosition(0.2);
            //clawRotationOffset += CLAW_SPEED;//Changes servo position in code
        }
        else {
            robot.leftArm.setPosition(0.493);
            robot.rightArm.setPosition(0.493);
        }



        if(gamepad1.x) {

            telemetry.addLine()
                    .addData("red", robot.color.red())
                    .addData("green", robot.color.green())
                    .addData("blue", robot.color.blue());
            telemetry.addData("argb", robot.color.argb());
        }

        if (gamepad1.y) {
            robot.color.enableLed(false);
        }

        if(gamepad1.b) {
            robot.color.enableLed(true);
        }




        //clawLiftOffset = Range.clip(clawLiftOffset, -1, 1);//Makes sure servo doesn't go past its range if it isn't continuous
        //clawRotationOffset = Range.clip(clawRotationOffset, -1, 1);//Makes sure servo doesn't go past its range if it isn't continuous
        //clawBarrierOffset = Range.clip(clawBarrierOffset, -1, 1);//Makes sure servo doesn't go past its range if it isn't continuous
        //robot.clawTilt.setPosition(clawRotationOffset);//Actually physically moves servo
        //robot.clawBarrier.setPosition(clawBarrierOffset);//Actually physically moves servo
        //robot.clawLiftRight.setPosition(-clawLiftOffset);//Actually physically moves servo
        //robot.clawLiftLeft.setPosition(clawLiftOffset);//Actually physically moves servo



        // Send telemetry message to signify robot running;

        //telemetry.addData("left",  "%.2f", left);
        //telemetry.addData("right", "%.2f", right);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}