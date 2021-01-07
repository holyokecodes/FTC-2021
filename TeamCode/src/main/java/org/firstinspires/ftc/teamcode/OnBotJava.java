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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static java.lang.Math.*;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 * <p>
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * <p>
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Mecanum Drive Program", group = "Pushbot")

public class OnBotJava extends OpMode {

    public DcMotor LeftFrontMotor;
    public DcMotor RightFrontMotor;
    public DcMotor LeftBackMotor;
    public DcMotor RightBackMotor;

    /* Declare OpMode members. */
    //HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
    double clawOffset = 0.0;                  // Servo mid position
    final double CLAW_SPEED = 0.02;                 // sets rate to move servo

    @Override
    public void init() {
        // Define motors and servoooooos
        LeftFrontMotor = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        RightFrontMotor = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        LeftBackMotor = hardwareMap.get(DcMotor.class, "LeftBackMotor");
        RightBackMotor = hardwareMap.get(DcMotor.class, "RightBackMotor");

        // Set all motors to zero power
        LeftFrontMotor.setPower(0);
        RightFrontMotor.setPower(0);
        LeftBackMotor.setPower(0);
        RightBackMotor.setPower(0);

        //Set motor NoSuchMethodException
        LeftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //zeroMotorBevahior
        LeftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
    @Override
    public void loop() {
        double x;
        double y;
        double maxSpeed = 1;
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        x = -gamepad1.left_stick_x;
        y = gamepad1.left_stick_y;
        telemetry.addData("x/y value", "x (%.2f) y (%.2f)", x, y);
        double lfm = 0;
        double lbm = 0;
        double rfm = 0;
        double rbm = 0;

        if (abs(x) >= 0.01 && abs(y) >= 0.01) {

            double angle = Math.atan2(y, x) * (180 / Math.PI);
            telemetry.addData("angle", "(%.2f)", angle);
            double radiandirection = Math.toRadians(angle);
            telemetry.addData("radiandirection", "(%.2f)", radiandirection);

            lfm = Math.cos(radiandirection) - Math.sin(radiandirection) * maxSpeed;
            rfm = Math.cos(radiandirection) + Math.sin(radiandirection) * maxSpeed;
            lbm = Math.cos(radiandirection) + Math.sin(radiandirection) * maxSpeed;
            rbm = Math.cos(radiandirection) - Math.sin(radiandirection) * maxSpeed;

            double[] motors = {lfm, lbm, rfm, rbm};

            Arrays.sort(motors);

            double min = motors[0];
            double max = motors[3];

            double powerReducer = 1;
            if (Math.abs(max) > maxSpeed || Math.abs(min) > maxSpeed) {
                if (Math.abs(max) >= Math.abs(min)) {
                    powerReducer *= (1 / max);
                } else {
                    powerReducer = (maxSpeed / Math.abs(min));
                }
            }
            lfm *= powerReducer;
            lbm *= powerReducer;
            rfm *= powerReducer;
            rbm *= powerReducer;

            telemetry.addData("lfm, lbm, rfm, rbm", "lfm (%.2f) lbm (%.2f) rfm (%.2f) rbm (%.2f)", lfm, lbm, rfm, rbm);


            LeftFrontMotor.setPower(lfm);
            LeftBackMotor.setPower(lbm);
            RightFrontMotor.setPower(rfm);
            RightBackMotor.setPower(rbm);
        } else {
            LeftFrontMotor.setPower(0);
            LeftBackMotor.setPower(0);
            RightFrontMotor.setPower(0);
            RightBackMotor.setPower(0);
        }
    }


    public void stop() {

    }
}
