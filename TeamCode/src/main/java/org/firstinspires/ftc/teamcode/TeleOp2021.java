/*
Copyright 2020 FIRST Tech Challenge Team 14853
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,http://192.168.49.1:8080/java/editor.html?/src/org/firstinspires/ftc/teamcode/MechanumDrive.java
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import java.util.Comparator;
import java.util.stream.Stream;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp(name = "Tele-op 2021")
public class TeleOp2021 extends LinearOpMode {
//    private Blinker expansion_Hub_2;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private Servo finger;
    private DcMotor shooter;
    private DcMotor intake;

    private BNO055IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    private final double sensitivity = 1;

    static final double ServoIncrement   =  0.075; // amount to slew servo each CYCLE_MS cycle
    // Maybe make this higher
    static final int    CycleMS    = 50;     // period of each cycle
    static final double MAX_POS     =  0.6;   // Maximum rotational position
    static final double MIN_POS     =  0.3;   // Minimum rotational position
    static final double MOTOR_INCREMENT   = 0.04;     // amount to ramp motor each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_FWD     =  1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV     = -1.0;     // Maximum REV power applied to motor

    // Define class members
    double  power   = 0;
    boolean rampUp  = true;

    // Define class members
    double  position = 0.3; // Starting position

    float joystickDeadzone = 0.1f;

    //    private double intakePower;
    @Override
    public void runOpMode() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        finger = hardwareMap.get(Servo.class, "Finger");
        shooter = hardwareMap.get(DcMotor.class, "Shooter");
        intake = hardwareMap.get(DcMotor.class, "Intake");

        boolean shooterOn = false;

        // Wait for the start button
        telemetry.addData(">", "Press Start to (insert neat movie reference here)." );
        telemetry.update();

        gamepad1.setJoystickDeadzone(joystickDeadzone);

        waitForStart();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double speedMultiplier = 1; //Multiplier for precision mode.
            if (gamepad1.right_trigger > 0.5){
                speedMultiplier = 0.5;
                telemetry.addData("Pesise Mode", "On");
            } else {
                telemetry.addData("Pesise Mode", "Off");
            }

            Orientation angles=imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX, AngleUnit.DEGREES);
            double forward = -gamepad1.left_stick_y;
            double right  =  -gamepad1.left_stick_x;
            double clockwise = gamepad1.right_stick_x;
            clockwise *= sensitivity;
            //this part of the code controls the mechanum drive
            double theta = 0;
            telemetry.addData("Theta", Double.toString(theta));
            //double theta = 0;
            double temp = forward * Math.sin(theta) + right * Math.cos(theta);
            right = forward * Math.cos(theta) - right * Math.sin(theta);
            forward = -temp;
            //right -= .005;
            double frontLeftPower = -forward - clockwise - right;
            double frontRightPower = forward - clockwise - right;
            double rearLeftPower = -forward - clockwise + right;
            double rearRightPower = forward - clockwise + right;
            telemetry.addData("Left Stick X", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
            double max = Math.abs(frontLeftPower);
            if (Math.abs(frontRightPower) > max) {
                max=Math.abs(frontRightPower);
            }
            if (Math.abs(rearRightPower) > max) {
                max=Math.abs(rearRightPower);
            }
            if (Math.abs(rearLeftPower) > max) {
                max=Math.abs(rearLeftPower);
            }
            if(max>1){
                frontLeftPower /= max;
                frontRightPower /= max;
                rearLeftPower /= max;
                rearRightPower /= max;
            }
            backLeft.setPower(rearLeftPower * speedMultiplier);
            backRight.setPower(rearRightPower * speedMultiplier);
            frontLeft.setPower(frontLeftPower * speedMultiplier);
            frontRight.setPower(frontRightPower * speedMultiplier);


            // slew the servo, according to the rampUp (direction) variable
            if (gamepad2.a) {
                // Keep stepping up until we hit the max value.
                position += ServoIncrement;
                if (position >= MAX_POS ) {
                    position = MAX_POS;
                }
            }
            else if (gamepad2.b) {

                // Keep stepping down until we hit the min value.
                position -= ServoIncrement;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                }
            }

            power += MOTOR_INCREMENT ;
            if (power <= MAX_REV ) {
                power = MAX_REV;
                rampUp = !rampUp;  // Switch ramp direction
            }
            // if you hit the x button, invert the state of the shooter.
            if (gamepad2.x){
                shooterOn = !shooterOn;
            }
            // Display the current values
            telemetry.addData("Motor Power", "%5.2f", power);
            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.update();

            // Set the servo to the new position and pause;
            finger.setPosition(position);
            if (shooterOn){
                shooter.setPower(power);
            }else{
                shooter.setPower(0);
            }
            intake.setPower(0);//power);
            sleep(CycleMS);
            idle();

            telemetry.addData("Status", "Running");
            telemetry.addData("angle", angles.firstAngle);
            telemetry.update();
        }
    }
}