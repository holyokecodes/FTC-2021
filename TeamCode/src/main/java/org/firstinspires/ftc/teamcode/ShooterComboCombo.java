package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Concept: Shooter Combo Tandem", group = "Concept")
public class ShooterComboCombo extends LinearOpMode  {

    static final double ServoIncrement   =  0.075; // amount to slew servo each CYCLE_MS cycle
    static final int    CycleMS    = 50;     // period of each cycle
    static final double MAX_POS     =  0.6;   // Maximum rotational position
    static final double MIN_POS     =  0.3;   // Minimum rotational position
    static final double MOTOR_INCREMENT   = 0.04;     // amount to ramp motor each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_FWD     =  1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV     = -1.0;     // Maximum REV power applied to motor

    // Define class members
    DcMotor shooter;
    DcMotor intake;
    double  power   = 0;
    boolean rampUp  = true;

    // Define class members
    Servo   servo;
    double  position = 0.3; // Starting position
    
    
    @Override
    public void runOpMode() {

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        servo = hardwareMap.get(Servo.class, "Finger");
        shooter = hardwareMap.get(DcMotor.class, "Shooter");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        
        // Wait for the start button
        telemetry.addData(">", "Press Start to (insert neat movie reference here)." );
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){

            // slew the servo, according to the rampUp (direction) variable.
            if (gamepad1.a) {
                // Keep stepping up until we hit the max value.
                position += ServoIncrement;
                if (position >= MAX_POS ) {
                    position = MAX_POS; 
                }
            }
            else if (gamepad1.b) {
                
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
            

            // Display the current value
            telemetry.addData("Motor Power", "%5.2f", power);
            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;
            servo.setPosition(position);
            shooter.setPower(power);
            intake.setPower(-power);
            sleep(CycleMS);
            idle();
            }
        intake.setPower(0);
        shooter.setPower(0);
        telemetry.addData(">", "Done");
        telemetry.update();
        }
    }
