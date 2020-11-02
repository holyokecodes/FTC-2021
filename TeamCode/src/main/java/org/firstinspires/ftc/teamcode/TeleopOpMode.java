package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Jacob on 11/2/20
 */

@TeleOp
public class TeleopOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Setup motors and sensors
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            // Run OP Mode
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}