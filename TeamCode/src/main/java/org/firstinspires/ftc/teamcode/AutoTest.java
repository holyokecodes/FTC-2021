package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="AutoTest")

public class AutoTest extends LinearOpMode{

    DcMotor intake;
    @Override
    public void runOpMode(){
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "Intake");

        waitForStart();
        /*
        Trajectory X = drivetrain.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(-20)))
                .strafeTo(new Vector2d(86,0))
                .build();
        Trajectory Y = drivetrain.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(-20)))
                .strafeTo(new Vector2d(0,-6))
                .build();

         */
        Trajectory ATrajectory = drivetrain.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(160)))
                .strafeTo(new Vector2d(75,16))
                .build();
        Trajectory BTrajectory = drivetrain.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(160)))
                .strafeTo(new Vector2d(86,-4))
                .build();
        Trajectory CTrajectory = drivetrain.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(160)))
                .strafeTo(new Vector2d(120,14))
                .build();
        Trajectory BReturnTrajectory = drivetrain.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(160)))
                .strafeTo(new Vector2d(-10,4))
                .build();

        Trajectory CReturnTrajectory = drivetrain.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(160)))
                .strafeTo(new Vector2d(-44,0))
                .build();
        drivetrain.followTrajectory(ATrajectory);
        intake.setPower(-.25);
        sleep(333);
        intake.setPower(0);
        //drivetrain.followTrajectory(AReturnTrajectory);
    }
}
