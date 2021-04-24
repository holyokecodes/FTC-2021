package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;


@Autonomous(name="Autonomous 2021 Shooter")

public class Auto2021Shooter extends LinearOpMode{

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "ATCqtA7/////AAABmQOFKAEJkk6fv6wgCQpID2hsV7lEm5wVpp8w+fJl/t3sb/GHzuVacXD5hvHQFp5yC24CVqwQz+4S8vpn6QqGV0s2Ib4RRCy7BZKJJkpreAYHpEczx/OERceRomSzGKp6VbV/2NB4XCMWh8p31v1gW0BGJeyA+tTybvJWo9JWFy+/qmfz0FHN/wHrZ5lG9hSM3V+Y1IvaXR/FVmwrmfZm8dhb5DG/iqBI/poHVC7nGuNwjtBEh5xPHyAIZWqkauulH+4amlDZ1DBOp4K7zZXgSf+YCvOZjRvv1CrOcgwunVoNPgRmVnJ2K4zIbyYA/S2NadKEUAjD7OFLDBay9+x0/B9C9pQSl2mowul9vBlAqRfe";


    /**
     * Initialize the Vuforia localization engine.
     */

    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    DcMotor intake;
    DcMotor shooter;
    Servo finger;

    SampleMecanumDrive drivetrain;

    private Vector2d targetZoneA = new Vector2d(60, 16);
    private Vector2d targetZoneB = new Vector2d(85, 16);
    private Vector2d targetZoneC = new Vector2d(101, 18);

    private Vector2d missRings = new Vector2d(37, -32);

    private Pose2d start = new Pose2d(0, 0 , Math.toRadians(180)); //Where the robot starts
    private Pose2d missRingsStart = new Pose2d(missRings, Math.toRadians(180)); //After missing the rings

    private Pose2d aZone = new Pose2d(targetZoneA, Math.toRadians(90)); //Where the robot is after going to zone a
    private Pose2d bZone = new Pose2d(targetZoneB, Math.toRadians(90)); //Where the robot is after going to zone b
    private Pose2d cZone = new Pose2d(targetZoneC, Math.toRadians(180)); //Where the robot is after going to zone c

    private Vector2d whiteLine = new Vector2d(45, 0);

    private Pose2d whiteLinePose = new Pose2d(whiteLine, Math.toRadians(180));


    Trajectory ATrajectory;
    Trajectory BTrajectory;
    Trajectory CTrajectory;

    Trajectory AReturnTrajectory;
    Trajectory BReturnTrajectory;
    Trajectory CReturnTrajectory;

    Trajectory MissRingsTrajectory;

    Trajectory EndTrajectory;

    @Override
    public void runOpMode(){
        intake = hardwareMap.get(DcMotor.class, "Intake");
        shooter = hardwareMap.get(DcMotor.class, "Shooter");
        finger = hardwareMap.get(Servo.class, "Finger");

        drivetrain = new SampleMecanumDrive(hardwareMap);

        drivetrain.setPoseEstimate(start);

        ATrajectory = drivetrain.trajectoryBuilder(start)
                .splineToSplineHeading(new Pose2d(-60, -40, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-6, -60, Math.toRadians(0)), Math.toRadians(0))
                .build();
        BTrajectory = drivetrain.trajectoryBuilder(start)
//                .strafeTo(targetZoneB)
//                .splineTo(missRings, Math.toRadians(180)) //Go off to the side of the rings
                .splineTo(missRings, 0)
                .splineTo(targetZoneB, 0) //Go to the target zone
//                .splineTo(whiteLine, Math.toRadians(-90)) //Go to the white line to shoot
                .build();
        CTrajectory = drivetrain.trajectoryBuilder(start)
//                .strafeTo(targetZoneC)
//                .splineTo(missRings, Math.toRadians(180)) //go off to the side of the rings
                .splineTo(missRings, 0)
                .splineTo(targetZoneC, 0) //Go to the target zone
//                .splineTo(whiteLine, Math.toRadians(-90)) //GO to the white line to shoot
                .build();



        AReturnTrajectory = drivetrain.trajectoryBuilder(aZone)
                .splineToSplineHeading(new Pose2d(-6, -60, Math.toRadians(0)), Math.toRadians(130))
                .splineToSplineHeading(new Pose2d(0, 0, Math.toRadians(180)), Math.toRadians(0))
                .build();
        BReturnTrajectory = drivetrain.trajectoryBuilder(bZone)
//                .strafeTo(whiteLine)
                .splineTo(whiteLine, 0)
                .build();
        CReturnTrajectory = drivetrain.trajectoryBuilder(cZone)
//                .strafeTo(whiteLine)
                .splineTo(whiteLine, 0)
                .build();



        MissRingsTrajectory = drivetrain.trajectoryBuilder(start)
//                .strafeTo(missRings)
                .splineTo(missRings, 0)
                .build();
        EndTrajectory = drivetrain.trajectoryBuilder(whiteLinePose)
//                .strafeTo(new Vector2d(55, 0))
                .splineTo(new Vector2d(55, 0), 0)
                .build();



        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            tfod.setZoom(2.5, 1.78);
        }


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode, and watch thou robot fly!");
        telemetry.update();

        waitForStart();

        String TargetZone = "It didn't return anything";
        if (opModeIsActive()) {
            TargetZone = detectObjects();

            //driving out to the zone
            sleep(3000);

            TargetZone = detectObjects();
            if (TargetZone.equalsIgnoreCase("A")) {
                doZoneA();
            } else if (TargetZone.equalsIgnoreCase("B")) {
                doZoneB();
            } else if (TargetZone.equalsIgnoreCase("C")) {
                doZoneC();
            } else {
                telemetry.addData("[ERROR]", TargetZone);
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void initShooter() {
        shooter.setPower(1);
        sleep(2000);
    }

    private void shootRing() {
        double position = 0.3;
        while (position < 0.6) {
            position += 0.05;
            finger.setPosition(position);
        }

        sleep(1000);

        while (position > 0.2) {
            position -= 0.05;
            finger.setPosition(position);
        }
    }

    private void doZoneA(){
        drivetrain.followTrajectory(ATrajectory);
        drivetrain.turn(Math.toRadians(-90));
        // turn on the motors to dump the wobble goal
        intake.setPower(-.25);

        sleep(333);
        intake.setPower(0);

        drivetrain.followTrajectory(AReturnTrajectory);
        drivetrain.turn(Math.toRadians(88));

        initShooter();
        for (int i = 0; i < 4; i++) {
            shootRing();
            sleep(433);
        }

        drivetrain.followTrajectory(EndTrajectory);
    }

    private void doZoneB(){
//        drivetrain.followTrajectory(MissRingsTrajectory);
        drivetrain.followTrajectory(BTrajectory);
        drivetrain.turn(Math.toRadians(90));
        // turn on the motors to dump the wobble goal
        intake.setPower(-.25);

        sleep(333);
        intake.setPower(0);

        drivetrain.turn(Math.toRadians(-90));
        drivetrain.followTrajectory(BReturnTrajectory);
        drivetrain.turn(Math.toRadians(-16));

        initShooter();
        for (int i = 0; i < 4; i++) {
            shootRing();
            sleep(433);
        }

        drivetrain.followTrajectory(EndTrajectory);
    }

    private void doZoneC(){
//        drivetrain.followTrajectory(MissRingsTrajectory);
        drivetrain.followTrajectory(CTrajectory);
        drivetrain.turn(Math.toRadians(225));
        // turn on the motors to dump the wobble goal
        intake.setPower(-.25);

        sleep(333);
        intake.setPower(0);

        drivetrain.turn(Math.toRadians(135));
        drivetrain.followTrajectory(CReturnTrajectory);

        initShooter();
        for (int i = 0; i < 4; i++) {
            shootRing();
            sleep(433);
        }

        drivetrain.followTrajectory(EndTrajectory);
    }

    private String detectObjects() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 0) {
                    // empty list.  no objects recognized.

                    telemetry.addData("TFOD", "No items detected.");
                    telemetry.addData("Target Zone (Height)", "A");
                    return "A";
                } else {
                    // list is not empty.
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {

                        /*
                         * I chose the height in this way because
                         * 98 px: Single
                         * 196 px: Quad
                         * So 150 px is a good number to choose between the two.
                         */
                        if (recognition.getHeight() > 150) {
                            telemetry.addData("Target Zone (Height)", "C");
                            telemetry.addData("Height", recognition.getHeight());
                            return "C";
                        } else {
                            telemetry.addData("Target Zone (Height)", "B");
                            telemetry.addData("Height", recognition.getHeight());
                            return "B";
                        }
                    }
                }
                telemetry.update();
            }
            return "No changed recognitions";
        }
        return "TFOD is null";
    }
}
