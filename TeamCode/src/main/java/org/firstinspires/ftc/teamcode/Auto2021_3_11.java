
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Autonomous(name="Autonomous 2021(March Version)")

public class Auto2021_3_11 extends LinearOpMode{

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


        private DcMotor backLeft;
        private DcMotor backRight;
        private DcMotor frontLeft;
        private DcMotor frontRight;
        private Servo finger;
        private DcMotorEx shooter;


    //@Override
        public void runOpMode(){
            SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
            intake = hardwareMap.get(DcMotor.class, "Intake");

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
            telemetry.addData(">", "Press Play to start op mode");
            telemetry.update();

            waitForStart();

            String TargetZone="A";
            if (opModeIsActive()) {

                Trajectory ATrajectory = drivetrain.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(160)))
                        .strafeTo(new Vector2d(60,16))
                        .build();
                Trajectory BTrajectory = drivetrain.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(160)))
                        .strafeTo(new Vector2d(80,16))
                        .build();
                Trajectory CTrajectory = drivetrain.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(160)))
                        .strafeTo(new Vector2d(101,16))
                        .build();
                Trajectory AReturnTrajectory = drivetrain.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(160)))
                        .strafeTo(new Vector2d(12,-2))
                        .build();
                Trajectory BReturnTrajectory = drivetrain.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(160)))
                        .strafeTo(new Vector2d(-16,4))
                        .build();
                Trajectory CReturnTrajectory = drivetrain.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(160)))
                        .strafeTo(new Vector2d(-38,0))
                        .build();

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 0 ) {
                            // empty list.  no objects recognized.

                            telemetry.addData("TFOD", "No items detected.");
                        } else {
                            // list is not empty.
                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            double maxSize = 0;

                            for (Recognition recognition : updatedRecognitions) {

                                /*
                                 * I chose the height in this way because
                                 * 98 px: Single
                                 * 196 px: Quad
                                 * So 120 px is a good number to choose between the two.
                                 */
                                if (recognition.getHeight() > maxSize) {
                                    maxSize = recognition.getHeight();
                                }
                            }
                            if (maxSize > 120) {
                                TargetZone = "C";
                            }else{
                                TargetZone = "B";
                            }
                            telemetry.addData("Max", maxSize);
                        }
                        telemetry.addData("Target Zone (Height)", TargetZone);
                        telemetry.update();
                    }
                }

                //driving out to the zone
                if (TargetZone.equalsIgnoreCase("A")) {
                    drivetrain.followTrajectory(ATrajectory);
                    drivetrain.turn(Math.toRadians(-90));
                }else if (TargetZone.equalsIgnoreCase("B")){
                    drivetrain.followTrajectory(BTrajectory);
                    drivetrain.turn(Math.toRadians(90));
                }else{
                    drivetrain.followTrajectory(CTrajectory);
                    drivetrain.turn(Math.toRadians(225));
                }

                // turn on the motors to dump the wobble goal
                intake.setPower(-.25);

                sleep(333);
                intake.setPower(0);

                //drives to the white line
                if (TargetZone.equalsIgnoreCase("A")){
                    drivetrain.followTrajectory(AReturnTrajectory);
                }else if (TargetZone.equalsIgnoreCase("B")){
                    drivetrain.turn(Math.toRadians(-90));
                    drivetrain.followTrajectory(BReturnTrajectory);
                }else{
                    drivetrain.turn(Math.toRadians(135));
                    drivetrain.followTrajectory(CReturnTrajectory);
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
    }

