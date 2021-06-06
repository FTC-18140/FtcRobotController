package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Autonomous(name="Start_RR: ", group="Start_RR")
//@Disabled
public class RR extends LinearOpMode {

    /* Declare OpMode members. */
    Thunderbot robot = new Thunderbot();   // Use a Thunderbots hardware

    /** Target zone methods for Red Right (RR) */

    public void powerShot_RR() throws InterruptedException {
        robot.shooterMotor.setPower(0.6); // Start up shooterMotors
        robot.shooterMotor2.setPower(0.6);

        robot.gyroDriveToLine(190,0.2); // Get back on the line

        robot.lineFollowLeft(300, 50, 0.4); // Line up for power shots

        robot.gyroDriveToLine(190,0.2); // Get back on the line

        sleep(2000); // Wait 2 secs to allow the rings to reach full power
        robot.shooterServo1.setPower(-1.0); // Move rings into shooterMotors to fire rings
        robot.shooterServo2.setPower(-1.0);
        robot.rampIntakeServo.setPower(-0.5);
        sleep(1000); // Wait 3 secs to allow all the rings to fire
        robot.shooterServo1.setPower(0);
        robot.shooterServo2.setPower(0);

        robot.lineFollowLeft(300, 8, 0.5); // Strafe left in order to line up the robot to fire the rings

        robot.shooterServo1.setPower(-1.0); // Move rings into shooterMotors to fire rings
        robot.shooterServo2.setPower(-1.0);
        robot.shooterMotor.setPower(0.6);
        robot.shooterMotor2.setPower(0.6);
        sleep(3000); // Wait 3 secs to allow all the rings to fire

    }


    public void targetZoneA_RR() throws InterruptedException {
        robot.gyroDriveForward(58, 0.6); // Go forward to line up on the shooting line
        robot.gyroDriveToLine(120, 0.2);

        robot.gyroTurn(85, -0.2); // turn 90

        robot.wobbleDrop(0.7); // Drop the wobble in target zone A
        sleep(250);

        robot.armMotor.setPower(-0.7); // Raise the wobble arm
        sleep(1000);
        robot.armMotor.setPower(0);

        robot.gyroTurn(85, 0.2); // turn 90
    }


    public void targetZoneB_RR() throws InterruptedException {
        robot.gyroDriveForward(58, 0.6); // Go forward to line up on the shooting line
        robot.gyroDriveToLine(120, 0.2);

        robot.gyroDriveForward(20, 0.5); // Go forward into target zone B

        robot.wobbleDrop(0.7); // Drop the wobble goal into target zone B
        sleep(250);

        robot.armMotor.setPower(-0.7); // Raise the wobble arm
        sleep(1000);
        robot.armMotor.setPower(0);

        robot.gyroDriveBackward(19, 0.5); // Get back to the white line
        robot.gyroDriveToLine(190, 0.2);
    }


    public void targetZoneC_RR() throws InterruptedException {
        robot.gyroDriveForward(58, 0.6); // Go forward to line up on the shooting line
        robot.gyroDriveToLine(120, 0.2);

        robot.lineFollowRight(300, 15, 0.4); // Hit the wall

        robot.gyroDriveForward(28, 1.0); // Go forward high power to square C
        robot.gyroDriveToLine(100, 0.2); // Drive to blue line

        robot.wobbleDrop(0.7); // Drop the wobble goal in the square C
        sleep(250);

        robot.armMotor.setPower(-0.7); // Raise the wobble arm
        sleep(1000);
        robot.armMotor.setPower(0);

        robot.driveBackwards(38, 1.0); // Drive back past the white line
        sleep(500);
        robot.gyroDriveToLine(120, 0.2); // line up on the white line

        robot.lineFollowLeft(300, 15, 0.4); // Get to target zone B end point
    }


    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the drive system variables.
        robot.initVuforia(hardwareMap);
        robot.initTfod(hardwareMap);
        robot.init(hardwareMap, telemetry);

        if (robot.tfod != null) {
            robot.tfod.activate();
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            robot.tfod.setZoom(2.0, 16.0/9.0);
        }

        //robot.checkRings();
        while (!opModeIsActive()) { // fixed version of checkRings method
            if (robot.tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();

                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    if (updatedRecognitions.size() == 0) {
                        // empty list.  no objects recognized.
                        telemetry.addData("TFOD", "No items detected.");
                        telemetry.addData("Target Zone", "A");
                        robot.nOfSetRings = 0;
                    } else {
                        // list is not empty.
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());

                            // check label to see which target zone to go after.
                            if (recognition.getLabel().equals("Single")) {
                                telemetry.addData("Target Zone", "B");
                                robot.nOfSetRings = 1;
                            } else if (recognition.getLabel().equals("Quad")) {
                                telemetry.addData("Target Zone", "C");
                                robot.nOfSetRings = 4;
                            } else {
                                telemetry.addData("Target Zone", "UNKNOWN");
                            }
                        }
                    }
                }
                telemetry.update();
            }
        }

        // Wait for the game to start (driver presses PLAY)ds
        waitForStart();

        // Note: use sleep when you want the robot to stop for a selected time
        while (opModeIsActive()){

            if (robot.tfod != null) {
                robot.tfod.shutdown();
            }

            if (robot.nOfSetRings == 0){
                // Square A
                targetZoneA_RR();
                powerShot_RR();
            } else if (robot.nOfSetRings == 1) {
                // Square B
                targetZoneB_RR();
                powerShot_RR();
            } else if (robot.nOfSetRings == 4){
                // Square C
                targetZoneC_RR();
                powerShot_RR();
            } else {
                // Just shoot power shots
                powerShot_RR();
            }
            break;
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }
}