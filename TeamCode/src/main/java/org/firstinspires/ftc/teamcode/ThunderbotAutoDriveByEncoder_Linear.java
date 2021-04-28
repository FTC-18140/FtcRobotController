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


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;
import java.util.concurrent.TimeUnit;

import static java.lang.Thread.sleep;


// slow down some parts of the code and try to lower some cool downs
// add a sensor  to increase accuracy (light sensor, gyro for strafe, voltage, etc.)

@Autonomous(name="Thunderbot: Auto Drive By Encoder", group="Thunderbot")
//@Disabled
public class ThunderbotAutoDriveByEncoder_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    Thunderbot robot = new Thunderbot();   // Use a Thunderbots hardware
    int state = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the drive system variables.
        robot.initVuforia();
        robot.initTfod();
        robot.init(hardwareMap, telemetry);

        if (robot.tfod != null) {
            robot.tfod.activate();
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            robot.tfod.setZoom(1.0, 16.0/9.0);
        }

        robot.checkRings();

        // Wait for the game to start (driver presses PLAY)ds
        waitForStart();

        // Note: use sleep when you want the robot to stop for a selected time
        while (opModeIsActive()){
            if (robot.tfod != null) {
                robot.tfod.shutdown();
            }
            robot.shooterMotor.setPower(0.60); // Start up shooterMotors
            robot.shooterMotor2.setPower(0.60);

            robot.gyroDriveForward(58, 0.6); // Go forward 70 inches to line up on the shooting line (could change)
            robot.gyroDriveToLine (120, 0.2 );

            robot.lineFollowLeft(190, 14, 0.5); // Strafe left 10 inches in order to line up the robot to fire the rings

            sleep(2000); // Wait 2 secs to allow the rings to reach full power
            robot.shooterServo1.setPower(-1.0); // Move rings into shooterMotors to fire rings
            robot.shooterServo2.setPower(-1.0);
            robot.rampIntakeServo.setPower(-0.5);
            sleep(1000); // Wait 3 secs to allow all the rings to fire
            robot.shooterServo1.setPower(0);
            robot.shooterServo2.setPower(0);

            robot.lineFollowLeft(190, 6, 0.5); // Strafe left 10 inches in order to line up the robot to fire the rings

            // Note: this time will be able to be reduced if needed
            robot.shooterServo1.setPower(-1.0); // Move rings into shooterMotors to fire rings
            robot.shooterServo2.setPower(-1.0);
            sleep(2500); // Wait 3 secs to allow all the rings to fire

            robot.lineFollowRight(190, 39, 0.4);

            robot.shooterMotor.setPower(0); // Turn off shooterMotors and shooterServos to conserve power
            robot.shooterMotor2.setPower(0);
            robot.shooterServo1.setPower(0);
            robot.shooterServo2.setPower(0);
            robot.rampIntakeServo.setPower(0);

            robot.gyroDriveForward(20, 0.5); // Go forward 15 inches into the square B

            robot.wobbleDrop(0.7); // Drop the wobble goal in the square B

            robot.gyroTurn(73, -0.2); // turn 90
            robot.strafeRight(94, 0.7); // strafe into wall   // changed from 0.6
            robot.gyroDriveForward(7, 0.5);
            robot.strafeLeftToObject(3, 20, 0.1); // look for and grab wobble
            robot.gyroDriveToLine (110, 0.2 );
            robot.strafeLeft(70, 0.5); // get to the white line

            break;
        }

            telemetry.addData("Path", "Complete");
            telemetry.update();

    }
}
