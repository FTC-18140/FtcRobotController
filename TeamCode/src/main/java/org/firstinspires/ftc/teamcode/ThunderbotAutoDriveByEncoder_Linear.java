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
        robot.init(hardwareMap, telemetry);

        // Wait for the game to start (driver presses PLAY)ds
        waitForStart();

        // Note: use sleep when you want the robot to stop for a selected time
        while (opModeIsActive()){
            robot.shooterMotor.setPower(0.64); // Start up shooterMotors
            robot.shooterMotor2.setPower(0.64);

            robot.gyroDriveForward(64, 0.5); // Go forward 70 inches to line up on the shooting line


            robot.strafeLeft(10, 0.4); // Strafe left 10 inches in order to line up the robot to fire the rings

            sleep(2000); // Wait 4 secs to allow the rings to reach full power
                                    // Note: this time will be able to be reduced if needed
            robot.shooterServo1.setPower(-1.0); // Move rings into shooterMotors to fire rings
            robot.shooterServo2.setPower(-1.0);
            robot.rampIntakeServo.setPower(-0.5);
            sleep(1000); // Wait 3 secs to allow all the rings to fire
            robot.shooterServo1.setPower(0);
            robot.shooterServo2.setPower(0);


            robot.strafeLeft(8, 0.4); // Strafe left 10 inches in order to line up the robot to fire the rings

            sleep(2000); // Wait 4 secs to allow the rings to reach full power
            // Note: this time will be able to be reduced if needed
            robot.shooterServo1.setPower(-1.0); // Move rings into shooterMotors to fire rings
            robot.shooterServo2.setPower(-1.0);
            sleep(2000); // Wait 3 secs to allow all the rings to fire


            robot.strafeRight(40, 0.4);

            robot.shooterMotor.setPower(0); // Turn off shooterMotors and shooterServos to conserve power
            robot.shooterMotor2.setPower(0);
            robot.shooterServo1.setPower(0);
            robot.shooterServo2.setPower(0);
            robot.rampIntakeServo.setPower(0);

            robot.gyroDriveForward(20, 0.5); // Go forward 15 inches into the square B

            robot.wobbleDrop(0.7); // Drop the wobble goal in the square B

            robot.strafeRight(18, 0.4); // Strafe right 18 inches to avoid the placed rings

            robot.gyroDriveBackward(28, 0.4); // Drive backwards 28 inches to get closer to the set wobble goal

            robot.gyroTurn(160, 0.2); // to 160 degrees in order to line up the wobbleArm with the wobble goal
                                                          // Note: If the robot is set off course this turn may cause the robot to spin

            robot.gyroDriveForward(16, 0.2); // Move forward 16 inches to grab the wobble

            robot.leftClaw.setPosition(0); // Grab the wobble and hold until program ends
            robot.rightClaw.setPosition(1);
            sleep(1000); // Wait 1 secs to give enough time to grab the wobble

            robot.gyroTurn(24, 0.1); // turn 24 degrees left aiming for the shooter line and square A

            robot.gyroDriveBackward(35, 0.5); // Drive backwards 35 inches to the shooter line and square A

            robot.gyroTurn(80, 0.2); // Turn 80 degrees left in order to drop the wobble in the middle of square A

            break;
        }

            telemetry.addData("Path", "Complete");
            telemetry.update();

    }
}
