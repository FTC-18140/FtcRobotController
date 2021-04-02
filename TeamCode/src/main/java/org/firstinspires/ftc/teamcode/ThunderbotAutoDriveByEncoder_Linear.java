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
            robot.shooterMotor.setPower(0.7); // Start up shooterMotors
            robot.shooterMotor2.setPower(0.7);

            robot.gyroDriveForward(64, 0.5); // Go forward 70 inches

            robot.strafeRight(10, 0.4);

            sleep(4000); // Wait 4 secs
            robot.shooterServo1.setPower(-1.0); // Move rings into shooterMotors
            robot.shooterServo2.setPower(-1.0);
            sleep(3000); // Wait 3 secs

            robot.strafeRight(10, 0.4);

            robot.shooterMotor.setPower(0); // Turn off shooterMotors
            robot.shooterMotor2.setPower(0);
            robot.shooterServo1.setPower(0); // Turn off shooterServos
            robot.shooterServo2.setPower(0);

            robot.gyroDriveForward(20, 0.5); // Go forward 15 inches

            robot.wobbleDrop(0.7); // drop the wobble goal

            robot.strafeRight(18, 0.4);

            robot.gyroDriveBackward(28, 0.4); // 50 0.5

            robot.gyroTurn(160, 0.2);

            robot.gyroDriveForward(16, 0.2);

            robot.leftClaw.setPosition(0);
            robot.rightClaw.setPosition(1);

            sleep(1000);

            robot.gyroTurn(24, 0.1); // x < 17 no

            robot.gyroDriveBackward(35, 0.5);

            robot.gyroTurn(80, 0.2);

            break;
        }

            telemetry.addData("Path", "Complete");
            telemetry.update();

    }
}
