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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; // gyro
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Thunderbot: Auto Drive By Encoder", group="Thunderbot")
//@Disabled
public class ThunderbotAutoDriveByEncoder_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    Thunderbot robot = new Thunderbot();   // Use a Thunderbot's hardware

    static final double DRIVE_SPEED = 0.1;
    static final double TURN_SPEED = 0.2;


    @Override
    public void runOpMode() {

        //Initialize the drive system variables.
        robot.init(hardwareMap, telemetry);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)
            robot.driveStraight(DRIVE_SPEED, -5, 48, this);  // S1: Forward 47 Inches with 5 Sec timeout
            //robot.pointTurn(TURN_SPEED, 48, 48, this);  // S2: Turn Right 12 Inches with 4 Sec timeout
            //robot.driveStraight(DRIVE_SPEED, -50, 50, this);  // S3: Reverse 24 Inches with 4 Sec timeout

           robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //turn 90 degrees
            while(opModeIsActive() && robot.angles.firstAngle < -90) { // degrees
                robot.leftFront.setPower(TURN_SPEED);// power
                robot.rightFront.setPower(-TURN_SPEED);
                robot.leftRear.setPower(TURN_SPEED);
                robot.rightRear.setPower(-TURN_SPEED);
            }

            telemetry.addData("Path", "Complete");
            telemetry.update();

            robot.stop();
        }
    }
}
