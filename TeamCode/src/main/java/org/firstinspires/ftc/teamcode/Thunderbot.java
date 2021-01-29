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

import android.os.AsyncTask;

import com.qualcomm.hardware.bosch.BNO055IMU; // gyro
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import com.qualcomm.robotcore.hardware.DcMotor; // motors
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Thunderbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 */

public class Thunderbot
{
    /* Public OpMode members. */
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftRear = null;
    DcMotor rightRear = null;

    static final double     COUNTS_PER_MOTOR_REV    = 28;      // goBuilda 5202 motors
    static final double     DRIVE_GEAR_REDUCTION    = 3;    // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    Telemetry telemetry         =  null;
    private ElapsedTime runtime  = new ElapsedTime();

    /* Gyro */
    BNO055IMU imu = null;
    Orientation angles = null;


    /* Constructor */
    public Thunderbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry telem) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        telemetry = telem;

        try
        {
            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU.
            imu = ahwMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        }
        catch (Exception p_exeception)
        {
            telemetry.addData("imu not found in config file", 0);
            imu = null;
        }

        //Define & Initialize Motors
        rightFront = hwMap.dcMotor.get("rightFront");
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightRear = hwMap.dcMotor.get("rightRear");
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront = hwMap.dcMotor.get("leftFront");
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear = hwMap.dcMotor.get("leftRear");
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Hardware Initialized");
        telemetry.addData("Status", "Encoders Reset");
        telemetry.addData("Status", "Thunderbot Ready");
        telemetry.update();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    //method for Driving from a Current Position to a Requested Position
    public void driveStraight(double speed, double distance, double timeoutS, LinearOpMode caller)
    {

        driveToPos( speed, distance, distance, timeoutS, caller);

    }

    public void pointTurn(double speed, double distance, double timeoutS, LinearOpMode caller)
    {
        driveToPos( speed, distance, -distance, timeoutS, caller);
    }


    public void driveToPos(double speed, double lDistance, double rDistance, double timeoutS, LinearOpMode caller )
    {
        int newLeftTarget;
        int newRightTarget;

        newLeftTarget = leftRear.getCurrentPosition() + (int)(lDistance * COUNTS_PER_INCH);
        newRightTarget = rightRear.getCurrentPosition() + (int)(rDistance * COUNTS_PER_INCH);
        leftFront.setTargetPosition(newLeftTarget);
        rightFront.setTargetPosition(newRightTarget);
        leftRear.setTargetPosition(newLeftTarget);
        rightRear.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        leftFront.setPower(Math.abs(speed));
        rightFront.setPower(Math.abs(speed));
        leftRear.setPower(Math.abs(speed));
        rightRear.setPower(Math.abs(speed));

        while (caller.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                isBusy() )
        {

            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
            telemetry.addData("Path2",  "Running at %7d :%7d",
                    leftFront.getCurrentPosition(),
                    rightFront.getCurrentPosition(),
                    leftRear.getCurrentPosition(),
                    rightRear.getCurrentPosition());
            telemetry.update();
        }


        // Stop the robot because it is done with teh move.
        stop();

        // Turn off RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public boolean isBusy()
    {
        return leftFront.isBusy() && rightFront.isBusy()  && leftRear.isBusy() && rightRear.isBusy();
    }

    public double updateHeading()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    public void gyroTurn(double targetHeading, double power)
    {
        double currentAngle = updateHeading();
        double startAngle = currentAngle;
        while(Math.abs(currentAngle-startAngle) < targetHeading) { // degrees
            leftFront.setPower(power);// power
            rightFront.setPower(-power);
            leftRear.setPower(power);
            rightRear.setPower(-power);
            currentAngle = updateHeading();
        }

            //telemetry for turning in degrees
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Heading: ",angles.firstAngle);
            telemetry.update();

        stop();
    }

    public void driveStraight (int duration, double power) throws InterruptedException{
        double leftFrontSpeed;
        double rightFrontSpeed;
        double leftRearSpeed;
        double rightRearSpeed;

        double target = mrGyro.getIntegratedValue;
        double startPosition = MLeft.getCurrentPosition();

        while (MLeft.getCurrentPosition() < duration + startPosition){
            int zAccumulated = mrGyro.getIntegratedValue;

            leftFrontSpeed = power + (zAccumulated - target)/100;
            rightFrontSpeed  = power - (zAccumulated - target)/100;
            leftRearSpeed  = power + (zAccumulated - target)/100;
            rightRearSpeed  = power - (zAccumulated - target)/100;

            leftFrontSpeed = Range.clip(leftFrontSpeed -1, 1);
            rightFrontSpeed = Range.clip(rightFrontSpeed -1, 1);
            leftRearSpeed = Range.clip(leftRearSpeed -1, 1);
            rightRearSpeed = Range.clip(rightRearSpeed -1, 1);

            MLeft.setPower(leftFrontSpeed);
            MLeft.setPower(leftRearSpeed);
            MRight.setPower(rightFrontSpeed);
            MRight.setPower(rightRearSpeed);

            telemetry.addData("1. Left", MLeft.getPower());
            telemetry.addData("2. Right", MRight.getPower());
            telemetry.addData("3. Distance to go", duration + startPosition - MLeft.getCurrentPosition());

            waitOneFullHardwareCycle();
        }
    }

    public void stop()
    {
        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
}
