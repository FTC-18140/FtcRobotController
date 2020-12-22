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

import com.qualcomm.hardware.bosch.BNO055IMU; // gyro
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import com.qualcomm.robotcore.hardware.DcMotor; // motors
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


import java.util.concurrent.TimeUnit;


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
    DcMotor LF = null;
    DcMotor RF = null;
    DcMotor LR = null;
    DcMotor RR = null;

    DcMotor intake = null;
    DcMotor rampMotor = null;
    DcMotor shooterMotor = null;
    Servo leftClaw = null;
    Servo rightClaw = null;

    static final double     COUNTS_PER_MOTOR_REV    = 28;      // goBuilda 5202 motors
    static final double     DRIVE_GEAR_REDUCTION    = 3;    // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    private long startTime = 0; // in nanoseconds
    private boolean moving;
    double NANOSECONDS_PER_SECOND = TimeUnit.SECONDS.toNanos(1);
    double initialHeading;
    double initialPosition;
    DcMotor encoderMotor;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    Telemetry telemetry         =  null;

    /* Gyro */
    BNO055IMU imu = null;


    /* Constructor */
    public Thunderbot()
    {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry telem) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        telemetry = telem;
        startTime = 0;

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


        try
        {
            RF = ahwMap.get(DcMotor.class, "rightFront");
            RF.setDirection(DcMotor.Direction.FORWARD);
            RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            telemetry.addData("rightFront mtr initialized", "");
        }
        catch(Exception p_exception)
        {
            telemetry.addData("rightFront not found in config file", "");
            RF = null;
        }
        try
        {
            RR = ahwMap.get(DcMotor.class, "rightRear");
            RR.setDirection(DcMotor.Direction.FORWARD);
            RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            telemetry.addData("rightRear mtr initialized", "");
        }
        catch (Exception p_exception)
        {
            telemetry.addData("rightRear not found in config file", "");
            RR = null;
        }
        try
        {
            LF = ahwMap.get(DcMotor.class, "leftFront");
            LF.setDirection(DcMotor.Direction.REVERSE);
            LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            telemetry.addData("leftFront mtr initialized", "");
        }
        catch (Exception p_exception)
        {
            telemetry.addData("leftFront not found in config file", "");
            LF = null;
        }
        try
        {
            LR = ahwMap.get(DcMotor.class, "leftRear");
            LR.setDirection(DcMotor.Direction.REVERSE);
            LR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            telemetry.addData("leftRear mtr initialized", "");
        }
        catch (Exception p_exception)
        {
            telemetry.addData("leftRear not found in config file","");
            LR = null;
        }

        //Intake Motors Hardware Mapping
        try {
            intake = ahwMap.dcMotor.get("intake");
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        catch (Exception p_exception)
        {
            intake = null;
            telemetry.addData("intake not found in config file", "");
        }
        try {
            rampMotor = ahwMap.dcMotor.get("rampMotor");
            rampMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        catch (Exception p_exception)
        {
            rampMotor = null;
            telemetry.addData("rampMotor not found in config file", "");
        }
        try {
            shooterMotor = ahwMap.dcMotor.get("shooterMotor");
            shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        catch (Exception p_exception)
        {
            shooterMotor = null;
            telemetry.addData("shooterMotor not found in config file", "");
        }

        //Servo Hardware Mapping
        try {
            leftClaw = ahwMap.servo.get("leftClaw");
            leftClaw.setPosition(0);
        }
        catch (Exception p_exception)
        {
            leftClaw = null;
            telemetry.addData("leftClaw not found in config file", "");
        }
        try {
            rightClaw = ahwMap.servo.get("rightClaw");
            rightClaw.setPosition(1);
        }
        catch (Exception p_exception)
        {
            rightClaw = null;
            telemetry.addData("rightClaw not found in config file", "");
        }

        moving = false;

    }

    public void joystickDrive(double leftStickX, double leftStickY, double rightStickX, double rightStickY, double powerLimit)
    {
        /*
            These are the calculations needed to make a simple mecaccnum drive.
              - The left joystick controls moving straight forward/backward and straight sideways.
              - The right joystick control turning.
        */
        double forward = leftStickY;
        double right = leftStickX;
        double clockwise = rightStickX;

        double leftFront = (forward + clockwise + right);
        double rightFront = (forward - clockwise - right);
        double leftRear = (forward + clockwise - right);
        double rightRear = (forward - clockwise + right);


        //Find the largest command value given and assign it to max.
        double max = 0.0;
        if (Math.abs(leftFront) > max)  { max = Math.abs(leftFront); }
        if (Math.abs(rightFront) > max) { max = Math.abs(rightFront); }
        if (Math.abs(leftRear) > max)   { max = Math.abs(leftRear); }
        if (Math.abs(rightRear) > max)  { max = Math.abs(rightRear); }

        //Set the minimum and maximum power allowed for drive moves and compare it to the parameter powerLimit.
        powerLimit = Range.clip(powerLimit, .05, 1);

        //If max still equals zero after checking all four motors, then set the max to 1
        if (max == 0.0)
        {
            max = 1;
        }

        // If max is greater than the power limit, divide all command values by max to ensure that all command
        // values stay below the magnitude of the power limit.
        if (max > powerLimit)
        {
            leftFront  = leftFront  / max * powerLimit;
            rightFront = rightFront / max * powerLimit;
            leftRear   = leftRear   / max * powerLimit;
            rightRear  = rightRear  / max * powerLimit;
        }

        if(motorsValid())
        {
            RF.setPower(rightFront);
            RR.setPower(rightRear);
            LF.setPower(leftFront);
            LR.setPower(leftRear);
        }
        else
        {
            telemetry.addData("A drive motor is not configured properly", "");
        }

    }

    /**
     * Chassis - Drives the four drive motors on the robot and will move the robot forward,
     * backward, left, or right.
     *
     * @param power  How fast the robot will drive.
     * @param direction  In which direction the robot will drive (forward, backward, left, right,
     *                   or 45 degrees in any direction).
     * @param distance  How far the robot will drive.
     * @param time  The max time this move can take. A time-out feature: if the move stalls for some
     *              reason, the timer will catch it.
     * @return  A boolean that tells us whether or not the robot is moving.
     */
    public boolean drive(double power, double direction, double distance, double time)
    {
        double driveDistance = COUNTS_PER_INCH * distance;
        double correction;

        double actual = updateHeading();

        if (!moving)
        {
            setZeroBehavior("BRAKE");

            initialHeading = updateHeading();
            if (Math.abs(initialHeading) > 130  &&  initialHeading < 0.0)
            {
                initialHeading += 360.0;
            }
            else
            {
                initialPosition = LF.getCurrentPosition(); //.getCurrentPosition();
                encoderMotor = LF;
            }
            resetStartTime();
            moving = true;
        }
        telemetry.addData("initial position: ", initialPosition);

        if (Math.abs(initialHeading) > 130  &&  actual < 0.0)
        {
            actual += 360;
        }

        correction = ((initialHeading - actual) * .03);

        double lStickX = power * Math.sin(Math.toRadians(direction));
        double lStickY = -(power * Math.cos(Math.toRadians(direction)));

//        telemetry.addData("Right Motor DD: ", Math.abs(rFrontMotor.getCurrentPosition() - initialPosition));
//        telemetry.addData("Left Motor DD:", Math.abs(lFrontMotor.getCurrentPosition() - initialPosition));

        telemetry.addData("Drive Distance: ", driveDistance);
        double tmpDistance = Math.abs(encoderMotor.getCurrentPosition() - initialPosition);
        telemetry.addData("Distance Driven:", tmpDistance);
        telemetry.addData("getRuntime() = ", getRuntime());
//        telemetry.addData("time = ", time);

        joystickDrive(lStickX, -lStickY, correction, 0.0, power);

//        if (((Math.abs(bulkData.getMotorCurrentPosition(encoderMotor)- initialPosition)) >= driveDistance) || (getRuntime() > time))
        if (((Math.abs(encoderMotor.getCurrentPosition() - initialPosition)) >= driveDistance) || (getRuntime() > time))
        {
            stopChassis();

            setZeroBehavior("FLOAT");

            encoderMotor = LF;
            moving = false;
        }

        return !moving;
    }

    /**
     * Chassis - The pointTurn method turns the robot to a target heading, automatically picking the turn
     * direction that is the shortest distance to turn to arrive at the target.
     *
     * @param targetHeading  The direction in which the robot will move.
     * @param time  The maximum time the move can take before the code moves on.
     * @param power  The power at which the robot will move.
     * @return A boolean that tells use whether or not the robot is moving.
     */
    public boolean pointTurn(double power, double targetHeading, double time)
    {
        power = Math.abs(power);
        double currentRawHeading = updateHeading();
        double error = 1;
        double directionalPower = 1;


        if (Math.abs(targetHeading) > 170 && currentRawHeading < 0.0)
        {
            currentRawHeading += 360;
        }

        if (!moving)
        {
            setZeroBehavior("BRAKE");

            initialHeading = currentRawHeading;
            error = targetHeading - initialHeading;

            if (error > 180)
            {
                error -= 360;
            }
            else if (error < -180)
            {
                error += 360;
            }

            if (error < 0)
            {
                directionalPower = -power;
            }
            else
            {
                directionalPower = power;
            }

            if ( Math.abs(error) < 60 )
            {
                directionalPower = error * 0.03;
                if (directionalPower > 0 )
                {
                    directionalPower = Range.clip( directionalPower, 0.25, power);
                }
                else
                {
                    directionalPower = Range.clip(directionalPower, -power, -0.25);
                }
            }

            resetStartTime();
            moving = true;
        }
        telemetry.addData("error: ", error);
        telemetry.addData("directionalPower: ", directionalPower);

        joystickDrive(0.0, 0.0, directionalPower, 0.0, power);

        if(Math.abs(targetHeading - currentRawHeading) < 3.0 || getRuntime() > time)
        {
            stopChassis();

            setZeroBehavior("FLOAT");

            moving = false;
        }

        return !moving;
    }

    public double updateHeading()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    /**
     * Chassis - check to ensure that all of the chassis motors are initialized properly.
     * @return boolean indicating good init (true) or not (false)
     */
    private boolean motorsValid()
    {
        boolean done = false;
        if(RF != null)
        {
            if(RR != null)
            {
                if(LF != null)
                {
                    if (LR != null)
                    {
                        done = true;
                    }
                }
            }
        }
        return done;
    }

    /** Stops the drive train. */
    public void stopChassis()
    {
        if(motorsValid())
        {
            RF.setPower(0.0);
            RR.setPower(0.0);
            LF.setPower(0.0);
            LR.setPower(0.0);
        }
    }

    public void stopIntake()
    {
        intake.setPower(0.0);
        rampMotor.setPower(0.0);
        shooterMotor.setPower(0.0);

    }

    /**
     * Robot - Reset the internal timer to zero.
     */
    private void resetStartTime()
    {
        startTime = System.nanoTime();
    }

    /**
     * Robot - Get the number of seconds this op mode has been running
     * <p>
     * This method has sub millisecond accuracy.
     * @return number of seconds this op mode has been running
     */
    private double getRuntime()
    {
        return (System.nanoTime() - startTime) / NANOSECONDS_PER_SECOND;
    }

    /**
     * Chassis - Tell the chassis motors what to do when power is set to 0
     * @param mode - which indicates the desired behavior
     */
    public void setZeroBehavior(String mode)
    {
        if(mode.equalsIgnoreCase("FLOAT"))
        {
            LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        else if(mode.equalsIgnoreCase("BRAKE"))
        {
            LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }



}
