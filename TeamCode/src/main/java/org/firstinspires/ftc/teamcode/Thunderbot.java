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
 */

public class Thunderbot
{
    /* Public OpMode members. */
    // Chassis drive motors
    DcMotor LF = null;
    DcMotor RF = null;
    DcMotor LR = null;
    DcMotor RR = null;

    // Mechanism motors
    DcMotor intake = null;
    DcMotor rampMotor = null;
    DcMotor shooterMotor = null;
    // Claw servos
    Servo leftClaw = null;
    Servo rightClaw = null;

    // Variables used to calculate encoder ticks per wheel rotation
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


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry telem)
    {
        // Save reference to Hardware map
        hwMap = ahwMap;
        telemetry = telem;
        startTime = 0;

        // Initialize the Gryo
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


        // Initialize Right-Front motor
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
        // Initialize Right-Rear motor
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
        // Initialize Left-Front motor
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
        // Initialize Left-Rear motor
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

        // Initialize Intake motor
        try
        {
            intake = ahwMap.dcMotor.get("intake");
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        catch (Exception p_exception)
        {
            intake = null;
            telemetry.addData("intake not found in config file", "");
        }
        // Initialize Ramp motor
        try
        {
            rampMotor = ahwMap.dcMotor.get("rampMotor");
            rampMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        catch (Exception p_exception)
        {
            rampMotor = null;
            telemetry.addData("rampMotor not found in config file", "");
        }
        // Initialize Shooter motor
        try
        {
            shooterMotor = ahwMap.dcMotor.get("shooterMotor");
            shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        catch (Exception p_exception)
        {
            shooterMotor = null;
            telemetry.addData("shooterMotor not found in config file", "");
        }
        // Initialize left-Claw servo
        try
        {
            leftClaw = ahwMap.servo.get("leftClaw");
            leftClaw.setPosition(0);
        }
        catch (Exception p_exception)
        {
            leftClaw = null;
            telemetry.addData("leftClaw not found in config file", "");
        }
        // Initialize right-Claw servo
        try
        {
            rightClaw = ahwMap.servo.get("rightClaw");
            rightClaw.setPosition(1);
        }
        catch (Exception p_exception)
        {
            rightClaw = null;
            telemetry.addData("rightClaw not found in config file", "");
        }

        // Used to indicate if the robot is actively moving or not
        moving = false;

    }

    /**
     * The most basic motor control method -- all methods that command the chassis use joystickDrive to do so
     * Converts joystick input to power and directional motor commands for a mechanum chassis.
     * @param leftStickX  x-axis on the left joystick
     * @param leftStickY  y-axis on the right joystick
     * @param rightStickX  x-axis on the left joystick
     * @param rightStickY  y-axis on the right joystick
     * @param powerLimit  The max power the chassis can use
     */
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
     * @param direction  In which direction the robot will drive (forward, backward, left, or right).
     * @param distance  How far the robot will drive (in).
     * @param time  The max time this move can take. A time-out feature: if the move stalls for some
     *              reason, the timer will catch it.
     * @return  A boolean that tells us whether or not the robot is moving.
     */
    public boolean drive(double power, double direction, double distance, double time)
    {
        // Converting inches of travel to encoder ticks
        double driveDistance = COUNTS_PER_INCH * distance;
        // Represents the difference between the robot's current heading and the target heading
        double correction;

        double actual = updateHeading();

        // Executes once at the beginning of the move
        if (!moving)
        {
            // Setting motors to "Brake" mode
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

        if (Math.abs(initialHeading) > 130  &&  actual < 0.0)
        {
            actual += 360;
        }

        correction = ((initialHeading - actual) * .03);

        double lStickX = power * Math.sin(Math.toRadians(direction));
        double lStickY = -(power * Math.cos(Math.toRadians(direction)));

        joystickDrive(lStickX, -lStickY, correction, 0.0, power);

        if (((Math.abs(encoderMotor.getCurrentPosition() - initialPosition)) >= driveDistance) || (getRuntime() > time))
        {
            // Stopping the drive motors
            stopChassis();
            // Setting motors to "Float"
            setZeroBehavior("FLOAT");
            // Indicating that the robot has stopped moving
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

        // Executes once at the beginning of the turn
        if (!moving)
        {
            setZeroBehavior("BRAKE");

            // Capturing the robot's start heading
            initialHeading = currentRawHeading;
            // Calculating the error between the current heading and the target heading
            error = targetHeading - initialHeading;

            // Adding or subtracting 360 degrees to the error to keep the gyro from flipping directions
            if (error > 180)
            {
                error -= 360;
            }
            else if (error < -180)
            {
                error += 360;
            }
            // Setting the direction of the motors depending on the turn direction
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

        joystickDrive(0.0, 0.0, directionalPower, 0.0, power);

        if(Math.abs(targetHeading - currentRawHeading) < 3.0 || getRuntime() > time)
        {
            stopChassis();

            setZeroBehavior("FLOAT");

            moving = false;
        }

        return !moving;
    }


    // Returns the current robot heading
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

    // Stops the mechanism motors
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
