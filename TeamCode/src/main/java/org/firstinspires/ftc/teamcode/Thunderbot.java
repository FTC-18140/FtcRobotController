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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static java.lang.Thread.sleep;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Thunderbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 */

public class Thunderbot
{
    /** Public OpMode members. */
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftRear = null;
    DcMotor rightRear = null;

    DcMotor armMotor = null;

    DcMotor intake = null;
    CRServo intakeServo = null;
    CRServo intakeServoTwo = null;
    CRServo shooterServo1 = null;
    CRServo shooterServo2 = null;
    Servo rampServo = null;
    DcMotor shooterMotor = null;
    DcMotor shooterMotor2 = null;

    Servo leftClaw = null;
    Servo rightClaw = null;

    TouchSensor touchSensor1 = null;
    TouchSensor touchSensor2 = null;
    ColorSensor leftColor = null;
    ColorSensor rightColor = null;

    // For state machines
    enum autoStates {
        Right,
        Left,
        KeepGoing,
        Grab
    }

    enum autoIfStateFails {

    }

    // converts inches to motor ticks
    static final double     COUNTS_PER_MOTOR_REV    = 28; // rev robotics hd hex motors planetary 411600
    static final double     DRIVE_GEAR_REDUCTION    = 20; // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0; // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


    /** local OpMode members. */
    HardwareMap hwMap =  null;
    private Telemetry telemetry;
    private ElapsedTime runtime  = new ElapsedTime();

    /** Gyro */
    BNO055IMU imu = null;
    Orientation angles = null;

    static double gyStartAngle = 0; // a shared gyro start position this will be updated using updateHeading()


    /** Constructor */
    public Thunderbot(){

    }

    /** Initialize standard Hardware interfaces */
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
        catch (Exception p_exeception) {
            telemetry.addData("imu not found in config file", 0);
            imu = null;
        }

        // Define & Initialize Motors
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

        armMotor = hwMap.dcMotor.get("armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor = hwMap.dcMotor.get("shooterMotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor2 = hwMap.dcMotor.get("shooterMotor2");
        shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hwMap.dcMotor.get("intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define & Initialize Servos
        leftClaw = hwMap.servo.get("leftClaw");
        rightClaw = hwMap.servo.get("rightClaw");
        intakeServo = hwMap.crservo.get("intakeServo");
        intakeServoTwo = hwMap.crservo.get("intakeServoTwo");
        shooterServo1 = hwMap.crservo.get("shooterServo1");
        shooterServo2 = hwMap.crservo.get("shooterServo2");
        //rampServo = hwMap.servo.get("rampServo");

        //  Define & Initialize Sensors
        touchSensor1 = hwMap.touchSensor.get("touchSensor1");
        touchSensor2 = hwMap.touchSensor.get("touchSensor2");
        leftColor = hwMap.colorSensor.get("rightColor"); // Note: swapping left and right makes it easier on autonomous
        rightColor = hwMap.colorSensor.get("leftColor");

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // holds wobble goal
        leftClaw.setPosition(0);
        rightClaw.setPosition(1);
    }


    /** Methods in progress */

    // Turns until distance sensor detects object
   /* public void findObject (autoStates direction, double power){

        switch(direction){

        case Left:
        while (distanceSensor.getDistance(DistanceUnit.INCH) < 15){ // Change the number if needed
            leftFront.setPower(-power);
            rightFront.setPower(power);
            leftRear.setPower(-power);
            rightRear.setPower(power);
        }

        case Right:
        while (distanceSensor.getDistance(DistanceUnit.INCH) < 15){ // Change the number if needed
            leftFront.setPower(power);
            rightFront.setPower(-power);
            leftRear.setPower(power);
            rightRear.setPower(-power);
            }

        }
    }


    // Moves towards target until within a certain distance. Stays on track using the distance sensor
    // Note: possibly replaces findObject
    public void travelToObject (autoStates command, double minDistance, double maxDistance, double power) throws InterruptedException {

        switch (command) {

            case KeepGoing:
                while (distanceSensor.getDistance(DistanceUnit.INCH) < maxDistance){ // this needs changing

                    if (distanceSensor.getDistance(DistanceUnit.INCH) < minDistance){
                        while (distanceSensor.getDistance(DistanceUnit.INCH) < minDistance) {
                            leftFront.setPower(power);
                            rightFront.setPower(power);
                            leftRear.setPower(power);
                            rightRear.setPower(power);
                        }
                    } else {
                       command = autoStates.Grab;
                    }
                }
                command = autoStates.Left;

            case Left:
                gyStartAngle = updateHeading();
                double startAngle = gyStartAngle;

                while (distanceSensor.getDistance(DistanceUnit.INCH) > maxDistance){ // Change the number if needed

                    if (Math.abs(gyStartAngle-startAngle) < 90){
                        command = autoStates.Right;
                    }
                        leftFront.setPower(-power);
                        rightFront.setPower(power);
                        leftRear.setPower(-power);
                        rightRear.setPower(power);
                        gyStartAngle = updateHeading();
                }
                command = autoStates.KeepGoing;

            case Right:
                while (distanceSensor.getDistance(DistanceUnit.INCH) > maxDistance){ // Change the number if needed
                    leftFront.setPower(-power);
                    rightFront.setPower(power);
                    leftRear.setPower(-power);
                    rightRear.setPower(power);
                }
                command = autoStates.KeepGoing;

            case Grab:
                stop();
                leftClaw.setPosition(0);
                rightClaw.setPosition(1);
        }
    }
*/

    // Note: White alpha = 190  Blue alpha = 86-110  Grey alpha = 65-85
    public void lineFollowRight (int color, double distance, double power) {

        encStartPosition = leftFront.getCurrentPosition();
        while (leftFront.getCurrentPosition() < (distance * COUNTS_PER_INCH + encStartPosition)) {
            // Strafe until wheels are off line
            if (leftColor.alpha() > color && rightColor.alpha() > color) {
                leftFront.setPower(power);
                leftRear.setPower(-power);
                rightFront.setPower(-power);
                rightRear.setPower(power);

                // When left color sensor is on the line and the right is off put power in the right wheels
            } else if (leftColor.alpha() > color && rightColor.alpha() < color) {
                leftFront.setPower(power);
                leftRear.setPower(-power - 0.1);
                rightFront.setPower(-power + 0.1);
                rightRear.setPower(power + 0.1);

                // When right color sensor is on the line and the left is off put power in the left wheels
            } else if (leftColor.alpha() < color && rightColor.alpha() > color) {
                leftFront.setPower(power);
                leftRear.setPower(-power + 0.1);
                rightFront.setPower(-power - 0.1);
                rightRear.setPower(power - 0.1);

                // when both color sensors are off the line move backwards
            } else {
                leftFront.setPower(power);
                leftRear.setPower(-power - 0.1);
                rightFront.setPower(-power - 0.1);
                rightRear.setPower(power - 0.1);
            }


            telemetry.addData("right Alpha", rightColor.alpha());
            telemetry.addData("left Alpha", leftColor.alpha());

            telemetry.addData("leftFront", leftFront.getCurrentPosition());
            telemetry.addData("rightFront", rightFront.getCurrentPosition());
            telemetry.addData("leftRear", leftRear.getCurrentPosition());
            telemetry.addData("rightRear", rightRear.getCurrentPosition());
            telemetry.update();
        }
        stop();
    }


    public void lineFollowLeft (int color, double distance, double power){

        encStartPosition = leftFront.getCurrentPosition();
        while (leftFront.getCurrentPosition() > (-distance * COUNTS_PER_INCH + encStartPosition)){
            if (leftColor.alpha() > color && rightColor.alpha() > color){
                leftFront.setPower(-power);
                leftRear.setPower(power);
                rightFront.setPower(power);
                rightRear.setPower(-power);

            } else if (leftColor.alpha() > color && rightColor.alpha() < color){
                leftFront.setPower(-power);
                leftRear.setPower(power -0.1);
                rightFront.setPower(power + 0.1);
                rightRear.setPower(-power + 0.1);

            } else if (leftColor.alpha() < color && rightColor.alpha() > color){
                leftFront.setPower(-power);
                leftRear.setPower(power + 0.1);
                rightFront.setPower(power - 0.1);
                rightRear.setPower(-power - 0.1);

            } else {
                leftFront.setPower(-power);
                leftRear.setPower(power - 0.1);
                rightFront.setPower(power - 0.1);
                rightRear.setPower(-power -0.1);
            }


            telemetry.addData("right Alpha", rightColor.alpha());
            telemetry.addData("left Alpha", leftColor.alpha());

            telemetry.addData("leftFront", leftFront.getCurrentPosition());
            telemetry.addData("rightFront", rightFront.getCurrentPosition());
            telemetry.addData("leftRear", leftRear.getCurrentPosition());
            telemetry.addData("rightRear", rightRear.getCurrentPosition());
            telemetry.update();
        }
        stop();
    }


    public void gyroDriveToLine (int color, double power){

        // Creation of speed doubles
        double leftFrontSpeed;
        double rightFrontSpeed;
        double leftRearSpeed;
        double rightRearSpeed;

        // Gets starting angle and position of encoders
        gyStartAngle = updateHeading();

        while (rightColor.alpha() < color && leftColor.alpha() < color){
            double currentAngle = updateHeading();
            telemetry.addData("current heading", currentAngle);

            // calculates required speed to adjust to gyStartAngle
            leftFrontSpeed = power + (currentAngle - gyStartAngle) / 100;
            rightFrontSpeed = power - (currentAngle - gyStartAngle) / 100;
            leftRearSpeed = power + (currentAngle - gyStartAngle) / 100;
            rightRearSpeed = power - (currentAngle - gyStartAngle) / 100;

            // Setting range of adjustments (I may be wrong about this)
            leftFrontSpeed = Range.clip(leftFrontSpeed, -1, 1);
            rightFrontSpeed = Range.clip(rightFrontSpeed, -1, 1);
            leftRearSpeed = Range.clip(leftRearSpeed, -1, 1);
            rightRearSpeed = Range.clip(rightRearSpeed, -1, 1);

            // Set new targets
            leftFront.setPower(leftFrontSpeed);
            leftRear.setPower(leftRearSpeed);
            rightFront.setPower(rightFrontSpeed);
            rightRear.setPower(rightRearSpeed);

            telemetry.addData("right Alpha", rightColor.alpha());
            telemetry.addData("left Alpha", leftColor.alpha());

            telemetry.addData("current angle", updateHeading());

            telemetry.update();
        }
        stop();
    }


    /** Movement methods */

    // Turns for a specific amount of degrees
    // Note: Negative power = right positive power = left
    public void gyroTurn(double targetHeading, double power){
        gyStartAngle = updateHeading();
        double startAngle = gyStartAngle;

        // Repeats until current angle (gyStartAngle) reaches targetHeading relative to startAngle
        while(Math.abs(gyStartAngle-startAngle) < targetHeading){
            leftFront.setPower(power);
            rightFront.setPower(-power);
            leftRear.setPower(power);
            rightRear.setPower(-power);
            gyStartAngle = updateHeading();

            // Telemetry
            telemetry.addData("current angle", updateHeading());
            telemetry.update();
        }
        stop();
    }


    // Drives in a straight line for a certain distance in inches
    // Note: can't use to go backwards
    double encStartPosition = 0;
    public void gyroDriveForward (double distance, double power){

        // Creation of speed doubles
        double leftFrontSpeed;
        double rightFrontSpeed;
        double leftRearSpeed;
        double rightRearSpeed;

        // Gets starting angle and position of encoders
        gyStartAngle = updateHeading();
        encStartPosition = leftFront.getCurrentPosition();
        telemetry.addData("startPos", encStartPosition);

        while (leftFront.getCurrentPosition() < (distance * COUNTS_PER_INCH + encStartPosition)){
            double currentAngle = updateHeading();
            telemetry.addData("current heading", currentAngle);

            // calculates required speed to adjust to gyStartAngle
            leftFrontSpeed = power + (currentAngle - gyStartAngle) / 100;
            rightFrontSpeed = power - (currentAngle - gyStartAngle) / 100;
            leftRearSpeed = power + (currentAngle - gyStartAngle) / 100;
            rightRearSpeed = power - (currentAngle - gyStartAngle) / 100;

            // Setting range of adjustments (I may be wrong about this)
            leftFrontSpeed = Range.clip(leftFrontSpeed, -1, 1);
            rightFrontSpeed = Range.clip(rightFrontSpeed, -1, 1);
            leftRearSpeed = Range.clip(leftRearSpeed, -1, 1);
            rightRearSpeed = Range.clip(rightRearSpeed, -1, 1);

            // Set new targets
            leftFront.setPower(leftFrontSpeed);
            leftRear.setPower(leftRearSpeed);
            rightFront.setPower(rightFrontSpeed);
            rightRear.setPower(rightRearSpeed);

            telemetry.addData("current angle", updateHeading());

            telemetry.addData("leftFront", leftFront.getCurrentPosition());
            telemetry.addData("rightFront", rightFront.getCurrentPosition());
            telemetry.addData("leftRear", leftRear.getCurrentPosition());
            telemetry.addData("rightRear", rightRear.getCurrentPosition());
            telemetry.update();
        }
        stop();
    }


    public void gyroDriveBackward (double distance, double power){

        // creation of speed doubles
        double leftFrontSpeed;
        double rightFrontSpeed;
        double leftRearSpeed;
        double rightRearSpeed;

        // Gets starting angle and position of encoders
        gyStartAngle = updateHeading();
        encStartPosition = leftFront.getCurrentPosition();
        telemetry.addData("startpos", encStartPosition);

        while (leftFront.getCurrentPosition() > (-distance * COUNTS_PER_INCH + encStartPosition)){
            double currentAngle = updateHeading();
            telemetry.addData("current heading", currentAngle);

            // calculates required speed to adjust to gyStartAngle
            leftFrontSpeed = power + (currentAngle - gyStartAngle) / 100;
            rightFrontSpeed = power - (currentAngle - gyStartAngle) / 100;
            leftRearSpeed = power + (currentAngle - gyStartAngle) / 100;
            rightRearSpeed = power - (currentAngle - gyStartAngle) / 100;

            // Setting range of adjustments (I may be wrong about this)
            leftFrontSpeed = Range.clip(leftFrontSpeed, 1, -1);
            rightFrontSpeed = Range.clip(rightFrontSpeed, 1, -1);
            leftRearSpeed = Range.clip(leftRearSpeed, 1, -1);
            rightRearSpeed = Range.clip(rightRearSpeed, 1, -1);

            // Set new targets
            leftFront.setPower(-leftFrontSpeed);
            leftRear.setPower(-leftRearSpeed);
            rightFront.setPower(-rightFrontSpeed);
            rightRear.setPower(-rightRearSpeed);

            telemetry.addData("current angle", updateHeading());

            telemetry.addData("leftFront", leftFront.getCurrentPosition()); // this works
            telemetry.addData("rightFront", rightFront.getCurrentPosition());
            telemetry.addData("leftRear", leftRear.getCurrentPosition());
            telemetry.addData("rightRear", rightRear.getCurrentPosition());
            telemetry.update();
        }
        stop();
    }


    // Drives left using Mecanum
    public void strafeLeft (double distance, double power){
        encStartPosition = leftFront.getCurrentPosition();

        while (leftFront.getCurrentPosition() > (-distance * COUNTS_PER_INCH + encStartPosition)){ // changed from < to != also added -
            leftFront.setPower(-power);
            leftRear.setPower(power);
            rightFront.setPower(power);
            rightRear.setPower(-power);

            telemetry.addData("leftFront", leftFront.getCurrentPosition());
            telemetry.addData("rightFront", rightFront.getCurrentPosition());
            telemetry.addData("leftRear", leftRear.getCurrentPosition());
            telemetry.addData("rightRear", rightRear.getCurrentPosition());
            telemetry.update();
        }
        stop();
    }


    // Drives right using Mecanum
    public void strafeRight (double distance, double power){
        encStartPosition = leftFront.getCurrentPosition();

        while (leftFront.getCurrentPosition() < (distance * COUNTS_PER_INCH + encStartPosition)){ // changed from < to != also added -
            leftFront.setPower(power);
            leftRear.setPower(-power);
            rightFront.setPower(-power);
            rightRear.setPower(power);

            telemetry.addData("leftFront", leftFront.getCurrentPosition());
            telemetry.addData("rightFront", rightFront.getCurrentPosition());
            telemetry.addData("leftRear", leftRear.getCurrentPosition());
            telemetry.addData("rightRear", rightRear.getCurrentPosition());
            telemetry.update();
        }
        stop();
    }


    /** Attachment methods*/
    // Drops wobble goal
    public void wobbleDrop (double power){
        int state = 0;

            switch (state){

                    // lower arm until touchSensor2 is inactive
                case 0:
                    while (!touchSensor2.isPressed()){
                        armMotor.setPower(power);
                    }
                    state++;

                    // stop arm and stop servoHold
                case 1:
                    armMotor.setPower(0);
                    state++;

                    // open claw
                case 2:
                    leftClaw.setPosition(0.5);
                    rightClaw.setPosition(0.5);
                    state++;
        }
    }


    /** Other methods */
    // Gets the current angle of the robot
    public double updateHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }


    // Resets all encoders
    public void resetEncoders(){
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    // Stop all motors
    public void stop(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }


    /** Unused methods*/
    // Grab rings and move rings to ramp
    public void intakeRings (double timeoutS){
        while (runtime.seconds() < timeoutS) {
            intake.setPower(1.0);
            intakeServo.setPower(-1.0);
        }
    }


    // Checks if the robot is busy
    public boolean isBusy() {
        return leftFront.isBusy() && rightFront.isBusy()  && leftRear.isBusy() && rightRear.isBusy();
    }


    // Method for Driving from a Current Position to a Requested Position
    // Note: Reverse movement is obtained by setting a negative distance (not speed)
    // Note: Use this to go backwards not gyroDriveStraight
    public void driveToPosition(double speed, double lDistance, double rDistance) {
        int newLeftTarget;
        int newRightTarget;

        resetEncoders();

        // Convert chosen distance from inches to motor ticks and set each motor to the new target
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

        // Reset the timeout time and start motion.
        runtime.reset();
        leftFront.setPower(Math.abs(speed));
        rightFront.setPower(Math.abs(speed));
        leftRear.setPower(Math.abs(speed));
        rightRear.setPower(Math.abs(speed));

        // Telemetry
        telemetry.addData("leftFront", leftFront.getCurrentPosition());
        telemetry.addData("rightFront", rightFront.getCurrentPosition());
        telemetry.addData("leftRear", leftRear.getCurrentPosition());
        telemetry.addData("rightRear", rightRear.getCurrentPosition());
        telemetry.update();

        // Stop the robot because it is done with teh move.
        stop();

        // Turn off RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Strafes using Mecanum wheels
    // Note: Negative power and distance goes right. Positive power and distance goes left
    public void strafe2 (double distance, double power){
        int newTarget1;
        int newTarget2;

        resetEncoders();

        // Convert chosen distance from inches to motor ticks and set each motor to the new target
        newTarget1 = leftRear.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        newTarget2 = rightRear.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        leftFront.setTargetPosition(-newTarget2);
        rightFront.setTargetPosition(newTarget1);
        leftRear.setTargetPosition(newTarget1);
        rightRear.setTargetPosition(-newTarget2);

        // Turn On RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset the timeout time and start motion.
        runtime.reset();
        leftFront.setPower(-Math.abs(-power));
        rightFront.setPower(Math.abs(power));
        leftRear.setPower(Math.abs(power));
        rightRear.setPower(-Math.abs(-power));

        // Telemetry
        telemetry.addData("leftFront", leftFront.getCurrentPosition()); // this works
        telemetry.addData("rightFront", rightFront.getCurrentPosition());
        telemetry.addData("leftRear", leftRear.getCurrentPosition());
        telemetry.addData("rightRear", rightRear.getCurrentPosition());
        telemetry.update();

        // Stop the robot because it is done with teh move.
        stop();

        // Turn off RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
