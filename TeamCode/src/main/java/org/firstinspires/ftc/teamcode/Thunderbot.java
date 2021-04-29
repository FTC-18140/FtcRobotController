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

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

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
    CRServo rampIntakeServo = null;
    DcMotor shooterMotor = null;
    DcMotor shooterMotor2 = null;

    Servo leftClaw = null;
    Servo rightClaw = null;

    TouchSensor touchSensor1 = null;
    TouchSensor touchSensor2 = null;
    ColorSensor leftColor = null;
    ColorSensor rightColor = null;
    DistanceSensor distanceSensor = null;

    //CameraName cameraName = null;

    // converts inches to motor ticks
    static final double     COUNTS_PER_MOTOR_REV    = 28; // rev robotics hd hex motors planetary 411600
    static final double     DRIVE_GEAR_REDUCTION    = 20;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0; // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (WHEEL_DIAMETER_INCHES * 3.1415);


    /** local OpMode members. */
    HardwareMap hwMap =  null;
    private Telemetry telemetry;
    private ElapsedTime runtime  = new ElapsedTime();

    /** Gyro */
    BNO055IMU imu = null;
    Orientation angles = null;

    static double gyStartAngle = 0; // a shared gyro start position this will be updated using updateHeading()

    /**Computor vision*/
    public static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Quad";
    public static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            "AdAW/iz/////AAABmYgL/USiRk6wrOU3PCgllcxrhasjPkR3tL10jedJq6lpPU579fPiO66TP5B2TqVBBQUzjhrWC19IXDOsKhj045Ri82dk7C2f9cnpR6rcdmxJqc0rOVFk7e4/hAo8Pfmisj6In2mN7ibcBAE3MkE6VzGF0Op8cukn4US3+jpnd9WnHjAwJTo+jM9PNkYhIwJrwLfnKIOYbT71xQptdT0FBFVBvcW8Ru3baL7xTD71qL9aJqP3M2VH7JrRrVroJUUZrfL3CB+l6eTiVfO3JLDDHR/7DHeuDtzpbqZBFrXce2X2zAl4I1sD1A/sX3j7k6nuIcStJ2AqXTDi93/H2YuM4PZN0NyMGb8ffUkkXDV6/d2L";
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    public int nOfSetRings;

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
            imu = ahwMap.get(BNO055IMU.class, "imu 1");
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
        rampIntakeServo = hwMap.crservo.get("rampIntakeServo");

        //  Define & Initialize Sensors
        touchSensor1 = hwMap.touchSensor.get("touchSensor1");
        touchSensor2 = hwMap.touchSensor.get("touchSensor2");
        leftColor = hwMap.colorSensor.get("rightColor"); // Note: swapping left and right makes it easier on autonomous
        rightColor = hwMap.colorSensor.get("leftColor");
        distanceSensor = hwMap.get(DistanceSensor.class, "distanceSensor");

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // holds wobble goal
        leftClaw.setPosition(0);
        rightClaw.setPosition(1);
    }

    /**Computer vision methods*/
    public void initVuforia(HardwareMap ahwMap) {
         //Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = ahwMap.get(WebcamName.class, "Webcam 1");


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    public void checkRings() {
        while (true) { // while(true) doesn't work try a different while loop
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    if (updatedRecognitions.size() == 0) {
                        // empty list.  no objects recognized.
                        telemetry.addData("TFOD", "No items detected.");
                        telemetry.addData("Target Zone", "A");
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
                            } else if (recognition.getLabel().equals("Quad")) {
                                telemetry.addData("Target Zone", "C");
                            } else {
                                telemetry.addData("Target Zone", "UNKNOWN");
                            }
                        }
                    }
                }
                telemetry.update();
            }
        }
    }

    public void initTfod(HardwareMap ahwMap) {
        int tfodMonitorViewId = ahwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", ahwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    /** Methods in progress */
    public void strafeLeftToObject (double minDistance, double maxDistance, double power) throws InterruptedException {

        while (true) {
            // Drive forward if the distance sensor sees an object
            if (distanceSensor.getDistance(DistanceUnit.INCH) > minDistance && distanceSensor.getDistance(DistanceUnit.INCH) < maxDistance) {
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftRear.setPower(power);
                rightRear.setPower(power);
                telemetry.addData("distance",distanceSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();

                // Strafe left looking for object
            } else if (distanceSensor.getDistance(DistanceUnit.INCH) > maxDistance) {
                leftFront.setPower(-power);
                rightFront.setPower(power);
                leftRear.setPower(power);
                rightRear.setPower(-power);
                telemetry.addData("distance",distanceSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();

                // When in range grab object
            }else if (distanceSensor.getDistance(DistanceUnit.INCH) < minDistance) {
                stop();
                leftClaw.setPosition(0);
                rightClaw.setPosition(1);
                sleep(1000);
                break;
            } else {
                stop();
                leftClaw.setPosition(0);
                rightClaw.setPosition(1);
                sleep(500);
                break;
            }
        }
    }


    /** Color/Distance Methods*/
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


    public void lineFollowRight (int color, double distance, double power){

        encStartPosition = rightFront.getCurrentPosition();
        while (rightFront.getCurrentPosition() > (-distance * COUNTS_PER_INCH + encStartPosition)){
            if (leftColor.alpha() > color && rightColor.alpha() > color){
                leftFront.setPower(power);
                leftRear.setPower(-power);
                rightFront.setPower(-power);
                rightRear.setPower(power);

            } else if (leftColor.alpha() > color && rightColor.alpha() < color){
                leftFront.setPower(power - 0.1);
                leftRear.setPower(-power -0.1);
                rightFront.setPower(-power);
                rightRear.setPower(power + 0.1);

            } else if (leftColor.alpha() < color && rightColor.alpha() > color){
                leftFront.setPower(power + 0.1);
                leftRear.setPower(-power + 0.1);
                rightFront.setPower(-power);
                rightRear.setPower(power - 0.1);

            } else {
                leftFront.setPower(power + 0.1);
                leftRear.setPower(-power + 0.1);
                rightFront.setPower(-power);
                rightRear.setPower(power + 0.1);
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


    public void travelToObject (double minDistance, double maxDistance, double power) throws InterruptedException {
        double wobbleAngle = 0.0;
        double currentAngle = updateHeading();
        while (true) {
            // Drive forward if the distance sensor sees an object
            if (distanceSensor.getDistance(DistanceUnit.INCH) > minDistance && distanceSensor.getDistance(DistanceUnit.INCH) < maxDistance) {
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftRear.setPower(power);
                rightRear.setPower(power);
                telemetry.addData("distance",distanceSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();
                wobbleAngle = updateHeading();

                // Turn left looking for object
            } else if (distanceSensor.getDistance(DistanceUnit.INCH) > maxDistance) {
                leftFront.setPower(power);
                rightFront.setPower(-power);
                leftRear.setPower(power);
                rightRear.setPower(-power);
                telemetry.addData("Current angle",  currentAngle);
                telemetry.update();

                // Turn right looking for object
            } else if (wobbleAngle > currentAngle) {
                leftFront.setPower(-power);
                rightFront.setPower(power);
                leftRear.setPower(-power);
                rightRear.setPower(power);
                telemetry.addData("Current angle",  currentAngle);
                telemetry.update();

                // When in range grab object
            }else if (distanceSensor.getDistance(DistanceUnit.INCH) < minDistance) {
                stop();
                leftClaw.setPosition(0);
                rightClaw.setPosition(1);
                sleep(1000);
                break;
            } else {
                break;
            }
            currentAngle = updateHeading();
        }
    }


    /** Gyro methods */
    // Turns for a specific amount of degrees
    // Note: Negative power = right positive power = left
    public void gyroTurn(double targetHeading, double power) {
        gyStartAngle = updateHeading();
        double startAngle = gyStartAngle;

        // Repeats until current angle (gyStartAngle) reaches targetHeading relative to startAngle
        while(Math.abs(gyStartAngle-startAngle) < targetHeading) {
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

        while (leftFront.getCurrentPosition() < (distance * COUNTS_PER_INCH + encStartPosition)) {
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

        while (leftFront.getCurrentPosition() > (-distance * COUNTS_PER_INCH + encStartPosition)) {
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


    //
    public void strafeLeft (double distance, double power) {
        encStartPosition = leftFront.getCurrentPosition();

        while (leftFront.getCurrentPosition() > (-distance * COUNTS_PER_INCH + encStartPosition)) { // changed from < to != also added -
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

    public void strafeRight (double distance, double power) {
        encStartPosition = leftFront.getCurrentPosition();

        while (leftFront.getCurrentPosition() < (distance * COUNTS_PER_INCH + encStartPosition)) { // changed from < to != also added -
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
    public void wobbleDrop (double power) throws InterruptedException {
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
    public double updateHeading() {
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
    public void stop() {
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

    /** Target zone methods */
    public void targetZoneA () throws InterruptedException {
        shooterMotor.setPower(0.60); // Start up shooterMotors
        shooterMotor2.setPower(0.60);

        gyroDriveForward(58, 0.6); // Go forward 70 inches to line up on the shooting line (could change)
        gyroDriveToLine (120, 0.2 );

        lineFollowLeft(190, 14, 0.5); // Strafe left 10 inches in order to line up the robot to fire the rings

        sleep(2000); // Wait 2 secs to allow the rings to reach full power
        shooterServo1.setPower(-1.0); // Move rings into shooterMotors to fire rings
        shooterServo2.setPower(-1.0);
        rampIntakeServo.setPower(-0.5);
        sleep(1000); // Wait 3 secs to allow all the rings to fire
        shooterServo1.setPower(0);
        shooterServo2.setPower(0);

        lineFollowLeft(190, 6, 0.5); // Strafe left 10 inches in order to line up the robot to fire the rings

        // Note: this time will be able to be reduced if needed
        shooterServo1.setPower(-1.0); // Move rings into shooterMotors to fire rings
        shooterServo2.setPower(-1.0);
        sleep(2500); // Wait 3 secs to allow all the rings to fire

        lineFollowRight(190, 39, 0.4);

        shooterMotor.setPower(0); // Turn off shooterMotors and shooterServos to conserve power
        shooterMotor2.setPower(0);
        shooterServo1.setPower(0);
        shooterServo2.setPower(0);
        rampIntakeServo.setPower(0);



        gyroTurn(73, -0.2); // turn 90

        gyroDriveForward(10,0.5);// drive forward to square A

        wobbleDrop(0.7); // Drop the wobble goal in the square A

        gyroDriveBackward(15, 0.5);// go backward the same amount as previously going forward

        strafeRight(90, 0.7); // strafe into wall  // May need to reduce the distance a bit

        strafeLeftToObject(3, 20, 0.1); // look for and grab wobble

        gyroDriveToLine (110, 0.2 );

        strafeLeft(65, 0.5); // get to the white line
    }

    public void targetZoneB() throws InterruptedException {
        shooterMotor.setPower(0.60); // Start up shooterMotors
        shooterMotor2.setPower(0.60);

        gyroDriveForward(58, 0.6); // Go forward 70 inches to line up on the shooting line (could change)
        gyroDriveToLine (120, 0.2 );

        lineFollowLeft(190, 14, 0.5); // Strafe left 10 inches in order to line up the robot to fire the rings

        sleep(2000); // Wait 2 secs to allow the rings to reach full power
        shooterServo1.setPower(-1.0); // Move rings into shooterMotors to fire rings
        shooterServo2.setPower(-1.0);
        rampIntakeServo.setPower(-0.5);
        sleep(1000); // Wait 3 secs to allow all the rings to fire
        shooterServo1.setPower(0);
        shooterServo2.setPower(0);

        lineFollowLeft(190, 6, 0.5); // Strafe left 10 inches in order to line up the robot to fire the rings

        // Note: this time will be able to be reduced if needed
        shooterServo1.setPower(-1.0); // Move rings into shooterMotors to fire rings
        shooterServo2.setPower(-1.0);
        sleep(2500); // Wait 3 secs to allow all the rings to fire

        lineFollowRight(190, 39, 0.4);

        shooterMotor.setPower(0); // Turn off shooterMotors and shooterServos to conserve power
        shooterMotor2.setPower(0);
        shooterServo1.setPower(0);
        shooterServo2.setPower(0);
        rampIntakeServo.setPower(0);



        gyroDriveForward(20, 0.5); // Go forward 15 inches into the square B

        wobbleDrop(0.7); // Drop the wobble goal in the square B

        gyroTurn(73, -0.2); // turn 73 degrees right

        strafeRight(94, 0.7); // strafe into wall   // changed from 0.6

        gyroDriveForward(10, 0.5);

        strafeLeftToObject(3, 20, 0.1); // look for and grab wobble

        //gyroDriveToLine (110, 0.2 );
        //maybe go backwards

        strafeLeft(70, 0.5); // get to the white line

        gyroTurn(80, 1.0);

        gyroDriveForward(10,0.5);
    }

    public void targetZoneC() throws InterruptedException {
        shooterMotor.setPower(0.60); // Start up shooterMotors
        shooterMotor2.setPower(0.60);

        gyroDriveForward(58, 0.6); // Go forward 70 inches to line up on the shooting line (could change)
        gyroDriveToLine (120, 0.2 );

        lineFollowLeft(190, 14, 0.5); // Strafe left 10 inches in order to line up the robot to fire the rings

        sleep(2000); // Wait 2 secs to allow the rings to reach full power
        shooterServo1.setPower(-1.0); // Move rings into shooterMotors to fire rings
        shooterServo2.setPower(-1.0);
        rampIntakeServo.setPower(-0.5);
        sleep(1000); // Wait 3 secs to allow all the rings to fire
        shooterServo1.setPower(0);
        shooterServo2.setPower(0);

        lineFollowLeft(190, 6, 0.5); // Strafe left 10 inches in order to line up the robot to fire the rings

        // Note: this time will be able to be reduced if needed
        shooterServo1.setPower(-1.0); // Move rings into shooterMotors to fire rings
        shooterServo2.setPower(-1.0);
        sleep(2500); // Wait 3 secs to allow all the rings to fire

        lineFollowRight(190, 39, 0.4);

        shooterMotor.setPower(0); // Turn off shooterMotors and shooterServos to conserve power
        shooterMotor2.setPower(0);
        shooterServo1.setPower(0);
        shooterServo2.setPower(0);
        rampIntakeServo.setPower(0);




        /*lineFollowRight(190, 21, 0.4);    // hit the wall

        gyroDriveForward(70, 0.5); // Go forward high power to square C
        // maybe look for blue line on square C
        // maybe turn to place more out of the way
        wobbleDrop(0.7); // Drop the wobble goal in the square B
        //go backwards fast
        // then look for white line either using gyroDriveToLine or a new method going backwards
        // lineFollowLeft

        gyroTurn(73, -0.2); // turn 90

        strafeRight(90, 0.7); // strafe into wall   // Maybe lower the distance more

        gyroDriveForward(7, 0.5);

        strafeLeftToObject(3, 20, 0.1); // look for and grab wobble

        gyroDriveToLine (110, 0.2 );

        strafeLeft(70, 0.5);          // get to Square C quickly
        */
    }

    public void targetZoneAB() throws InterruptedException {
        shooterMotor.setPower(0.60); // Start up shooterMotors
        shooterMotor2.setPower(0.60);

        gyroDriveForward(58, 0.6); // Go forward 70 inches to line up on the shooting line (could change)
        gyroDriveToLine (120, 0.2 );

        lineFollowLeft(190, 14, 0.5); // Strafe left 10 inches in order to line up the robot to fire the rings

        sleep(2000); // Wait 2 secs to allow the rings to reach full power
        shooterServo1.setPower(-1.0); // Move rings into shooterMotors to fire rings
        shooterServo2.setPower(-1.0);
        rampIntakeServo.setPower(-0.5);
        sleep(1000); // Wait 3 secs to allow all the rings to fire
        shooterServo1.setPower(0);
        shooterServo2.setPower(0);

        lineFollowLeft(190, 6, 0.5); // Strafe left 10 inches in order to line up the robot to fire the rings

        // Note: this time will be able to be reduced if needed
        shooterServo1.setPower(-1.0); // Move rings into shooterMotors to fire rings
        shooterServo2.setPower(-1.0);
        sleep(2500); // Wait 3 secs to allow all the rings to fire

        lineFollowRight(190, 39, 0.4);

        shooterMotor.setPower(0); // Turn off shooterMotors and shooterServos to conserve power
        shooterMotor2.setPower(0);
        shooterServo1.setPower(0);
        shooterServo2.setPower(0);
        rampIntakeServo.setPower(0);

        gyroDriveForward(20, 0.5); // Go forward 15 inches into the square B

        wobbleDrop(0.7); // Drop the wobble goal in the square B

        gyroTurn(73, -0.2); // turn 90
        strafeRight(94, 0.7); // strafe into wall   // changed from 0.6
        gyroDriveForward(7, 0.5);
        strafeLeftToObject(3, 20, 0.1); // look for and grab wobble
        gyroDriveToLine (110, 0.2 );
        strafeLeft(70, 0.5); // get to the white line
    }
}


