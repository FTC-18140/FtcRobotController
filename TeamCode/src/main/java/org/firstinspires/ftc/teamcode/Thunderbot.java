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

public class Thunderbot {
    /** Public OpMode members */
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
    BNO055IMU imu = null;
    Orientation angles = null;

    // converts inches to motor ticks
    static final double COUNTS_PER_MOTOR_REV = 28; // rev robotics hd hex motors planetary 411600
    static final double DRIVE_GEAR_REDUCTION = 20;
    static final double WHEEL_DIAMETER_INCHES = 4.0; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (WHEEL_DIAMETER_INCHES * 3.1415);


    /** local OpMode members */
    HardwareMap hwMap = null;
    private Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();

    static double gyStartAngle = 0; // a shared gyro start position this will be updated using updateHeading()

    /** Computor vision */
    public static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Quad";
    public static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            "AdAW/iz/////AAABmYgL/USiRk6wrOU3PCgllcxrhasjPkR3tL10jedJq6lpPU579fPiO66TP5B2TqVBBQUzjhrWC19IXDOsKhj045Ri82dk7C2f9cnpR6rcdmxJqc0rOVFk7e4/hAo8Pfmisj6In2mN7ibcBAE3MkE6VzGF0Op8cukn4US3+jpnd9WnHjAwJTo+jM9PNkYhIwJrwLfnKIOYbT71xQptdT0FBFVBvcW8Ru3baL7xTD71qL9aJqP3M2VH7JrRrVroJUUZrfL3CB+l6eTiVfO3JLDDHR/7DHeuDtzpbqZBFrXce2X2zAl4I1sD1A/sX3j7k6nuIcStJ2AqXTDi93/H2YuM4PZN0NyMGb8ffUkkXDV6/d2L";
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    public int nOfSetRings;

    /** Constructor */
    public Thunderbot() {

    }

    /** Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry telem) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        telemetry = telem;

        try {
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
        } catch (Exception p_exeception) {
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

    /** Computer vision methods */
    // Initializes Vuforia
    public void initVuforia(HardwareMap ahwMap) {
        //Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = ahwMap.get(WebcamName.class, "Webcam 1");


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    // Doesn't work right now
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

    // Initialize tenserFlow
    public void initTfod(HardwareMap ahwMap) {
        int tfodMonitorViewId = ahwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", ahwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }


    /** Methods in progress */



    /** Color sensor Methods */
    // Follows lines using the color sensors left
    public void lineFollowLeft(int color, double distance, double power) {

        encStartPosition = leftFront.getCurrentPosition();

        // Follow the line until the robot reaches the specified distance
        while (leftFront.getCurrentPosition() > (-distance * COUNTS_PER_INCH + encStartPosition)) {

            // If both color sensors see the specified color the robot continues left
            if (leftColor.alpha() > color && rightColor.alpha() > color) {
                leftFront.setPower(-power);
                leftRear.setPower(power);
                rightFront.setPower(power);
                rightRear.setPower(-power);

                // If the right color sensor is off the line increase the right wheel powers and reduce the powers in the left
            } else if (leftColor.alpha() > color && rightColor.alpha() < color) {
                leftFront.setPower(-power);
                leftRear.setPower(power - 0.05);
                rightFront.setPower(power + 0.05);
                rightRear.setPower(-power + 0.05);

                // If the left color sensor is off the line increase the left wheel powers and increase the powers in the right
            } else if (leftColor.alpha() < color && rightColor.alpha() > color) {
                leftFront.setPower(-power);
                leftRear.setPower(power + 0.05);
                rightFront.setPower(power - 0.15);
                rightRear.setPower(-power - 0.15);

                // If both wheels are off the line, move backwards
            } else {
                leftFront.setPower(-power);
                leftRear.setPower(power - 0.1);
                rightFront.setPower(power - 0.1);
                rightRear.setPower(-power - 0.1);
            }

            // Telemetry for wheels and color sensors
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


    // Follows lines moving right
    public void lineFollowRight(int color, double distance, double power) {

        encStartPosition = rightFront.getCurrentPosition();

        // Follow the line until the robot reaches the specified distance
        while (rightFront.getCurrentPosition() > (-distance * COUNTS_PER_INCH + encStartPosition)) {

            // If both color sensors see the specified color the robot continues right
            if (leftColor.alpha() > color && rightColor.alpha() > color) {
                leftFront.setPower(power);
                leftRear.setPower(-power);
                rightFront.setPower(-power);
                rightRear.setPower(power);

                // If the right color sensor is off the line increase the right wheel powers and reduce the powers in the left
            } else if (leftColor.alpha() > color && rightColor.alpha() < color) {
                leftFront.setPower(power - 0.1);
                leftRear.setPower(-power - 0.1);
                rightFront.setPower(-power);
                rightRear.setPower(power + 0.1);

                // If the left color sensor is off the line increase the left wheel powers and increase the powers in the right
            } else if (leftColor.alpha() < color && rightColor.alpha() > color) {
                leftFront.setPower(power + 0.1);
                leftRear.setPower(-power + 0.1);
                rightFront.setPower(-power);
                rightRear.setPower(power - 0.1);

                // If both wheels are off the line, move backwards
            } else {
                leftFront.setPower(power + 0.1);
                leftRear.setPower(-power + 0.1);
                rightFront.setPower(-power);
                rightRear.setPower(power + 0.1);
            }

            // Telemetry for wheels and color sensors
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


    // Drives forward and stops on the line
    public void gyroDriveToLine(int color, double power) {

        // Creation of speed doubles
        double leftFrontSpeed;
        double rightFrontSpeed;
        double leftRearSpeed;
        double rightRearSpeed;

        // Gets starting angle and position of encoders
        gyStartAngle = updateHeading();

        // Move forward until the color sensors detect the line
        while (rightColor.alpha() < color && leftColor.alpha() < color) {
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

            // Telemetry for color sensors and current angle
            telemetry.addData("right Alpha", rightColor.alpha());
            telemetry.addData("left Alpha", leftColor.alpha());
            telemetry.addData("current angle", updateHeading());

            telemetry.update();
        }
        stop();
    }


    /** Distance sensor Methods */
    // Turns until the distance sensor detects an object
    public void turnToObject(double minDistance, double maxDistance, double power) throws InterruptedException {
        double wobbleAngle = 0.0;
        double currentAngle = updateHeading();
        while (true) {
            // Drive forward if the distance sensor sees an object
            if (distanceSensor.getDistance(DistanceUnit.INCH) > minDistance && distanceSensor.getDistance(DistanceUnit.INCH) < maxDistance) {
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftRear.setPower(power);
                rightRear.setPower(power);
                telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();
                wobbleAngle = updateHeading();

                // Turn left looking for object
            } else if (distanceSensor.getDistance(DistanceUnit.INCH) > maxDistance) {
                leftFront.setPower(power);
                rightFront.setPower(-power);
                leftRear.setPower(power);
                rightRear.setPower(-power);
                telemetry.addData("Current angle", currentAngle);
                telemetry.update();

                // Turn right looking for object
            } else if (wobbleAngle > currentAngle) {
                leftFront.setPower(-power);
                rightFront.setPower(power);
                leftRear.setPower(-power);
                rightRear.setPower(power);
                telemetry.addData("Current angle", currentAngle);
                telemetry.update();

                // When in range grab object
            } else if (distanceSensor.getDistance(DistanceUnit.INCH) < minDistance) {
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

    // Strafes left until the distance sensor sees an object
    public void strafeLeftToObject(double minDistance, double maxDistance, double power) throws InterruptedException {

        while (true) {
            // Drive forward if the distance sensor sees an object
            if (distanceSensor.getDistance(DistanceUnit.INCH) > minDistance && distanceSensor.getDistance(DistanceUnit.INCH) < maxDistance) {
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftRear.setPower(power);
                rightRear.setPower(power);
                telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();

                // Strafe left looking for object
            } else if (distanceSensor.getDistance(DistanceUnit.INCH) > maxDistance) {
                leftFront.setPower(-power);
                rightFront.setPower(power);
                leftRear.setPower(power);
                rightRear.setPower(-power);
                telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();

                // When in range grab object
            } else if (distanceSensor.getDistance(DistanceUnit.INCH) < minDistance) {
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


    /** Gyro methods */
    // Turns for a specific amount of degrees
    // Note: Negative power = left   positive power = right
    public void gyroTurn(double targetHeading, double power) {
        gyStartAngle = updateHeading();
        double startAngle = gyStartAngle;

        // Repeats until current angle (gyStartAngle) reaches targetHeading relative to startAngle
        while (Math.abs(gyStartAngle - startAngle) < targetHeading) {
            leftFront.setPower(power);
            rightFront.setPower(-power);
            leftRear.setPower(power);
            rightRear.setPower(-power);
            gyStartAngle = updateHeading();

            // Telemetry fpr the gyro's angle
            telemetry.addData("current angle", updateHeading());
            telemetry.update();
        }
        stop();
    }


    // Drives in a straight line for a certain distance in inches
    double encStartPosition = 0;
    public void gyroDriveForward(double distance, double power) {

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

    // Drives backwards in a straight line for a certain distance in inches
    public void gyroDriveBackward(double distance, double power) {

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

            // Telemetry for the wheel encoders and gyro angle
            telemetry.addData("current angle", updateHeading());
            telemetry.addData("leftFront", leftFront.getCurrentPosition()); // this works
            telemetry.addData("rightFront", rightFront.getCurrentPosition());
            telemetry.addData("leftRear", leftRear.getCurrentPosition());
            telemetry.addData("rightRear", rightRear.getCurrentPosition());
            telemetry.update();
        }
        stop();
    }


    /** Just encoder methods */

    // Moves backwards a specified distance
    public void driveBackwards(double distance, double power) {
        encStartPosition = leftFront.getCurrentPosition();

        while (leftFront.getCurrentPosition() > (-distance * COUNTS_PER_INCH + encStartPosition)) {
            leftFront.setPower(-power);
            leftRear.setPower(-power);
            rightFront.setPower(-power);
            rightRear.setPower(-power);

            // Telemetry for the encoder wheels
            telemetry.addData("leftFront", leftFront.getCurrentPosition());
            telemetry.addData("rightFront", rightFront.getCurrentPosition());
            telemetry.addData("leftRear", leftRear.getCurrentPosition());
            telemetry.addData("rightRear", rightRear.getCurrentPosition());
            telemetry.update();
        }
        stop();
    }

    // Strafes left a specified distance
    public void strafeLeft(double distance, double power) {
        encStartPosition = leftFront.getCurrentPosition();

        while (leftFront.getCurrentPosition() > (-distance * COUNTS_PER_INCH + encStartPosition)) {
            leftFront.setPower(-power);
            leftRear.setPower(power);
            rightFront.setPower(power);
            rightRear.setPower(-power);

            // Telemetry for the encoder wheels
            telemetry.addData("leftFront", leftFront.getCurrentPosition());
            telemetry.addData("rightFront", rightFront.getCurrentPosition());
            telemetry.addData("leftRear", leftRear.getCurrentPosition());
            telemetry.addData("rightRear", rightRear.getCurrentPosition());
            telemetry.update();
        }
        stop();
    }

    // Strafe right a specified distance
    public void strafeRight(double distance, double power) {
        encStartPosition = leftFront.getCurrentPosition();

        while (leftFront.getCurrentPosition() < (distance * COUNTS_PER_INCH + encStartPosition)) {
            leftFront.setPower(power);
            leftRear.setPower(-power);
            rightFront.setPower(-power);
            rightRear.setPower(power);

            // Telemetry for the encoder wheels
            telemetry.addData("leftFront", leftFront.getCurrentPosition());
            telemetry.addData("rightFront", rightFront.getCurrentPosition());
            telemetry.addData("leftRear", leftRear.getCurrentPosition());
            telemetry.addData("rightRear", rightRear.getCurrentPosition());
            telemetry.update();
        }
        stop();
    }

    /** Attachment methods */
    // Drops wobble goal
    public void wobbleDrop(double power) throws InterruptedException {
        int state = 0;

        switch (state) {

            // lower arm until touchSensor2 is inactive
            case 0:
                while (!touchSensor2.isPressed()) {
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
    public void resetEncoders() {
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

    /** Unused methods */

    // Grab rings and move rings to ramp
    public void intakeRings(double timeoutS) {
        while (runtime.seconds() < timeoutS) {
            intake.setPower(1.0);
            intakeServo.setPower(-1.0);
        }
    }


    // Checks if the robot is busy
    public boolean isBusy() {
        return leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy();
    }


    // Method for Driving from a Current Position to a Requested Position
    // Note: Reverse movement is obtained by setting a negative distance (not speed)
    // Note: Use this to go backwards not gyroDriveStraight
    public void driveToPosition(double speed, double lDistance, double rDistance) {
        int newLeftTarget;
        int newRightTarget;

        resetEncoders();

        // Convert chosen distance from inches to motor ticks and set each motor to the new target
        newLeftTarget = leftRear.getCurrentPosition() + (int) (lDistance * COUNTS_PER_INCH);
        newRightTarget = rightRear.getCurrentPosition() + (int) (rDistance * COUNTS_PER_INCH);
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
    public void strafe2(double distance, double power) {
        int newTarget1;
        int newTarget2;

        resetEncoders();

        // Convert chosen distance from inches to motor ticks and set each motor to the new target
        newTarget1 = leftRear.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newTarget2 = rightRear.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
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