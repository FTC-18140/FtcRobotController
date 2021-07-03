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
    /*
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
        */


    // Initialize tenserFlow
    public void initTfod(HardwareMap ahwMap) {
        int tfodMonitorViewId = ahwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", ahwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }



    /** Methods */

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

}