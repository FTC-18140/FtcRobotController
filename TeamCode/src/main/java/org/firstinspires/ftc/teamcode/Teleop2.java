package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;



import static java.lang.Math.abs;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

@TeleOp(name="Teleop2", group="Teleop")
public class Teleop2 extends OpMode
{
    //motors

    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftRear = null;
    DcMotor rightRear = null;
    DcMotor intake;
    DcMotor shooterMotor2;
    DcMotor shooterMotor;
    DcMotor armMotor;

    //servos

    Servo leftClaw;
    Servo rightClaw;
    Servo rampServo;
    CRServo intakeServo;
    CRServo intakeServoTwo;
    CRServo shooterServo1;
    CRServo shooterServo2;

    //touch sensors

    TouchSensor touchSensor1;
    TouchSensor touchSensor2;

    //digital channels

    DigitalChannel digitalTouch;
    DigitalChannel digitalTouch2;



    public void init()
    {
        //logic for Right front
        try
        {
            rightFront = hardwareMap.dcMotor.get("rightFront");
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception p_exeception)
        {
            rightFront = null;

        }
        //logic for right rear
        try
        {
            rightRear = hardwareMap.dcMotor.get("rightRear");
            rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
            rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
        catch (Exception p_exeception)
        {
            rightRear = null;

        }
        //logic for leftFront
        try
    {

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    catch (Exception p_exeception)
    {
        leftFront = null;
    }
        //logic for leftRear
       try
        {
            leftRear = hardwareMap.dcMotor.get("leftRear");
            leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
            leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
        catch (Exception p_exeception)
        {
            leftRear = null;
        }


       //motors

        intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setPower(1);

        shooterMotor2 = hardwareMap.dcMotor.get("shooterMotor2");
        shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setPower(0);

        shooterMotor = hardwareMap.dcMotor.get("shooterMotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setPower(0);

        armMotor = hardwareMap.dcMotor.get("armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setPower(0);

        //servos for wobble goal arm

        leftClaw = hardwareMap.servo.get("leftClaw");
        leftClaw.setPosition(0);

        rightClaw = hardwareMap.servo.get("rightClaw");
        rightClaw.setPosition(1);


        //Servos for intake and shooter

        intakeServoTwo = hardwareMap.crservo.get("intakeServoTwo");
        intakeServoTwo.setPower(1);

        rampServo = hardwareMap.servo.get("rampServo");
        rampServo.setPosition(0);

        intakeServo = hardwareMap.crservo.get("intakeServo");
        intakeServo.setPower(-1);

        shooterServo1 = hardwareMap.crservo.get("shooterServo1");
        shooterServo1.setPower(-1);

        shooterServo2 = hardwareMap.crservo.get("shooterServo2");
        shooterServo2.setPower(-1);



        //touch sensors

        touchSensor1 = hardwareMap.touchSensor.get("touchSensor1");
        touchSensor2 = hardwareMap.touchSensor.get("touchSensor2");

        // get a reference to our digitalTouch object.
       //  digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
     //    digitalTouch2 = hardwareMap.get(DigitalChannel.class, "digital_sensor");

        // set the digital channel to input.
       // digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        //digitalTouch2.setMode(DigitalChannel.Mode.INPUT);

    }

    public void start(){}



    public void loop() {


        //arm motor controls

        if (gamepad1.left_bumper && !touchSensor1.isPressed()) {
            armMotor.setPower(0.7);


        } else if (touchSensor1.isPressed()) {
            armMotor.setPower(0);
            armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        if (gamepad1.left_bumper && !touchSensor2.isPressed()) {
            armMotor.setPower(0.7);


        } else if (touchSensor2.isPressed()) {
            armMotor.setPower(0);
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        }

        //Gamepad 2 arm controls


        if (gamepad2.left_bumper && !touchSensor1.isPressed()) {
            armMotor.setPower(0.7);


        } else if (touchSensor1.isPressed()) {
            armMotor.setPower(0);
            armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        if (gamepad2.left_bumper && !touchSensor2.isPressed()) {
            armMotor.setPower(0.7);


        } else if (touchSensor2.isPressed()) {
            armMotor.setPower(0);
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        }


        //joystick values x and y on left stick; x only on right stick
        double forward = gamepad1.left_stick_y;     // push left joystick forward to go forward
        double right = -gamepad1.left_stick_x;        // push left joystick to the right to strafe right
        double clockwise = -gamepad1.right_stick_x;   // push right joystick to the right to rotate clockwise

//intake
        if (gamepad1.x) {
             intake.setPower(-1);

        } else {
            intake.setPower(1);
                    }

//ramp servo
        if (gamepad1.left_bumper) {
            rampServo.setPosition(0.3);
        } else {
            rampServo.setPosition(0);
        }
//arm kill switch
        if (gamepad1.b) {
            armMotor.setPower(0);
        }

        //Shooter servo kill switch
        if (gamepad1.dpad_up) {
            shooterServo1.setPower(0);
            shooterServo2.setPower(0);
        }else {
            shooterServo2.setPower(-1);
            shooterServo1.setPower(-1);
        }





        if (gamepad2.x) {
            intake.setPower(-1);

        } else {
            intake.setPower(1);
        }

//ramp servo
        if (gamepad2.left_bumper) {
            rampServo.setPosition(0.3);
        } else {
            rampServo.setPosition(0);
        }
//arm kill switch
        if (gamepad2.b) {
            armMotor.setPower(0);
        }

//reverse switch

        if (!touchSensor2.isPressed() && armMotor.getPower() == 0) {
            if (gamepad2.y) {
                armMotor.setPower(-0.5);

            }
        }

//shooting button

        if (gamepad1.a) {

            shooterMotor2.setPower(1);
            shooterMotor.setPower(1);

        } else {

            shooterMotor2.setPower(0);
            shooterMotor.setPower(0);

        }


        //arm claw controls

        if (gamepad1.right_bumper) {
            leftClaw.setPosition(0.5);
            rightClaw.setPosition(0.5);
        }
        else {
            leftClaw.setPosition(0);
            rightClaw.setPosition(1);
        }



// Gamepad2 controls



        if (gamepad2.a) {

            shooterMotor2.setPower(1);
            shooterMotor.setPower(1);

        } else {

            shooterMotor2.setPower(0);
            shooterMotor.setPower(0);

        }


        //arm claw controls

        if (gamepad2.right_bumper) {
            leftClaw.setPosition(0.5);
            rightClaw.setPosition(0.5);
        }
        else {
            leftClaw.setPosition(0);
            rightClaw.setPosition(1);
        }

        //inverse kinematic transformation
// to convert your joystick inputs to 4 motor commands:
        double mfrontLeft = forward + clockwise + right;
        double mfrontRight = forward - clockwise - right;
        double mbackLeft = forward + clockwise - right;
        double mbackRight = forward - clockwise + right;


        //wheel speed commands
// so that no wheel speed command exceeds magnitude of 1:
        double max = abs(mfrontLeft);
        if (abs(mfrontRight) > max) {
            max = abs(mfrontRight);
        }
        if (abs(mbackLeft) > max) {
            max = abs(mbackLeft);
        }
        if (abs(mbackRight) > max) {
            max = abs(mbackRight);
        }
        if (max > 1) {
            mfrontLeft /= max;
            mfrontRight /= max;
            mbackLeft /= max;
            mbackRight /= max;
        }

        rightFront.setPower(mfrontRight);
        rightRear.setPower(mbackRight);
        leftFront.setPower(mfrontLeft);
        leftRear.setPower(mbackLeft);

        telemetry.addData("lx: ", gamepad1.left_stick_x);
        telemetry.addData("ly: ", gamepad1.left_stick_y);
        telemetry.addData("rx: ", gamepad1.right_stick_x);
        telemetry.addData("ry: ", gamepad1.right_stick_y);

        telemetry.update();
        
    }
}