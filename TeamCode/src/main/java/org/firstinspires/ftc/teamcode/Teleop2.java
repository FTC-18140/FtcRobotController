package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDIO;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

@TeleOp(name="Teleop2", group="Teleop")
public class Teleop2 extends OpMode
{
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftRear = null;
    DcMotor rightRear = null;
    DcMotor intake;
    DcMotor shooterMotor2;
    DcMotor shooterMotor;
    Servo leftClaw;
    Servo rightClaw;
    DcMotor armMotor;
    com.qualcomm.robotcore.hardware.TouchSensor touchSensor1;
    TouchSensor touchSensor2;
    DigitalChannel digitalTouch;
    DigitalChannel digitalTouch2;

    boolean forwardIntake;
    boolean shooter2;
    boolean shooter;


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


        leftClaw = hardwareMap.servo.get("leftClaw");
        leftClaw.setPosition(0);

        rightClaw = hardwareMap.servo.get("rightClaw");
        rightClaw.setPosition(1);

        touchSensor1 = hardwareMap.touchSensor.get("touchSensor1");
        touchSensor2 = hardwareMap.touchSensor.get("touchSensor2");


        // get a reference to our digitalTouch object.
         digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
         digitalTouch2 = hardwareMap.get(DigitalChannel.class, "digital_sensor");

        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        digitalTouch2.setMode(DigitalChannel.Mode.INPUT);



    }


    public void start(){}


    public void loop() {


/*
        if (digitalTouch2.getState() == true) {
            telemetry.addData("Touch Digital", "Is Not Pressed");

        } else {
            telemetry.addData("Touch Digital", "Is Pressed");

        }
*/


/*        if (digitalTouch.getState() == false) {
            telemetry.addData("Digital Touch", "Is Not Pressed");
        } else {
            telemetry.addData("Digital Touch", "Is Pressed");
        }

        if (touchSensor2.isPressed()) {
            armMotor.setPower(0);

    }
        if (touchSensor1.isPressed()) {
            armMotor.setPower(0);
if (gamepad1.left_bumper && !touchSensor1.isPressed()) {
            armMotor.setPower(0.3);

        } else if (touchSensor1.isPressed())
            {
            armMotor.setPower(0);
            armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        if (gamepad1.left_bumper && !touchSensor2.isPressed()) {
            armMotor.setPower(0.3);


        } else if (touchSensor2.isPressed()){
            armMotor.setPower(0);
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        }
*/

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


        //joystick values x and y on left stick; x only on right stick
        double forward = gamepad1.left_stick_y;     // push left joystick forward to go forward
        double right = -gamepad1.left_stick_x;        // push left joystick to the right to strafe right
        double clockwise = -gamepad1.right_stick_x;   // push right joystick to the right to rotate clockwise

        if (gamepad1.x && !forwardIntake) {
            if (intake.getPower() == 1) intake.setPower(-1);
            else intake.setPower(1);
            forwardIntake = true;
        } else if (!gamepad1.x) forwardIntake = false;


        if (gamepad1.b)
            armMotor.setPower(0);

        if (!touchSensor2.isPressed() && armMotor.getPower() == 0) {
            if (gamepad1.y) {
                armMotor.setPower(-0.5);

            }
        }

        if (gamepad1.a) {

            shooterMotor2.setPower(1);
            shooterMotor.setPower(1);


        } else {
            shooterMotor2.setPower(0);
            shooterMotor.setPower(0);
        }


        // shooter motor toggle switch






        //arm claw controls

        if (gamepad1.right_bumper) {
            leftClaw.setPosition(0.5);
            rightClaw.setPosition(0.5);
        } else {
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


    }
}