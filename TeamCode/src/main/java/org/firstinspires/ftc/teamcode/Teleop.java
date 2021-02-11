package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import static java.lang.Math.abs;

@TeleOp(name="Teleop", group="Teleop")
//@Disabled
public class Teleop extends OpMode
{
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftRear = null;
    DcMotor rightRear = null;
    DcMotor intake ;
    DcMotor rampMotor;
    DcMotor shooterMotor;
    Servo leftClaw;
    Servo rightClaw;


    boolean forwardIntake = true;
    boolean ramp = true;
    boolean shooter = true;






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



        rampMotor = hardwareMap.dcMotor.get("rampMotor");
        rampMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        shooterMotor = hardwareMap.dcMotor.get("shooterMotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        //ServoHardwareMapping

        leftClaw = hardwareMap.servo.get("leftClaw");
        leftClaw.setPosition(0);

        rightClaw = hardwareMap.servo.get("rightClaw");
        rightClaw.setPosition(1);





    }








    public void start(){}

    public void loop() {

        //joystick values x and y on left stick; x only on right stick
        double forward = gamepad1.left_stick_y;     // push left joystick forward to go forward
        double right = -gamepad1.left_stick_x;        // push left joystick to the right to strafe right
        double clockwise = -gamepad1.right_stick_x;   // push right joystick to the right to rotate clockwise

        //Intake reverse 5;

        intake.setPower(1);


        if(gamepad1.x && !forwardIntake) {
            if (intake.getPower() == 1) intake.setPower(-1);
            else intake.setPower(1);
            forwardIntake = true;
        } else if (!gamepad1.x) forwardIntake = false;


         //Ramp motor toggle switch

        rampMotor.setPower(0.3);

        if(gamepad1.b && !ramp) {
            if (rampMotor.getPower() == 0.3) rampMotor.setPower(0);
            else rampMotor.setPower(0.3);
            ramp = true;
        } else if (!gamepad1.b) ramp = false;



        // shooter motor toggle switch

        shooterMotor.setPower(0);

        if(gamepad1.a && !shooter) {
            if (shooterMotor.getPower() == 0) shooterMotor.setPower(1);
            else shooterMotor.setPower(0);
            shooter = true;
        } else if (!gamepad1.a) shooter = false;



        //arm claw controls

        if(gamepad1.right_bumper) {
            leftClaw.setPosition(.5);
            rightClaw.setPosition(.5);
        }  else {
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
// so that no wheel speed command exceeds magnitude of .5:
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
        if (max > 0.5) {
            mfrontLeft/=max;
            mfrontRight/=max;
            mbackLeft/=max;
            mbackRight/=max;
        }
        rightFront.setPower(mfrontRight);
        rightRear.setPower(mbackRight);
        leftFront.setPower(mfrontLeft);
        leftRear.setPower(mbackLeft);

    }


    public void stop()
    {
        rightFront.setPower(0.0);
        leftFront.setPower(0.0);
        rightRear.setPower(0.0);
        leftRear.setPower(0.0);
    }


}