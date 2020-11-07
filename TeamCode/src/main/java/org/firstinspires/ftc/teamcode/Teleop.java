package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static java.lang.Math.abs;

@TeleOp(name="Teleop", group="Teleop")
public class Teleop extends OpMode
{
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftRear = null;
    DcMotor rightRear = null;
    

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










    }
    public void start(){}

    public void loop()
    {


        //joystick values x and y on left stick; x only on right stick
        double forward = gamepad1.left_stick_y;     // push left joystick forward to go forward
        double right = -gamepad1.left_stick_x;        // push left joystick to the right to strafe right
        double clockwise = -gamepad1.right_stick_x;   // push right joystick to the right to rotate clockwise


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

    //Intake section





















    //shooter section






}
