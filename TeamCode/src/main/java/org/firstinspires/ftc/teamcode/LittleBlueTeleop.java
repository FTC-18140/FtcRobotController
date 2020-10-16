package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Little Blue Teleop", group="Teleop")
public class LittleBlueTeleop extends OpMode
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
        rightFront.setPower(gamepad1.right_stick_y);
        rightRear.setPower(gamepad1.right_stick_y);
        leftFront.setPower(gamepad1.left_stick_y);
        leftRear.setPower(gamepad1.left_stick_y);
    }
    public void stop()
    {
        rightFront.setPower(0.0);
        leftFront.setPower(0.0);
        rightRear.setPower(0.0);
        leftRear.setPower(0.0);
    }
}
