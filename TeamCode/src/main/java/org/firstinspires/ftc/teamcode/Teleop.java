package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import static java.lang.Math.abs;

@TeleOp(name="Teleop", group="Teleop")
//@Disabled
public class Teleop extends LinearOpMode {
    Thunderbot robot = new Thunderbot();

    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftRear = null;
    DcMotor rightRear = null;



    boolean forwardIntake;
    boolean ramp;
    boolean shooter;

    @Override
    public void runOpMode() {


        robot.init(hardwareMap, telemetry);



        robot.start();

        robot.loop();

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

        //Intake reverse


        if (gamepad1.x && !forwardIntake) {
            if (robot.intake.getPower() == 1) robot.intake.setPower(-1);
            else robot.intake.setPower(1);
            forwardIntake = true;
        } else if (!gamepad1.x) forwardIntake = false;


        //Ramp motor toggle switch


        robot.rampMotor.setPower(0.3);

        if (gamepad1.b && !ramp) {
            if (robot.rampMotor.getPower() == 0.3) robot.rampMotor.setPower(0);
            else robot.rampMotor.setPower(0.3);
            ramp = true;
        } else if (!gamepad1.b) ramp = false;


        // shooter motor toggle switch

       robot.shooterMotor.setPower(0);

        if (gamepad1.a && !shooter) {
            if (robot.shooterMotor.getPower() == 0) robot.shooterMotor.setPower(1);
            else robot.shooterMotor.setPower(0);
            shooter = true;
        } else if (!gamepad1.a) shooter = false;


        //arm claw controls

        if (gamepad1.right_bumper) {
            robot.leftClaw.setPosition(.5);
            robot.rightClaw.setPosition(.5);
        } else {
            robot.leftClaw.setPosition(0);
            robot.rightClaw.setPosition(1);
        }




        robot.stop();

        

    }



}