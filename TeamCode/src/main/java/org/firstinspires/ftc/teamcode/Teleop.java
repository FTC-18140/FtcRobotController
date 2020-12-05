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
    DcMotor intake ;
    DcMotor rampMotor;
    DcMotor shooterMotor;
    Servo leftClaw;
    Servo rightClaw;

    boolean forwardIntake;
    boolean ramp;
    boolean shooter;

    @Override
    public void runOpMode() {


        robot.init(hardwareMap, telemetry);

        public void start () {
        }

        public void loop () {

            //joystick values x and y on left stick; x only on right stick
            double forward = gamepad1.left_stick_y;     // push left joystick forward to go forward
            double right = -gamepad1.left_stick_x;        // push left joystick to the right to strafe right
            double clockwise = -gamepad1.right_stick_x;   // push right joystick to the right to rotate clockwise

            //Intake reverse

            intake.setPower(1);

            if (gamepad1.x && !forwardIntake) {
                if (intake.getPower() == 1) intake.setPower(-1);
                else intake.setPower(1);
                forwardIntake = true;
            } else if (!gamepad1.x) forwardIntake = false;


            //Ramp motor toggle switch


            rampMotor.setPower(0.3);

            if (gamepad1.b && !ramp) {
                if (rampMotor.getPower() == 0.3) rampMotor.setPower(0);
                else rampMotor.setPower(0.3);
                ramp = true;
            } else if (!gamepad1.b) ramp = false;


            // shooter motor toggle switch

            shooterMotor.setPower(0);

            if (gamepad1.a && !shooter) {
                if (shooterMotor.getPower() == 0) shooterMotor.setPower(1);
                else shooterMotor.setPower(0);
                shooter = true;
            } else if (!gamepad1.a) shooter = false;


            //arm claw controls

            if (gamepad1.right_bumper) {
                leftClaw.setPosition(.5);
                rightClaw.setPosition(.5);
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
                mfrontLeft /= max;
                mfrontRight /= max;
                mbackLeft /= max;
                mbackRight /= max;
            }
            rightFront.setPower(mfrontRight);
            rightRear.setPower(mbackRight);
            leftFront.setPower(mfrontLeft);
            leftRear.setPower(mbackLeft);

        }


        public void stop ()
        {
            rightFront.setPower(0.0);
            leftFront.setPower(0.0);
            rightRear.setPower(0.0);
            leftRear.setPower(0.0);
        }

    }
}
