package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Teleop", group="Teleop")
public class Teleop extends OpMode
{
    Thunderbot robot = new Thunderbot();

    boolean forwardIntake = true;
    boolean ramp = true;
    boolean shooter = true;


    // Executes once when the init button is pressed
    public void init()
    {

        robot.init(hardwareMap, telemetry);

    }


    // Executes once directly after the start button is pressed
    public void start(){}

    // Executes in an infinite loop until the stop button is pressed
    public void loop() {

        // Intake Controls
        //Intake reverse 5;
        robot.intake.setPower(1);

        if(gamepad1.x && !forwardIntake)
        {
            if (robot.intake.getPower() == 1)  { robot.intake.setPower(-1); }
            else  { robot.intake.setPower(1); }
            forwardIntake = true;
        }
        else if (!gamepad1.x)  { forwardIntake = false; }


         //Ramp motor toggle switch
        robot.rampMotor.setPower(0.3);

        if(gamepad1.b && !ramp)
        {
            if (robot.rampMotor.getPower() == 0.3)  { robot.rampMotor.setPower(0); }
            else  { robot.rampMotor.setPower(0.3); }
            ramp = true;
        }
        else if (!gamepad1.b)  { ramp = false; }



        // shooter motor toggle switch
        robot.shooterMotor.setPower(0);

        if(gamepad1.a && !shooter)
        {
            if (robot.shooterMotor.getPower() == 0)  { robot.shooterMotor.setPower(1); }
            else  { robot.shooterMotor.setPower(0); }
            shooter = true;
        }
        else if (!gamepad1.a)  { shooter = false; }



        //arm claw controls
        if(gamepad1.right_bumper)
        {
            robot.leftClaw.setPosition(.5);
            robot.rightClaw.setPosition(.5);
        }
        else
        {
            robot.leftClaw.setPosition(0);
            robot.rightClaw.setPosition(1);
        }

        // chassis controls
        robot.joystickDrive(gamepad1.left_stick_x,
                            gamepad1.left_stick_y,
                            gamepad1.right_stick_x,
                            gamepad1.right_stick_y,
                           1);

    }


    // Executes once when the stop button is pressed
    public void stop()
    {
        robot.stopChassis();
    }

}
