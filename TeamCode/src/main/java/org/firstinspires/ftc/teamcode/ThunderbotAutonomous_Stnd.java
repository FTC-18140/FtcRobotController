package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="Autonomous", group="Thunderbot")
public class ThunderbotAutonomous_Stnd extends OpMode
{
    Thunderbot robot = new Thunderbot();
    int state = 0;

    /** Directional variables used to simulate joystick commands in autonomous.
     * Simulates a forward drive command.*/
    static final double FORWARD = 0.0;
    /** Directional variables used to simulate joystick commands in autonomous.
     * Simulates a backward drive command.*/
    static final double BACKWARD = 180.0;
    /** Directional variables used to simulate joystick commands in autonomous.
     * Simulates a right strafe command.*/
    static final double RIGHT = 270.0;
    /** Directional variables used to simulate joystick commands in autonomous.
     * Simulates a left strafe command.*/
    static final double LEFT = 90.0;

    public void init()
    {
        robot.init(hardwareMap, telemetry);
    }

    public void start(){}

    public void loop()
    {
        switch (state)
        {
            case 0:
                if(robot.drive(.5, FORWARD, 18, 3))
                {
                    robot.stopChassis();
                    state++;
                    break;
                }

            case 1:
                if(robot.pointTurn(.25, 90, 3))
                {
                    robot.stopChassis();
                    state++;
                    break;
                }

            case 2:
                if(robot.drive(.6, LEFT, 18, 4))
                {
                    robot.stopChassis();
                    state++;
                    break;
                }
        }
    }

    public void stop()
    {
        robot.stopChassis();
        robot.stopIntake();
    }

}
