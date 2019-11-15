package org.firstinspires.ftc.teamcode.sbfActions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;

public class FoundationAction extends RobotAction
{
    double thePosition;

    FoundationAction(String id, String nextAction, double duration, double position)
    {
        super(id, nextAction, duration);

        thePosition = position;
    }

    FoundationAction(String[] params)
    {
        this(params[0],
             params[1],
             Double.parseDouble(params[2]),
             Double.parseDouble(params[3]));
    }

    @Override
    public void init(Telemetry telem, Robot theRobot)
    {
        super.init(telem, theRobot);
    }

    @Override
    public void entry()
    {
        super.entry();
    }

    @Override
    public boolean execute()
    {
        return robot.foundationDrive(thePosition);
    }

    @Override
    public void exit()
    {
        robot.stop();
        super.exit();
    }
}
