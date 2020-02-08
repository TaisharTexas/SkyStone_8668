package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.sbfUtil.PVector;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class Path
{
    ArrayList<PVector> pathPoints;
    ArrayList<Double> maxSpeeds;
    ArrayList<Double> targetHeadings;
    ArrayList<String> auxActions;


    public Path()
    {
        pathPoints = new ArrayList<PVector>();
        maxSpeeds = new ArrayList<Double>();
        targetHeadings = new ArrayList<Double>();
        auxActions = new ArrayList<String>();
    }

    public void addPoint(float x, float y, double maxSpeed, double desiredHeading, String auxAction)
    {
        PVector point = new PVector(x, y);
        pathPoints.add(point);
        maxSpeeds.add(maxSpeed);
        targetHeadings.add(desiredHeading);
        if(auxAction.isEmpty())
        {
            auxAction = "NULL";
        }
        auxActions.add(auxAction);
    }
}
