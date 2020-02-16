package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.sbfUtil.PVector;

import java.lang.reflect.Array;
import java.util.ArrayList;

/**
 * Represents the data necesary for a path that Pursuit will follow.
 *
 * @author Andrew, 8668 Should Be Fine!
 */
public class Path
{
    /**
     * The x,y PVector points that defines the locations that the robot should visit.  A pair of
     * PVectors represents the endpoints for a segment.
     */
    ArrayList<PVector> pathPoints;
    /**
     * The speed the robot should use for each path segment.
     */
    ArrayList<Double> maxSpeeds;
    /**
     * The heading the robot should seek for each path segment.
     */
    ArrayList<Double> targetHeadings;
    /**
     * The ID for the other actions that should be kicked off when each segment is loaded.
     */
    ArrayList<String> auxActions;

    /**
     * Default constructor.  Creates the ArrayLists necessary to hold all of the items for each
     * point on the path.
     */
    public Path()
    {
        pathPoints = new ArrayList<PVector>();
        maxSpeeds = new ArrayList<Double>();
        targetHeadings = new ArrayList<Double>();
        auxActions = new ArrayList<String>();
    }

    /**
     * Adds a point to the Path to be used by the Pursuit class.  This creates another segment.
     *
     * @param x
     * @param y
     * @param maxSpeed
     * @param desiredHeading
     * @param auxAction
     */
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
