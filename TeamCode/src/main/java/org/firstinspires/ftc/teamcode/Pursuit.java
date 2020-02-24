package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sbfUtil.PVector;


/**
 * Implements a pursuit robot maneuvering algorithm based on the steering behavior algorithm originally
 * published by Craig Reynolds in "Steering Behavior for Autonomous Characters".
 *
 * Takes a Path which describes a series of line segments and computes the deisred velocity for the
 * robot to follow the path and traverse the segments, finally arriving at its goal.  The robot
 * is not following a predetermined path, rather, it is computing needed velocities to point the robot
 * towards the line segment which leads to the next point in the path.
 *
 * The concepts in this class are most directly influenced by Daniel Shiffman and his presentations
 * on the Nature of Code, specifically chapter 6 on Autonomous Agents.
 *
 * @see <a href="https://natureofcode.com/book/chapter-6-autonomous-agents/">Autonomous Agents by Daniel Shiffman</a>
 * @see <a href="https://youtu.be/2qGsBClh3hE">Great video lecture on Path Following by Daniel Shiffman</a>
 * @see <a href="http://www.red3d.com/cwr/papers/1999/gdc99steer.pdf">Steering Bheavior for Autonomous Characters</a>
 *
 * @author Andrew, 8668 Should Be Fine!
 */
@Config
public class Pursuit
{
    /** The robot's current location. */
    public PVector location;
    /** The robot's current velocity. */
    public PVector velocity;
    /** The desired direction and length of travel. */
    public PVector desiredVelocity;
    /** The current heading of the robot */
    public double currentHeading;
    /** The current rate of directional change and direction of change. */
    public double currentAngularVelocity;
    /** The angular velocity the robot needs to have converted to a joystick command. */
    public double joystickAngularVelocity;
    /** The minimum distance away from the target point before the robot can switch path segments. */
    public static double endZone = 10.0;
    /** The max speed the robot can drive at -- used to calculate maxAccel */
    private double maxSpeed;
    /** A measure of how quickly the robot can change velocity. */
    private double maxAccel;
    /** A gain dictating how rapidly the robot can turn. */
    public static double turnGain = 50;
    /** A gain dictating how rapidly the robot can turn. */
    public static double turnEndZone = 55;
    /** How far ahead the robot projects its location -- allows the robot to respond to changes in
     * the path before arriving at the actual change. Makes movements smooth. */
    public static double pathLookahead = 6.0;
    /** Maximum rate the robot can turn at. */
    public static double maxTurnSpd = 350;
    /** Maximum rate the robot can turn at. */
    private double maxTurnAccel;
    /** Used when calculating maxAccel.*/
    public static double maxFastAccelGain = 20;
    /** Used when calculating maxAccel.*/
    public static double maxMedAccelGain = 15;
    /** Used when calculating maxAccel.*/
    public static double maxLowAccelGain = 10;
    /** Used on the first segment of a path to determine how quickly the robot can accelerate from zero. */
    private double accelerationSteepness = 4.0;
    /** How long the robot can take to accelerate -- used when determining how steeply the robot
     * can accelerate from zero. */
    private double timeToAccelerate = 1.0;
    /** The current path segment the robot is on. */
    public int currentSegment = 0;
    /** Marks whether or not the current segment is the last segment of the path. */
    private boolean lastSegment = false;
    /** Marks whether or not the robot has completed its move.*/
    private boolean done = false;
    /** A telemetry object passed down from the opmode.*/
    Telemetry telemetry;
    /** The time elapsed since the variable was last reset. */
    public double elapsedTime = 0;
    /** */
    private PVector end;
    /** The ID for any secondary actions that need to be loaded to the runmap simultaneously with the path points.*/
    public String auxAction;

    /**
     * Class constructor. Sets any variables that require a value before pursuit starts.
     * @param x The start x position.
     * @param y The start y position.
     * @param telem A telemetry object passed down from the opmode.
     */
    public Pursuit(float x, float y, Telemetry telem)
    {
        telemetry = telem;
//        acceleration = new PVector(0, 0);
        velocity = new PVector(0,0);
//        localVelocity = new PVector( 0,0);
        location = new PVector(x,y);
        desiredVelocity = new PVector(0,0);
//        endZone = 6; //inch

        //  30 in/sec correlates to maximum unloaded motor speed
        //  30.615 in/sec = 15.7 rad/sec * 1.3 gearing * 1.5 in wheel radius
        maxSpeed = 13; //inches/second

        maxAccel = maxSpeed * maxMedAccelGain;

        //unit: degrees per second turned -- max turn rate is 343 degrees/sec
        maxTurnAccel = maxTurnSpd * turnGain;

    }

    /**
     * Use robot's current location and move the robot along the path starting from the start point
     * and moving to the end point.
     * @param drivePath contains the points which the robot should follow
     */
    public void follow(Path drivePath)
    {
        PVector target;
        PVector start = drivePath.pathPoints.get(currentSegment);
        end = drivePath.pathPoints.get(currentSegment + 1);

//        telemetry.addData("Going to: ", end);

        double theMaxSpeed = drivePath.maxSpeeds.get(currentSegment + 1);
        double theTargetHeading = drivePath.targetHeadings.get(currentSegment + 1);

        double radius = theMaxSpeed / 6.0;
        radius = Range.clip(radius,1.0,radius);
        radius = endZone;
        auxAction = "NULL";

        /**
         * Determine the maximum acceleration by basing it off the maximum speed.  However, if
         * we are accelerating from stop on the first segment, use a smoother acceleration value
         * that comes from the exponential calculation.
         */
        if(theMaxSpeed >= 20 && theMaxSpeed < 26)
        {
            maxAccel = theMaxSpeed * maxMedAccelGain;
        }
        else if(theMaxSpeed >= 26)
        {
            maxAccel = theMaxSpeed * maxFastAccelGain;
            if (currentSegment == 0)
            {
                maxAccel = maxAccel / (1.0 + Math.exp(-accelerationSteepness * (elapsedTime - timeToAccelerate)));
                maxAccel = Range.clip( maxAccel, 0, 35*maxFastAccelGain);
            }
        }
        else
        {
            maxAccel = theMaxSpeed * maxLowAccelGain;
        }
        maxTurnAccel = maxTurnSpd * turnGain;

//        telemetry.addData("v.elapsedTime: ", elapsedTime);
//        telemetry.addData("v.gain: ", maxAccel);

        /**
         * Determine if I am targeting the last point, then set a flag which indicates I am on the
         * last segment.  This has implications for when I get to the end.... I need to slow down
         * and stop, since it is the last point.
         */
        if(drivePath.pathPoints.size() == currentSegment + 2)
        {
            lastSegment = true;
        }
        telemetry.addData("CurrentSegment, Last Segment?: ", "%d, %s", currentSegment, String.valueOf(lastSegment ));

        /**
         * 1. Calculate a Projected Future Location
         *
         * Do this by taking the current location and current velocity and calculating a
         * projected location a few inches out from the current location in the direction the current
         * velocity is pointing.
         */
        PVector velocityCopy = velocity.copy();
        velocityCopy.setMag(pathLookahead);

        PVector projectedLoc = PVector.add(location, velocityCopy);

        /**
         * 2. Calculate the Desired Target Location to shoot for
         *
         * Do this by taking the Projected Location and projecting it onto the current path segment.
         * This projection is the point on the path segment which is closest to the Projected Location
         * calculated in the previous step.  This point is called the Normal Point since it represents
         * the point on the path which is perpendicular to the Projected Location of the robot.
         *
         * Once this Normal Point has been calculated, shift this point along the path towards the
         * endpoint to give the robot a nice target ahead to shoot for.
         *
         * There are a few cases that have to be checked when the Normal Point is close to the
         * start and end of the path segment.  For example, if the Normal Point is close to the
         * end point, it's time to switch to the next segment.  Also, if the Normal Point is calculated
         * to be outside of the segment defined by start and end, it must be corrected.
         */

        if(projectedLoc.dist(end) < radius && lastSegment)
        {
            target = end.copy();

        }
        else if(projectedLoc.dist(end) < radius)
        {
            currentSegment++;
            start = drivePath.pathPoints.get(currentSegment);
            end = drivePath.pathPoints.get(currentSegment + 1);
            auxAction = drivePath.auxActions.get(currentSegment);

            theMaxSpeed = drivePath.maxSpeeds.get(currentSegment + 1);
            theTargetHeading = drivePath.targetHeadings.get(currentSegment + 1);

            PVector normalPoint = getNormalPoint(projectedLoc, start, end);

            if(normalPoint.dist(end) > start.dist(end))
            {
                normalPoint = start;
            }
            else if(normalPoint.dist(start) > end.dist(start))
            {
                normalPoint = end;
            }

            PVector pathDirection = PVector.sub(end, start);
            pathDirection.setMag(2);

            target = normalPoint.copy();
            target.add(pathDirection);

        }
        else
        {
            PVector normalPoint = getNormalPoint(projectedLoc, start, end);

            if(normalPoint.dist(end) > start.dist(end))
            {
                normalPoint = start;
            }
            else if(normalPoint.dist(start) > end.dist(start))
            {
                normalPoint = end;
            }

            PVector pathDirection = PVector.sub(end, start);
            pathDirection.setMag(pathLookahead);

            target = normalPoint.copy();
            target.add(pathDirection);
        }
        telemetry.addData("ProjectedLoc: ", projectedLoc);
        telemetry.addData("Distance Proj to End: ", projectedLoc.dist(end));


        if(location.dist(end) < radius && !lastSegment)
        {
            currentSegment++;
        }

        telemetry.addData("Target loc: ", target);

        /**
         * 3.  Calcualte the Steering Vector to achieve the Target Location
         */
        arrive(target, theMaxSpeed);

        /**
         * 4.  Calculate the Rotation Speed to achieve the Target Heading
         */
        point(theTargetHeading, maxTurnSpd);
        telemetry.addData("end: ", end);

        telemetry.addData("Distance to end: ", location.dist(end));

        if(lastSegment && location.dist(end) <= 3.5 && (Math.abs(currentHeading)-Math.abs(theTargetHeading)) <= 2)
        {
            done = true;
        }
    }

    /**
     * Calculate the needed linear velocity to arrive at the target.
     * @param target The target the linear velocity needs to point towards
     * @param theMaxSpeed The max speed the robot can use to get to the target.
     */
    private void arrive(PVector target, double theMaxSpeed)
    {

        /**
         * Find the needed velocity to move to target and call it desiredVelocity.  This is calculated
         * by subtraction my current vector location from the target vector location.  We care calling
         * this velocity because it represents a change in position we desire to achieve over the
         * next period of time.
         */
        desiredVelocity = PVector.sub(target, location);
        telemetry.addData("Robot Loc: ", location );
//        telemetry.addData("Desired velocity: ", desiredVelocity);

        //speed is the magnitude of desiredVelocity
        float speed = desiredVelocity.mag();

        /**
         * If the robot is close to the end of the segment, slow down by scaling the desiredVelocitiy
         * based on the distance to the end.
         */
        if(location.dist(end) < endZone)
        {
            float m = scaleVector(speed, 0, (float)endZone, 8.0f, (float)theMaxSpeed);
            desiredVelocity.setMag(m);
        }
        else
        {
            //set speed to maximum allowed speed
            desiredVelocity.setMag(theMaxSpeed);
        }

//        telemetry.addData("v.desired velocity set maxSpd: ", desiredVelocity);

        /**
         *  Find the amount of velocity change is needed and call it steerAcceleration.  This is
         *  calculated by taking the desiredVelocity to reach the target location and subtract the
         *  robot's current velocity.  We care calling this acceleration because it represents a
         *  change in velocity we desire to achieve over the next period of time.
         */
        PVector steerAcceleration = PVector.sub(desiredVelocity, velocity);
        telemetry.addData("Robot Velocity: ", velocity);

//        telemetry.addData("v.steerAcceleration: ", steerAcceleration);

        // limit rate of change to robot velocity
        steerAcceleration.limit(maxAccel);
//        telemetry.addData("v.limit steerAcceleration: ", steerAcceleration);

        if (currentSegment == 0 && !lastSegment)
        {
            steerAcceleration.setMag(maxAccel);
        }

//        telemetry.addData("v.setMag steerAcceleration: ", steerAcceleration);

        /**
         * Corrects robot velocity by adding our steerAcceleration to it.  This results in a new
         * desiredVelocity that is changed to help the robot move towards the target location.
         */
//        telemetry.addData("v.velocity: ", velocity);
        desiredVelocity = PVector.add(velocity, steerAcceleration);

//        telemetry.addData("Robot Desired velocity: ", desiredVelocity);

        //make sure final velocity isn't too fast
        desiredVelocity.limit(theMaxSpeed);

    }

    /**
     * Calculates the needed angular acceleration to arrive at the target heading by the end of the segment.
     * @param targetHeading The heading desired at the end of the move.
     * @param theMaxTurnSpeed The max rate the robot can turn.
     */
    private void point(double targetHeading, double theMaxTurnSpeed)
    {
        double desiredAngularVelocity = (targetHeading-currentHeading);

        if(Math.abs(desiredAngularVelocity) < turnEndZone)
        {
            double wantedAngularVelocity = Math.abs(desiredAngularVelocity);
            float m = (float)(theMaxTurnSpeed * (wantedAngularVelocity/turnEndZone));
            m = Range.clip(m, 5, (float)theMaxTurnSpeed);
            desiredAngularVelocity = m * Math.signum(desiredAngularVelocity);
        }
        else
        {
            desiredAngularVelocity = theMaxTurnSpeed * Math.signum(desiredAngularVelocity);
        }

//        telemetry.addData("mp.desiredAngularVel: ", desiredAngularVelocity);

//        telemetry.addData("mp.currentAngularVel: ", currentAngularVelocity);
        double requiredAngularAccel = desiredAngularVelocity - currentAngularVelocity;
        requiredAngularAccel = Range.clip(requiredAngularAccel, -maxTurnAccel, maxTurnAccel);

//        telemetry.addData("mp.requiredAngularAccel: ", requiredAngularAccel);

        joystickAngularVelocity = currentAngularVelocity + requiredAngularAccel;
        telemetry.addData("mp.joystickAngularVelocity: ", joystickAngularVelocity);

    }

    /**
     * A function to get the normal point from a point (p) to a line segment (a-b)
     * This function could be optimized to make fewer new Vector objects
     *
     * @param projectedLoc  The projected location of the robot. Calculated by subtracting the
     *                      projected location from the starting position and subtracting the target
     *                      position from the start position -- then find the dot product of the
     *                      two values.
     * @param segmentStart  The first waypoint. Generally the point where the robot starts from,
     *                      although this is not necessarily the case.
     * @param segmentEnd  The target waypoint to which the robot is attempting to drive.
     * @return The target direction the robot will drive at to move towards the end point.
     * */
    private PVector getNormalPoint(PVector projectedLoc, PVector segmentStart, PVector segmentEnd) {
        // Vector from a to p
        PVector ap = PVector.sub(projectedLoc, segmentStart);
        // Vector from a to b
        PVector ab = PVector.sub(segmentEnd, segmentStart);
        ab.normalize(); // Normalize the line
        // Project vector "diff" onto line by using the dot product
        ab.mult(ap.dot(ab));
        PVector normalPoint = PVector.add(segmentStart, ab);
        return normalPoint;
    }


    /**
     * Scales a vector by the imputed parameters.
     * @param value The thing being scaled
     * @param start1
     * @param stop1
     * @param start2
     * @param stop2
     * @return The scaled vector
     */
    private float scaleVector(float value, float start1, float stop1, float start2, float stop2)
    {
        return  start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1));
    }

    /**
     * Updates the robot's current location.
     * @param currentPosition The position being added to the location vector.
     */
    public void updatePosition(PVector currentPosition)
    {
        location = PVector.add(location, currentPosition);
    }

    /**
     * Updates the robot's velocity.
     * @param currentVelocity The velocity vector being set.
     */
    public void updateVelocity(PVector currentVelocity)
    {
        velocity.set(currentVelocity.x, currentVelocity.y);
    }

    /**
     * Updates the robot's angular velocity.
     * @param angularVelocity The angular velocity being set.
     */
    public void updateAngularVelocity( double angularVelocity )
    {
        currentAngularVelocity = angularVelocity;
    }

    /**
     * Updates the robot's current heading.
     * @param heading The heading being set.
     */
    public void updateHeading( double heading )
    {
        // check to see if the heading changes unrealistically due to gyro rollover
        if (Math.abs(heading - currentHeading) > 225)
        {
            if (heading < 0)
            {
                heading += 360;
            }
            else if (heading > 0)
            {
                heading -= 360;
            }
        }
        currentHeading = heading;
    }

    /**
     * Marks whether or not pursuit has reported complete.
     * @return True: pursuit is done -- False: pursuit is not done
     */
    public boolean getDone()
    {
        return done;
    }


}
