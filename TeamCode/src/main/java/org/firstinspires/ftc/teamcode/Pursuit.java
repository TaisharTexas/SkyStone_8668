package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sbfUtil.PVector;

@Config
/**
 * Credit:
 *
 * @author Andrew, SBF Robotics
 */
public class Pursuit
{
    // TODO: figure out if there are any variables here that do not need to be used.  Also comment!
    public PVector location;
    public PVector velocity;
    public PVector localVelocity;
    public PVector acceleration;
    public PVector desiredVelocity;
    public PVector initialPosition;
    public double currentHeading;
    public double currentAngularVelocity;
    public double joystickAngularVelocity;

    public static double endZone = 6.0;
    private double maxSpeed;
    private double maxAccel;
    private double gainValue;
    private double turnGain;

    public static double pathLookahead = 5.0;

    private double accelerationSteepness = 4.0;
    private double timeToAccelerate = 1.0;

    private int currentSegment = 0;
    private boolean lastSegment = false;
    private boolean done = false;
    private boolean moving = false;

    Telemetry telemetry;
    public double elapsedTime = 0;
    private PVector end;

    public Pursuit(float x, float y, Telemetry telem)
    {
        telemetry = telem;
        acceleration = new PVector(0, 0);
        velocity = new PVector(0,0);
        localVelocity = new PVector( 0,0);
        location = new PVector(x,y);
        desiredVelocity = new PVector(0,0);
//        endZone = 6; //inch

        //  30 in/sec correlates to maximum unloaded motor speed
        //  30.615 in/sec = 15.7 rad/sec * 1.3 gearing * 1.5 in wheel radius
        maxSpeed = 13; //inches/second

        gainValue = 2.8;
        maxAccel = maxSpeed * gainValue;

        //unit: degrees per second turned -- max turn rate is 343 degrees/sec
        turnGain = 250;

    }

    /**
     * Given a start waypoint and end waypoint, take the robot's current location and move the robot
     * first to the start point and then to the end point.
     */
//    public boolean follow(Path drivePath)
    public void follow(Path drivePath)
    {
        if(!moving)
        {
            moving = true;
        }
        PVector target;
        PVector start = drivePath.pathPoints.get(currentSegment);
        end = drivePath.pathPoints.get(currentSegment + 1);

//        telemetry.addData("Going to: ", end);

        double theMaxSpeed = drivePath.maxSpeeds.get(currentSegment + 1);
        double theTargetHeading = drivePath.targetHeadings.get(currentSegment + 1);

        double radius = theMaxSpeed / 6.0;
        radius = Range.clip(radius,1.0,radius);


        if(theMaxSpeed >= 20 && theMaxSpeed < 26)
        {
            maxAccel = theMaxSpeed *  5;
        }
        else if(theMaxSpeed >= 26)
        {
            maxAccel = theMaxSpeed * 8;
            if (currentSegment == 0)
            {
                maxAccel = maxAccel / (1.0 + Math.exp(-accelerationSteepness * (elapsedTime - timeToAccelerate)));
//                maxAccel = Math.sqrt(elapsedTime * 15.0 * 240);
                maxAccel = Range.clip( maxAccel, 0, 240);
            }
        }
        else
        {
            maxAccel = theMaxSpeed * gainValue;
        }

//        telemetry.addData("v.elapsedTime: ", elapsedTime);
//        telemetry.addData("v.gain: ", maxAccel);
        if(drivePath.pathPoints.size() == currentSegment + 2)
        {
            lastSegment = true;
        }

        telemetry.addData("CurrentSegment, Last Segment?: ", "%d, %s", currentSegment, String.valueOf(lastSegment ));
        PVector velocityCopy = velocity.copy();
        velocityCopy.setMag(2);

        PVector projectedLoc = PVector.add(location, velocityCopy);


        if(projectedLoc.dist(end) < radius && lastSegment)
        {
            target = end.copy();
        }
        else if(projectedLoc.dist(end) < radius)
        {
            currentSegment++;
            start = drivePath.pathPoints.get(currentSegment);
            end = drivePath.pathPoints.get(currentSegment + 1);
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
            pathDirection.setMag(pathLookahead);

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


        if(location.dist(end) < radius && !lastSegment)
        {
            currentSegment++;
        }

        telemetry.addData("Target loc: ", target);
        arrive(target, theMaxSpeed);
        point(theTargetHeading, 175);

        if(lastSegment && location.dist(end) <= 3.5 && (Math.abs(currentHeading)-Math.abs(theTargetHeading)) <= 3)
        {
            done = true;
        }
    }

    private void arrive(PVector target, double theMaxSpeed)
    {
        //converts the 0-1 power scale of the csv file to the 0-30 power scale used by the pursuit algorithm
//        theMaxSpeed = theMaxSpeed * 30.0;

        //find the needed velocity to move to target and call it desiredVelocity
        desiredVelocity = PVector.sub(target, location);
        telemetry.addData("Robot Loc: ", location );
//        telemetry.addData("Desired velocity: ", desiredVelocity);

        //speed is the magnitude of desiredVelocity
        float speed = desiredVelocity.mag();

        if(location.dist(end) < endZone)
        {
            float m = scaleVector(speed, 0, (float)endZone, 1.0f, (float)theMaxSpeed);
            desiredVelocity.setMag(m);
        }
        else
        {
            //set speed to maximum allowed speed
            desiredVelocity.setMag(theMaxSpeed);
        }

//        telemetry.addData("v.desired velocity set maxSpd: ", desiredVelocity);


        // steerAcceleration is the amount of needed change in velocity
        PVector steerAcceleration = PVector.sub(desiredVelocity, velocity);
        telemetry.addData("Robot Velocity: ", velocity);

//        telemetry.addData("v.steerAcceleration: ", steerAcceleration);

        // limit rate of change to robot velocity
        steerAcceleration.limit(maxAccel);
//        telemetry.addData("v.limit steerAcceleration: ", steerAcceleration);

        if (currentSegment == 0 && !lastSegment)
        {
            steerAcceleration.setMag(maxAccel);  // try this
        }

//        telemetry.addData("v.setMag steerAcceleration: ", steerAcceleration);

        //corrects robot velocity by adding error (steerAcceleration)
//        telemetry.addData("v.velocity: ", velocity);
        desiredVelocity = PVector.add(velocity, steerAcceleration);

//        telemetry.addData("Robot Desired velocity: ", desiredVelocity);


        //make sure final velocity isn't too fast
        desiredVelocity.limit(theMaxSpeed);

    }

    private void point(double targetHeading, double theMaxTurnSpeed)
    {
        double desiredAngularVelocity = (targetHeading-currentHeading);

        double slowDown = 30;

        if(Math.abs(desiredAngularVelocity) < slowDown)
        {
            double wantedAngularVelocity = Math.abs(desiredAngularVelocity);
            float m = (float)(theMaxTurnSpeed * (wantedAngularVelocity/slowDown));
            desiredAngularVelocity = m * Math.signum(desiredAngularVelocity);
        }
        else
        {
            desiredAngularVelocity = theMaxTurnSpeed * Math.signum(desiredAngularVelocity);
        }

        telemetry.addData("mp.desiredAngularVel: ", desiredAngularVelocity);

//        telemetry.addData("mp.currentAngularVel: ", currentAngularVelocity);
        double requiredAngularAccel = desiredAngularVelocity - currentAngularVelocity;
        requiredAngularAccel = Range.clip(requiredAngularAccel, -turnGain, turnGain);

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


    private float scaleVector(float value, float start1, float stop1, float start2, float stop2)
    {
        return  start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1));
    }

    public void updatePosition(PVector currentPosition)
    {
        location = PVector.add(location, currentPosition);
    }

    public void updateVelocity(PVector currentVelocity)
    {
        velocity.set(currentVelocity.x, currentVelocity.y);
    }

    public void updateAngularVelocity( double angularVelocity )
    {
        currentAngularVelocity = angularVelocity;
    }

    public void updateHeading( double heading )
    {
        currentHeading = heading;
    }

    public boolean getDone()
    {
//        return !moving;
        return done;
    }


}
