package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Vehicle;
import org.firstinspires.ftc.teamcode.PVector;
import java.text.DecimalFormat;

public class ComputerDebugging
{
    //this is what actually sends our messages
    private static UdpServer udpServer;
    //this is what we will use to build the messages
    private static StringBuilder messageBuilder = new StringBuilder();

    //use this to format decimals
    private static DecimalFormat df = new DecimalFormat("#.00");

    /**
     * Initializes udp server and starts it's thread
     */
    public ComputerDebugging()
	{
        UdpServer.kill = false;
        udpServer = new UdpServer(18668);
        Thread runner = new Thread(udpServer);
        runner.start();//go go go
    }



    /**
     * Sends the robot location to the debug computer
     */
    public static void sendRobotLocation(Vehicle vehicle)
	{
        //return if not using computer
//        if(!Robot.usingComputer){return;}

        //first send the robot location
        messageBuilder.append("ROBOT,");
        messageBuilder.append(df.format(vehicle.location.x));
        messageBuilder.append(",");
        messageBuilder.append(df.format(vehicle.location.y));
        messageBuilder.append(",");
        messageBuilder.append(df.format(0)); // REPLACE with HEADING
        messageBuilder.append("%");

    }

    /**
     * Sends the robot location to the debug computer
     */
//    public static void sendRobotLocation(Vehicle car){
//        //return if not using computer
//        if(!Robot.usingComputer){return;}
//
//        //first send the robot location
//        messageBuilder.append("CAR,");
//        messageBuilder.append(df.format(car.getX()));
//        messageBuilder.append(",");
//        messageBuilder.append(df.format(car.getY()));
//        messageBuilder.append(",");
//        messageBuilder.append(df.format(car.getHeading()));
//        messageBuilder.append("%");
//
//    }


    /**
     * Sends the location of any point you would like to send
     * @param thePoint the point you want to send
     */
    public static void sendKeyPoint(PVector thePoint)
	{
//        if(!Robot.usingComputer){return;}

        messageBuilder.append("P,")
                .append(df.format(thePoint.x))
                .append(",")
                .append(df.format(thePoint.y))
                .append("%");
    }


    /**
     * This is a point you don't want to clear every update
     * @param thePoint the point you want to send
     */
//    public static void sendLogPoint(PVector thePoint)
//	{
////        if(!Robot.usingComputer){return;}
//
//        messageBuilder.append("LP,")
//                .append(df.format(thePoint.x))
//                .append(",")
//                .append(df.format(thePoint.y))
//                .append("%");
//    }

    /**
     * This is a point you don't want to clear every update
     * @param thePoint the point you want to send
     */
    public static void sendLogPoint(PVector thePoint)
	{
//        if(!Robot.usingComputer){return;}

        messageBuilder.append("LP,")
                      .append(df.format(thePoint.x))
                      .append(",")
                      .append(df.format(thePoint.y))
                      .append("%");
    }



    /**
     * Used for debugging lines
     * @param point1
     * @param point2
     */
    public static void sendLine(PVector point1, PVector point2)
	{
        //return if not using the computer
//        if(!Robot.usingComputer){return;}
        messageBuilder.append("LINE,")
                .append(df.format(point1.x))
                .append(",")
                .append(df.format(point1.y))
                .append(",")
                .append(df.format(point2.x))
                .append(",")
                .append(df.format(point2.y))
                .append("%");
    }


    /**
     * This kills the udpServer background thread
     */
    public static void stopAll()
	{
//        if(!Robot.usingComputer){return;}

        UdpServer.kill = true;
    }

    /**
     * Sends the data accumulated over the update by adding it to the udpServer
     */
    public static void markEndOfUpdate()
	{
//        if(!Robot.usingComputer){return;}
        messageBuilder.append("CLEAR,%");

//        udpServer.addMessage(messageBuilder.toString());
        udpServer.splitAndSend(messageBuilder.toString());
        messageBuilder = new StringBuilder();
    }

    /**
     * Forces a clear log
     */
    public static void clearLogPoints()
	{
//        if(!Robot.usingComputer){return;}
        udpServer.splitAndSend("CLEARLOG,%");

    }
}
