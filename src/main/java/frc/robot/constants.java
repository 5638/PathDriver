package frc.robot;

public class constants{
    //timeout
    public static final int t = 10;

    //heading tolerance
    public static final double kToleranceDegrees = 2.0f; 

    //drivetrain pidf
    public static final double dtkP = 0.9;
    public static final double dtkI = 0;
    public static final double dtkD = 0;
    public static final double dtkF = 0;

    //elevator pidf
    public static final double ekP = 0.5;
    public static final double ekI = 0;
    public static final double ekD = 10;
    public static final double ekF = 0;

    //dump pidf
    public static final double dkP = 0.25;
    public static final double dkI = 0;
    public static final double dkD = 0;
    public static final double dkF = 0;
}