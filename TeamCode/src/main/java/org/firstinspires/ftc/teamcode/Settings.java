package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config // makes all of these variables show up on the ftc dashboard and lets you edit them while the robot is on without changing code
public class Settings {

    public static double flywheelVel = 1500; // max target Flywheel RPM
    public static boolean justTurnFlywheelOn = false;
    public static double secretForward = 0;
    public static double secretTurn = 0;
    public static double secretIntake = 0;

    public static double FlywheelKP = 0, FlywheelKI = 0, FlywheelKD = 0;

}
