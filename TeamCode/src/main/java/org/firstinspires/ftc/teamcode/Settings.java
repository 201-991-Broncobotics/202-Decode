package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config // makes all of these variables show up on the ftc dashboard and lets you edit them while the robot is on without changing code
public class Settings {

    public static double flywheelVel = 37500; // max target Flywheel RPM

    public static double flywheelVel2 = 37500; // max target Flywheel RPM
    public static boolean justTurnFlywheelOn = true;
    public static double secretForward = 0;
    public static double secretTurn = 0;
    public static double secretIntake = 0;
    public static double FlywheelKP = 1.2, FlywheelKI = 0, FlywheelKD = 0.001;
    //Ki Should remain 0, KP should be like 0.5-2 ish, KD sshould be like 0.0001 or something like that
    public static double servoAngle = 0.39;
    public static double servoAngle2 = 0.39;

}
