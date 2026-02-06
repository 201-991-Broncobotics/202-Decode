package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Settings {

    // Flywheel config
    // 28 ticks is fine for a GoBILDA 5203/5202-style encoder
    public static double flywheelTicks = 28.0;          // encoder ticks per rev

    // Target RPMs (adjust on the field)
    public static double flywheelVel = 90000;
    public static double flywheelVelBumper = 5000; // short/bumper shot RPM

    // PIDF for RUN_USING_ENCODER
    // Start with KF so it coasts at target speed, then add a bit of KP if needed
    public static double FlywheelKF = 0.015;            // feedforward
    public static double FlywheelKP = 0.0008;
    public static double FlywheelKI = 0.0;
    public static double FlywheelKD = 0.0;

    // Hidden drive/intake offsets
    public static double secretForward = 0.0;
    public static double secretTurn = 0.0;
    public static double secretIntake = 0.0;

    // Angle servo for shooter
    public static double servoAngle = 0.40;

    // Turret PID (if used anywhere)
    public static double turret_P = 0.3;
    public static double turret_I = 0.0;
    public static double turret_D = 0.0002;

    // Limelight mounting offset (degrees)
    public static double limelight_angular_offset = 3.0;

    // Extra flag (if you still need it)
    public static boolean justTurnFlywheelOn = true;
}
