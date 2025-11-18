package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name="Auton", group="Iterative Opmode")
public class BasicAuton extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);

        ElapsedTime mRuntime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        // You can create any variables you need here
        double FrameTime = 0;
        double flywheelVelocity = 0;
        ElapsedTime liftServoTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        boolean justStarted = true;
        double FlywheelTargetVel = 0;
        double flywheelPower = 0;
        boolean FlywheelOn = false, FlywheelJustToggled = false;
        double IntakePower = 0;
        boolean IntakeOn = false, IntakeJustToggled = false;
    }}