package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Roadrunner.TankDrive;

@Autonomous(name="Auton", group="Autonomous")
public class BasicAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // MeepMeep starting pose
        Pose2d startPose = new Pose2d(62, 12, Math.toRadians(180));

        // Your TankDrive constructor sets the pose internally
        TankDrive drive = new TankDrive(hardwareMap, startPose);

        // DO NOT call setPoseEstimate() — your drive does not have it

        // Build your MeepMeep path using ActionBuilder
        Action fullAuto = drive.actionBuilder(startPose)
                .lineToX(55)
                .turn(Math.toRadians(-20))
                .waitSeconds(6.7)
                .turn(Math.toRadians(20))
                .lineToX(35)
                .turn(Math.toRadians(-90))
                .lineToY(60)
                .lineToY(12)
                .turn(Math.toRadians(90))
                .lineToX(-12)
                .turn(Math.toRadians(-45))
                .build();

        telemetry.addLine("READY");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Run the entire action using RR Actions API
        Actions.runBlocking(fullAuto);
    }
}

