package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Roadrunner.TankDrive;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Blue", group="Autonomous")
public class Blue_Auton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);
        ElapsedTime clock = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        ElapsedTime servoClock = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        boolean justStarted = true;

        // MeepMeep starting pose
        Pose2d startPose = new Pose2d(62, 12, Math.toRadians(180));

        // Your TankDrive constructor sets the pose internally
        TankDrive drive = new TankDrive(hardwareMap, startPose);

        // DO NOT call setPoseEstimate() — your drive does not have it

        // Build your MeepMeep path using ActionBuilder
        Action fullAuto = drive.actionBuilder(startPose)
                .lineToX(12)
                .turn(Math.toRadians(32))
                .build();

        telemetry.addLine("READY");
        telemetry.update();

        waitForStart();

        clock.startTime();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {

            // Run the entire action using RR Actions API
            Actions.runBlocking(fullAuto);
            LLResult llResult = robot.limelight.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                double tx = llResult.getTx(); // horizontal offset
                tx += Settings.limelight_angular_offset; // limelight mount angular offset
            }

            robot.angle.setPosition(Settings.servoAngle);
            robot.intake.setPower(-1);
            robot.diffy.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.flywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Settings.FlywheelKP, Settings.FlywheelKI, Settings.FlywheelKD, 0));
            robot.flywheel.setVelocity(Settings.flywheelVel / 60.0 * 24);
            if (clock.time(TimeUnit.SECONDS) > 5) {
                justStarted = false;
                servoClock.startTime();
                if (servoClock.time(TimeUnit.SECONDS) >= 1) {
                    robot.lift.setPower(1);
                    robot.intake.setPower(0);
                }
                if(servoClock.time(TimeUnit.SECONDS) >= 2){
                    robot.lift.setPower(-5);
                    robot.intake.setPower(0);
                }

                /*if (servoClock.time(TimeUnit.SECONDS) < 0.8 && !justStarted) {
                    robot.lift.setPower(-0.25);
                    robot.intake.setPower(0);
                    // robot.lift.setPosition(Settings.ServoTopPosition);
                } else if (servoClock.time(TimeUnit.SECONDS) < 1.8 && !justStarted) {
                    robot.lift.setPower(0.25);
                    robot.intake.setPower(0);
                    // robot.lift.setPosition(Settings.ServoBottomPosition);
                } else robot.lift.setPower(0);
                */

            }
        }
    }
}

