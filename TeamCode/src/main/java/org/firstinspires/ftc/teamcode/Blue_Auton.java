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

@Autonomous(name="Blue", group="Autonomous")
public class Blue_Auton extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);
        ElapsedTime clock = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        // MeepMeep starting pose
        Pose2d startPose = new Pose2d(62, 12, Math.toRadians(180));

        // Your TankDrive constructor sets the pose internally
        TankDrive drive = new TankDrive(hardwareMap, startPose);

        // Build MeepMeep path
        Action fullAuto = drive.actionBuilder(startPose)
                .lineToX(12)
                .turn(Math.toRadians(32))
                .build();

        telemetry.addLine("BLUE AUTO READY");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        clock.reset();

        // Drivetrain initially off (RR will drive it)
        robot.driveWithControllers(0, 0, 0);

        // Shooter / turret setup
        robot.diffy.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.angle.setPosition(Settings.servoAngle);

        // Use BLUE pipeline (1)
        robot.limelight.pipelineSwitch(1);

        // Flywheel PID + velocity
        robot.flywheel.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(
                        Settings.FlywheelKP,
                        Settings.FlywheelKI,
                        Settings.FlywheelKD,
                        0
                )
        );
        robot.flywheel.setVelocity(Settings.flywheelVel / 60.0 * 24);

        // Lift + shot timing (3 shots)
        int shotsFired = 0;
        double lastShotTime = -5;
        double spinupTime = 1.5;     // wait after flywheel start before first shot
        double betweenShots = 1.2;   // gap between full shots

        double liftDownTime = 0.4;
        double liftUpTime = 0.4;
        double preShotPause = 0.3;   // extra delay between lift motion and intake fire

        // 0 = idle, 1 = lift down, 2 = lift up, 3 = pause before fire
        int liftPhase = 0;
        double liftPhaseStartTime = -10;

        // Run the RR trajectory once at the beginning
        Actions.runBlocking(fullAuto);

        // Main loop just handles aiming + shooting
        while (opModeIsActive() && !isStopRequested() && shotsFired < 3) {
            double t = clock.time();

            // --- Turret auto-aim for Blue ---
            double turretPower = 0;
            LLResult llResult = robot.limelight.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                double tx = llResult.getTx() + Settings.limelight_angular_offset;
                double error = tx;

                double deadband = 1.0;
                double k = 0.06;
                double minCommand = 0.06;
                double maxPower = 0.6;

                if (Math.abs(error) < deadband) {
                    turretPower = 0;
                } else {
                    double cmd = +k * error;

                    if (error > 0) {
                        cmd -= minCommand;
                    } else if (error < 0) {
                        cmd += minCommand;
                    }

                    if (cmd > maxPower) cmd = maxPower;
                    if (cmd < -maxPower) cmd = -maxPower;

                    turretPower = cmd;
                }

                telemetry.addData("Auto Tx", "%.2f", tx);
                telemetry.addData("Auto TurretPwr", "%.2f", turretPower);
            } else {
                turretPower = 0.18; // slow search
                telemetry.addData("Auto", "No Tag");
            }

            robot.turret.setPower(turretPower);
            robot.angle.setPosition(Settings.servoAngle);

            // --- Decide if we start a new shot cycle ---
            boolean canStartNewShot =
                    (t > spinupTime) &&
                            (t - lastShotTime > betweenShots) &&
                            (shotsFired < 3) &&
                            (liftPhase == 0);

            if (canStartNewShot) {
                liftPhase = 1;
                liftPhaseStartTime = t;
            }

            // --- Lift + shooting state machine ---
            double phaseTime = t - liftPhaseStartTime;

            switch (liftPhase) {
                case 0:
                    // idle
                    robot.lift.setPower(0);
                    break;

                case 1:
                    // lift down
                    if (phaseTime < liftDownTime) {
                        robot.lift.setPower(-0.25);
                    } else {
                        robot.lift.setPower(0);
                        liftPhase = 2;
                        liftPhaseStartTime = t;
                    }
                    break;

                case 2:
                    // lift up
                    if (phaseTime < liftUpTime) {
                        robot.lift.setPower(0.25);
                    } else {
                        robot.lift.setPower(0);
                        liftPhase = 3;
                        liftPhaseStartTime = t;
                    }
                    break;

                case 3:
                    // pause before firing
                    robot.lift.setPower(0);
                    if (phaseTime >= preShotPause) {
                        // FIRE ONE PIXEL
                        robot.intake.setPower(-1);
                        sleep(250);
                        robot.intake.setPower(0);

                        shotsFired++;
                        lastShotTime = clock.time();
                        liftPhase = 0;
                    }
                    break;
            }

            telemetry.addData("Time", "%.2f", t);
            telemetry.addData("Shots", shotsFired);
            telemetry.addData("LiftPhase", liftPhase);
            telemetry.update();

            idle();
        }

        // Stop everything
        robot.flywheel.setVelocity(0);
        robot.intake.setPower(0);
        robot.turret.setPower(0);
        robot.lift.setPower(0);
        robot.driveWithControllers(0, 0, 0);
    }
}
