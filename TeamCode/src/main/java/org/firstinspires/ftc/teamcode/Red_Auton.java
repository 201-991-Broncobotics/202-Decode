package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Red Far 3-Shot (Static)", group="Autonomous")
public class Red_Auton extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);
        ElapsedTime clock = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        telemetry.addLine("Red Static 3-Shot READY");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        clock.reset();

        // Drivetrain stays off the whole time
        robot.driveWithControllers(0, 0, 0);

        // Shooter / turret setup
        robot.diffy.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.angle.setPosition(Settings.servoAngle);

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

        int shotsFired = 0;
        double lastShotTime = -5;
        double spinupTime = 1.5;     // wait before first shot
        double betweenShots = 1.2;   // MORE gap between completed shots

        // Lift timing (per shot)
        double liftDownTime = 0.4;   // how long to move lift down
        double liftUpTime = 0.4;     // how long to move lift up
        double preShotPause = 0.3;   // EXTRA pause after lift before intake fires

        // 0 = idle, 1 = lift down, 2 = lift up, 3 = wait before fire
        int liftPhase = 0;
        double liftPhaseStartTime = -10;

        while (opModeIsActive() && !isStopRequested() && shotsFired < 3) {
            double t = clock.time();

            // --- Turret auto-aim like TeleOp ---
            double turretPower = 0;
            LLResult llResult = robot.limelight.getLatestResult();

            if (llResult != null && llResult.isValid()) {
                double tx = llResult.getTx();
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
                // Simple search if tag not found
                turretPower = 0.18;
                telemetry.addData("Auto", "No Tag");
            }

            robot.turret.setPower(turretPower);
            robot.angle.setPosition(Settings.servoAngle);

            // --- Decide if we should start a new shot cycle ---
            boolean canStartNewShot =
                    (t > spinupTime) &&
                            (t - lastShotTime > betweenShots) &&
                            (shotsFired < 3) &&
                            (liftPhase == 0);   // only start if lift is idle

            if (canStartNewShot) {
                liftPhase = 1;              // start lift-down phase
                liftPhaseStartTime = t;
            }

            // --- Lift + shooting state machine ---
            double phaseTime = t - liftPhaseStartTime;

            switch (liftPhase) {
                case 0:
                    // idle: keep lift stopped
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
                        liftPhase = 3;           // go into pre-shot pause
                        liftPhaseStartTime = t;
                    }
                    break;

                case 3:
                    // pause before firing (extra time gap between lift and shot)
                    robot.lift.setPower(0);
                    if (phaseTime >= preShotPause) {
                        // FIRE ONE PIXEL
                        robot.intake.setPower(-1);
                        sleep(250);
                        robot.intake.setPower(0);

                        shotsFired++;
                        lastShotTime = clock.time();
                        liftPhase = 0;          // back to idle
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
