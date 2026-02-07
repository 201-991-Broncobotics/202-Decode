package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpRed", group="Iterative Opmode")
public class TeleOpRed extends LinearOpMode {

    private static final double TICKS_PER_TURRET_REV = 2151.0; // unused now, ok to keep
    private double lastTx = 0;

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);
        Gamepad driver = gamepad1, operator = gamepad2;
        ElapsedTime mRuntime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        double FrameTime = 0;
        double flywheelVelocity = 0;
        ElapsedTime liftServoTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        boolean justStarted = true;
        double FlywheelTargetVel = 0;
        double flywheelPower = 0;
        boolean flywheelPowerToggle = false, xPressedLast = false;
        double IntakePower = 0;
        boolean IntakeOn = false, IntakeJustToggled = false;

        // TURRET SETUP
        robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.turret.setDirection(DcMotorSimple.Direction.FORWARD);

        // FLYWHEEL SETUP (flywheel MUST be DcMotorEx in RobotHardware)
        DcMotorEx fly = robot.flywheel;
        fly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        // RED ALLIANCE - Pipeline 0
        robot.limelight.pipelineSwitch(0);
        telemetry.addData("Pipeline", "Red (0)");
        telemetry.update();

        robot.diffy.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive() && !isStopRequested()) {

            // ---------- DRIVETRAIN ----------
            robot.driveWithControllers(
                    driver.left_stick_y + Settings.secretForward,
                    -0.7 * driver.left_stick_x + Settings.secretTurn,
                    1
            );

            // ---------- TURRET CONTROL (NO WIRE WRAP) ----------
            double turretPower = 0;
            boolean manualTurretActive = driver.dpad_left || driver.dpad_right;

            if (manualTurretActive) {
                // Manual dpad control
                turretPower = (driver.dpad_left ? 0.6 : 0.0)
                        + (driver.dpad_right ? -0.6 : 0.0);

            } else {
                // Auto track with Limelight
                LLResult llResult = robot.limelight.getLatestResult();
                if (llResult != null && llResult.isValid()) {
                    double tx = llResult.getTx();
                    lastTx = tx;

                    double kCenter = 0.03;   // gain near center
                    double kEdge = 0.015;    // smaller gain when far off
                    double deadband = 2.0;
                    double minCommand = 0.10;

                    double k = (Math.abs(tx) > 15.0) ? kEdge : kCenter;

                    if (Math.abs(tx) < deadband) {
                        turretPower = 0;
                    } else {
                        double error = tx;
                        double base = -k * error;

                        if (error > 0) {
                            base -= minCommand;
                        } else {
                            base += minCommand;
                        }

                        turretPower = Math.max(-0.3, Math.min(0.3, base));
                    }

                    telemetry.addData("Tx raw", "%.2f", tx);
                    telemetry.addData("Turret Power", "%.2f", turretPower);

                } else {
                    double searchPower = 0.15;

                    if (lastTx > 0) {
                        turretPower = -searchPower;
                    } else if (lastTx < 0) {
                        turretPower = searchPower;
                    } else {
                        turretPower = 0;
                    }

                    telemetry.addData("LL valid", "No Targets");
                    telemetry.addData("Search Power", "%.2f", turretPower);
                }
            }

            // Apply turret power directly (no wrap limiting)
            robot.turret.setPower(turretPower);

            // ---------- ANGLE LOCK ----------
            robot.angle.setPosition(Settings.servoAngle);

            // ---------- FLYWHEEL ----------
            if (driver.left_trigger > 0.25) {
                // Toggle between two speeds with X
                if (driver.x && !xPressedLast) {
                    flywheelPowerToggle = !flywheelPowerToggle;
                }
                xPressedLast = driver.x;

                double targetRpm = flywheelPowerToggle
                        ? Settings.flywheelVelBumper
                        : Settings.flywheelVel;

                double targetVelo = targetRpm / 60.0 * 28.0; // RPM -> ticks/sec
                fly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                fly.setVelocity(targetVelo);

                FlywheelTargetVel = targetRpm;
                flywheelVelocity = fly.getVelocity() * 60.0 / 28.0;
            } else if (driver.left_bumper) {
                double targetVelo = -Settings.flywheelVel / 240.0 * 28.0;
                fly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                fly.setVelocity(targetVelo);

                FlywheelTargetVel = -Settings.flywheelVel / 4.0;
                flywheelVelocity = fly.getVelocity() * 60.0 / 28.0;
            } else {
                fly.setVelocity(0);
                FlywheelTargetVel = 0;
                flywheelVelocity = 0;
            }

            // ---------- INTAKE ----------
            if (driver.right_trigger > 0.1) {
                IntakePower = 1;
            } else if (driver.right_bumper) {
                IntakePower = -1;
            }

            if (IntakeOn) {
                robot.intake.setPower(IntakePower);
            } else {
                IntakePower = 0 + Settings.secretIntake;
            }

            if (driver.right_trigger > 0.1 || driver.right_bumper) {
                if (!IntakeJustToggled) {
                    IntakeOn = !IntakeOn;
                    IntakeJustToggled = true;
                }
            } else {
                IntakeJustToggled = false;
            }

            // ---------- LIFT ----------
            if (driver.b) {
                liftServoTimer.reset();
                robot.intake.setPower(0);
                justStarted = false;
            }
            if (liftServoTimer.time() < 0.8 && !justStarted) {
                robot.lift.setPower(-0.25);
                robot.intake.setPower(0);
            } else if (liftServoTimer.time() < 1.8 && !justStarted) {
                robot.lift.setPower(0.25);
                robot.intake.setPower(0);
            } else {
                robot.lift.setPower(0);
            }

            // ---------- TELEMETRY ----------
            FrameTime = mRuntime.time();
            mRuntime.reset();
            telemetry.addData("FPS:", 1.0 / FrameTime);
            telemetry.addData("Turret Position", robot.turret.getCurrentPosition());
            telemetry.addData("Turret Power", turretPower);
            telemetry.addData("Flywheel Target Speed (rpm)", FlywheelTargetVel);
            telemetry.addData("Flywheel Speed (rpm)", flywheelVelocity);
            telemetry.addData("Flywheel encoder", robot.flywheel.getCurrentPosition());
            telemetry.addData("Flywheel power", flywheelPower);
            telemetry.addData("Flywheel Angle: ", robot.angle.getPosition());
            telemetry.addLine();
            telemetry.update();
        }
    }
}
