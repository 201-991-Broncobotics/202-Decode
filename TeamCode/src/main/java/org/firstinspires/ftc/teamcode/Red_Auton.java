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
        double spinupTime = 1.5;   // wait before first shot
        double betweenShots = 0.7; // gap between shots

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
                    double cmd = +k * error;  // use same sign that works in TeleOp

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

            // --- Shooting logic ---
            boolean readyToShoot = t > spinupTime && (t - lastShotTime) > betweenShots;

            if (readyToShoot && shotsFired < 3) {
                robot.intake.setPower(-1);
                sleep(250);         // pulse length for one pixel
                robot.intake.setPower(0);

                shotsFired++;
                lastShotTime = clock.time();
            }

            telemetry.addData("Time", "%.2f", t);
            telemetry.addData("Shots", shotsFired);
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
