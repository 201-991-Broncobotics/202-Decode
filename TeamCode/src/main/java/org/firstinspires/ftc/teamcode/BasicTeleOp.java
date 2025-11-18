package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TeleOp", group="Iterative Opmode")
public class BasicTeleOp extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);

        Gamepad driver = gamepad1, operator = gamepad2; // initialize controllers

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



        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            // DRIVETRAIN
            // Call the Mecanum drive method from RobotHardware to drive the robot
            // If you want to use absolute driving, you might want to make use of Roadrunner's localizer (check BasicAuton)
            robot.driveWithControllers(driver.left_stick_y + Settings.secretForward, -0.7 * driver.left_stick_x + Settings.secretTurn, 1); // 1 - 0.6 * driver.left_trigger



            // OTHER MOTORS / MECHANISMS

            // Very basic way of moving a motor though I would use either a PID or the RUN_TO_POSITION mode
            // robot.Arm.setPower(operator.right_stick_y);


            // This won't do anything until you tune the PID in RobotHardware
            // robot.Arm.setPower(robot.ArmPID.getPower(0));

            // This is all you would need to do to use RUN_TO_POSITION:
            // robot.Arm.setTargetPosition(0);

            // TURRET
            robot.turret.setPower((driver.dpad_left ? 0.6 : 0.0) + (driver.dpad_right ? -0.6 : 0.0));

            // FLYWHEEL
            if (driver.left_trigger > 0.1) {
                FlywheelTargetVel = Settings.flywheelVel;
                robot.angle.setPosition(Settings.servoAngle);

            } else if (driver.left_bumper) {
                FlywheelTargetVel = Settings.flywheelVel2;
                robot.angle.setPosition(Settings.servoAngle2);
            }

            if (FlywheelOn) {

                robot.flywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Settings.FlywheelKP, Settings.FlywheelKI, Settings.FlywheelKD, 0)); // F is just a constant, like for gravity which isn't needed here
                robot.flywheel.setVelocity(FlywheelTargetVel/60.0 * 28); // in encoder tick per second
                // robot.flywheel.setPower(0.7); // in case the velocity thing doesn't work

                //Set flap angle
                robot.angle.setPosition(Settings.servoAngle);

            } else robot.flywheel.setPower(0);

            if (driver.left_trigger > 0.1 || driver.left_bumper) {
                if (!FlywheelJustToggled) {
                    FlywheelOn = !FlywheelOn;
                    FlywheelJustToggled = true;
                }
            } else FlywheelJustToggled = false;


            // INTAKE
            if (driver.right_trigger > 0.1) {
                IntakePower = 1;
            } else if (driver.right_bumper) {
                IntakePower = -1;
            }

            if (IntakeOn) robot.intake.setPower(IntakePower);
            else IntakePower = 0 + Settings.secretIntake; // secret stuff just gives me the ability to drive the robot from my computer

            if (driver.right_trigger > 0.1 || driver.right_bumper) {
                if (!IntakeJustToggled) {
                    IntakeOn = !IntakeOn;
                    IntakeJustToggled = true;
                }
            } else IntakeJustToggled = false;

            // LIFT SERVO
            if (driver.b) {
                liftServoTimer.reset();
                robot.intake.setPower(0);
                justStarted = false; // waits until you hit the button first before it resets the timer, just cause the timer gets restarted with the robot being enabled
            }
            if (liftServoTimer.time() < 0.8 && !justStarted) {
                robot.lift.setPower(-0.25);
                robot.intake.setPower(0);
                // robot.lift.setPosition(Settings.ServoTopPosition);
            } else if (liftServoTimer.time() < 1.8 && !justStarted) {
                robot.lift.setPower(0.25);
                robot.intake.setPower(0);
                // robot.lift.setPosition(Settings.ServoBottomPosition);
            } else robot.lift.setPower(0);


            // This is an example of how to use a servo as a claw
            //if (operator.y) robot.Claw.setPosition(0); // setPosition's units are weird though it usually is 1 = 180 degrees, 0.5 = 90 degrees, etc.
            //else if (operator.x) robot.Claw.setPosition(0.5);



            // TELEMETRY
            FrameTime = mRuntime.time(); // time it takes for the entire code to run once
            mRuntime.reset();
            telemetry.addData("FPS:", 1.0 / FrameTime); // This will find how many times this loop runs per second, also called Hertz
            telemetry.addData("Turret Position", robot.turret.getCurrentPosition());
            // telemetry.addData("Lift Position", robot.lift.getPosition());
            telemetry.addData("Flywheel Target Speed (rpm)", FlywheelTargetVel);
            telemetry.addData("Flywheel Speed (rpm)", flywheelVelocity);
            telemetry.addData("Flywheel encoder", robot.flywheel.getCurrentPosition());
            telemetry.addData("Flywheel power", flywheelPower);
            telemetry.addData("Flywheel Angle: ", robot.angle.getPosition());


            // Telemetry
            telemetry.addLine(""); // blank line
            telemetry.update();
        }
    }
}