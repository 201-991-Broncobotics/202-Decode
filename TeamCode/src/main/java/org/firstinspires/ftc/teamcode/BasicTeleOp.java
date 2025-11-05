package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp", group="Iterative Opmode")
public class BasicTeleOp extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);

        Gamepad driver = gamepad1, operator = gamepad2; // initialize controllers

        ElapsedTime mRuntime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        // You can create any variables you need here
        double FrameRate = 0;



        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            // DRIVETRAIN
            // Call the Mecanum drive method from RobotHardware to drive the robot
            // If you want to use absolute driving, you might want to make use of Roadrunner's localizer (check BasicAuton)
            robot.driveWithControllers(driver.left_stick_y, -1 * driver.left_stick_x, 1 - 0.6 * driver.left_trigger);



            // OTHER MOTORS / MECHANISMS

            // Very basic way of moving a motor though I would use either a PID or the RUN_TO_POSITION mode
            // robot.Arm.setPower(operator.right_stick_y);


            // This won't do anything unless you tune the PID in RobotHardware
            // robot.Arm.setPower(robot.ArmPID.getPower(0));

            // This is all you would need to do to use RUN_TO_POSITION:
            // robot.Arm.setTargetPosition(0);


            // This is an example of how to use a servo as a claw
            //if (operator.y) robot.Claw.setPosition(0); // setPosition's units are weird though it usually is 1 = 180 degrees, 0.5 = 90 degrees, etc.
            //else if (operator.x) robot.Claw.setPosition(0.5);



            // TELEMETRY
            FrameRate = 1 / (mRuntime.time() / 1000); // This will find how many times this loop runs per second
            telemetry.addData("FPS:", FrameRate); // also called Hz
            telemetry.addData("Heading", Math.toDegrees(robot.getHeading()));
            mRuntime.reset();

            // Telemetry
            telemetry.addLine("");
            telemetry.update();
        }
    }
}