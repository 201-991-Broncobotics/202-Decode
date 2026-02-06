package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="FlywheelEncoderCheck", group="Testing")
public class FlywheelEncoderCheck extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);
        DcMotorEx fly = robot.flywheel;

        // Reset encoder so one turn is easy to read
        fly.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fly.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Mark flywheel, then turn it EXACTLY 1 full turn by hand after start.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Current Position (Ticks)", fly.getCurrentPosition());
            telemetry.addData("Current Velocity (Ticks/Sec)", fly.getVelocity());
            telemetry.update();
            idle();
        }
    }
}
