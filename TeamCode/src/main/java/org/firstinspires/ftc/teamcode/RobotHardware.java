package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import javax.tools.ForwardingFileObject;


public class RobotHardware {

    public final HardwareMap map;
    public final Telemetry telemetry;

    public final Limelight3A limelight;


    public final PIDFController turretPID = new PIDFController(
            Settings.turret_P,
            Settings.turret_I,
            Settings.turret_D,
            0.0
    );

    public final DcMotor RT, RB, LT, LB, diffy, turret, intake;
    public final DcMotorEx flywheel;

    /*
    DcMotor is the basic motor class for using a motor though there is also DcMotorEx which allows
    for running a motor at a specific speed
     */

    public final CRServo lift; // If you need to use a servo similar to a motor, use CRServo
    public final Servo angle;

    // This is everything you would ever need to know in order to use a HuskyLens: https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336


    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        RT = hardwareMap.get(DcMotor.class, "RT");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LT = hardwareMap.get(DcMotor.class, "LT");
        LB = hardwareMap.get(DcMotor.class, "LB");
        diffy = hardwareMap.get(DcMotor.class, "diffy");
        turret = hardwareMap.get(DcMotor.class, "turret");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        lift = hardwareMap.get(CRServo.class, "lift");
        angle = hardwareMap.get(Servo.class, "angle");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");


        // This offers a description of what some of the run modes and zeroPowerBehavior do if you click on some of the methods
        // https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotor.html
        //turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        LT.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RT.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.REVERSE);
        diffy.setDirection(DcMotor.Direction.FORWARD);
        turret.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setDirection(DcMotorEx.Direction.FORWARD);
        angle.setDirection(Servo.Direction.FORWARD);

        //Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        LT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        diffy.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // This resets the encoder
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // This activates the motor to use the PID



        LT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // applying brake is helpful for drive motors and motors using pids
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        diffy.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); // flywheel set to brake so it can recharge the battery as it slows down

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        angle.scaleRange(0, 1);

        // Limelight init
        limelight.pipelineSwitch(0);
        limelight.start();

        // You don't need to use this specific PID as the DcMotor and DcMotorEx have their own form of
        // a PID which you can use by setting the runMode to RUN_TO_POSITION


        telemetry.addLine("Robot Hardware Initialized");
        telemetry.update();
        this.map = hardwareMap;

    } // initializes everything


    public void driveWithControllers(double forward, double turn, double throttle) {
        double max_power = Math.max(1, Math.max(forward, turn));
        LT.setPower(throttle * (-forward - turn) / max_power);
        LB.setPower(throttle * -(-forward - turn) / max_power);
        RT.setPower(throttle * (forward - turn) / max_power);
        RB.setPower(throttle * -(forward - turn) / max_power);
        diffy.setPower(throttle * forward / max_power);
    }


    // This is a place where you can add your own methods

}