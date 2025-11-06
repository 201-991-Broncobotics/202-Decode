package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.function.DoubleSupplier;


public class RobotHardware {

    public final HardwareMap map;
    public final Telemetry telemetry;


    public final DcMotor RT, RB, LT, LB, diffy;

    public final IMU imu;

    /*
    DcMotor is the basic motor class for using a motor though there is also DcMotorEx which allows
    for running a motor at a specific speed
     */


    //public final DcMotorEx Arm;

    //public final Servo Claw; // If you need to use a servo similar to a motor, use CRServo

    // This is everything you would ever need to know in order to use a HuskyLens: https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336


    public PIDController FieldCentricPoint;
    public final DoubleSupplier heading;


    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot;
        orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        heading = () -> imu.getRobotYawPitchRollAngles().getYaw();



        RT = hardwareMap.get(DcMotor.class, "RT");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LT = hardwareMap.get(DcMotor.class, "LT");
        LB = hardwareMap.get(DcMotor.class, "LB");
        diffy = hardwareMap.get(DcMotor.class, "diffy");
        //Arm = hardwareMap.get(DcMotorEx.class, "Arm");
        //Claw = hardwareMap.get(Servo.class, "Claw");



        FieldCentricPoint = new PIDController(0, 0, 0, heading);


        // RF.setDirection(DcMotor.Direction.REVERSE); // This is one way of reversing the direction of motor if needed

        //Claw.setDirection(Servo.Direction.FORWARD);


        // This offers a description of what some of the run modes and zeroPowerBehavior do if you click on some of the methods
        // https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotor.html
        LT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // This resets the encoder at the start of the code
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        diffy.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LT.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.FORWARD);
        RT.setDirection(DcMotorSimple.Direction.FORWARD);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);
        diffy.setDirection(DcMotorSimple.Direction.FORWARD);

        //Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        LT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        diffy.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        LT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        diffy.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        // You don't need to use this specific PID as the DcMotor and DcMotorEx have their own form of
        // a PID which you can use by setting the runMode to RUN_TO_POSITION

        // This sets up the Arm PID (instructions for how to tune this I put in the PIDController class)
        //ArmPID = new PIDController(0.0, 0.0, 0.0, Arm::getCurrentPosition);



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


    public void driveFieldCentric(double forward, double strafe, double turn, double throttle) {
        double angle = 0;
        double magnitude = Math.hypot(forward, strafe) * throttle;


    }

    public double getHeading() {
        return heading.getAsDouble();
    }


    // This is where you can add your own methods

}