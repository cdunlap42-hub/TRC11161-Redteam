package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@Config
public class MecanumDrive {
    //public static double DASHBOARD_TEST_VAR = 0.0;
    // --- Dashboard Params ---
    public static class Params {
        // Road Runner Constants (Tune these via Dashboard!)
        public double inchesPerTick = 0.0029;
        public double lateralMultiplier = 1.0;
        public double trackWidthTicks = 15.0;
    }

    public static Params PARAMS = new Params();

    // --- Hardware ---
    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;
    public final IMU imu;
    public final Encoder leftEncoder, rightEncoder, perpEncoder;

    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        FlightRecorder.write("MECANUM_DRIVE_CONFIG", this);

        // Motor Initialization
        leftFront = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        leftBack  = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        rightBack = hardwareMap.get(DcMotorEx.class, "back_right_drive");
        rightFront = hardwareMap.get(DcMotorEx.class, "front_right_drive");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);

        // Initialize Dead Wheels
        leftEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "par0")));
        rightEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "intake")));
        perpEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "conveyance")));

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        double leftFrontPower = powers.linearVel.x - powers.linearVel.y - powers.angVel;
        double leftBackPower = powers.linearVel.x + powers.linearVel.y - powers.angVel;
        double rightBackPower = powers.linearVel.x - powers.linearVel.y + powers.angVel;
        double rightFrontPower = powers.linearVel.x + powers.linearVel.y + powers.angVel;

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);
    }
}