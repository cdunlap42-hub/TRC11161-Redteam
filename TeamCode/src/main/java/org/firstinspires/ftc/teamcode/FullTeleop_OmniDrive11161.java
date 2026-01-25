package org.firstinspires.ftc.teamcode;

import android.graphics.Color; // Needed for HSV conversion
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Full TeleOp - Color Sensor", group = "Linear OpMode")
public class FullTeleop_OmniDrive11161 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // --- Hardware ---
    private com.qualcomm.robotcore.hardware.DcMotorEx frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private com.qualcomm.robotcore.hardware.DcMotorEx spinnerMotor, intake, conveyance;
    private Servo rampServo, kicker;
    private CRServo revolver;

    // Color Sensor
    private NormalizedColorSensor colorSensor;

    // --- Variables ---
    private boolean intakeIsOn = false, xWasPressed1 = false;
    private boolean spinnerIsOn = false, aWasPressed2 = false;
    private boolean isKicking = false, rightTriggerWasPressed = false;
    private ElapsedTime kickerTimer = new ElapsedTime();

    // --- UPDATED KICKER CONSTANTS ---
    static final double KICKER_REST_POS = 0.7;
    static final double KICKER_UP_POS   = 0.2;
    static final double KICK_DURATION   = 0.3;

    static final double RAMP_POS_60 = 0.40, RAMP_POS_40 = 0.30, RAMP_POS_20 = 0.20, RAMP_POS_REST = 0.0;
    static final double SPINNER_VELOCITY = 1400;

    @Override
    public void runOpMode() {
        // --- Initialization ---
        frontLeftDrive  = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "front_left_drive");
        backLeftDrive   = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "front_right_drive");
        backRightDrive  = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "back_right_drive");
        spinnerMotor    = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "spinner_motor");
        intake          = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "intake");
        conveyance      = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotorEx.class, "conveyance");
        rampServo       = hardwareMap.get(Servo.class, "ramp_servo");
        revolver        = hardwareMap.get(CRServo.class, "revolver");
        kicker          = hardwareMap.get(Servo.class, "kicker");

        colorSensor     = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        // Directions & Initial Positions
        // Flipped directions to fix opposite joystick behavior
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotor.Direction.REVERSE);
        conveyance.setDirection(DcMotor.Direction.REVERSE);

        rampServo.setDirection(Servo.Direction.REVERSE);

        // Kicker configuration
        kicker.setDirection(Servo.Direction.FORWARD);

        // Set initial positions
        kicker.setPosition(KICKER_REST_POS);
        rampServo.setPosition(RAMP_POS_REST);

        waitForStart();
        while (opModeIsActive()) {
            // --- 1. Driving ---
            double axial = -gamepad1.left_stick_y, lateral = gamepad1.left_stick_x, yaw = gamepad1.right_stick_x;
            frontLeftDrive.setPower(axial + lateral + yaw);
            frontRightDrive.setPower(axial - lateral - yaw);
            backLeftDrive.setPower(axial - lateral + yaw);
            backRightDrive.setPower(axial + lateral - yaw);

            // --- 2. Intake/Conveyance Toggle ---
            if (gamepad1.x && !xWasPressed1) intakeIsOn = !intakeIsOn;
            xWasPressed1 = gamepad1.x;
            if (gamepad1.y) { intake.setPower(-1.0); conveyance.setPower(-1.0); intakeIsOn = false; }
            else if (intakeIsOn) { intake.setPower(1.0); conveyance.setPower(1.0); }
            else { intake.setPower(0.0); conveyance.setPower(0.0); }

            // --- 3. Spinner Toggle ---
            if (gamepad2.a && !aWasPressed2) spinnerIsOn = !spinnerIsOn;
            aWasPressed2 = gamepad2.a;
            spinnerMotor.setVelocity(spinnerIsOn ? SPINNER_VELOCITY : 0);

            // --- 4. Ramp Positions ---
            if (gamepad2.b) rampServo.setPosition(RAMP_POS_60);
            else if (gamepad2.y) rampServo.setPosition(RAMP_POS_40);
            else if (gamepad2.x) rampServo.setPosition(RAMP_POS_20);

            // --- 5. Revolver ---
            if (gamepad2.right_bumper) revolver.setPower(1.0);
            else if (gamepad2.left_bumper) revolver.setPower(-1.0);
            else revolver.setPower(0.0);

            // --- 6. Kicker Logic ---
            boolean rtPressed = gamepad2.right_trigger > 0.5;
            if (rtPressed && !rightTriggerWasPressed && !isKicking) {
                isKicking = true;
                kickerTimer.reset();
                kicker.setPosition(KICKER_UP_POS);
            }
            if (isKicking && kickerTimer.seconds() > KICK_DURATION) {
                kicker.setPosition(KICKER_REST_POS);
                isKicking = false;
            }
            rightTriggerWasPressed = rtPressed;

            // --- Color Sensor Logic ---
            float[] hsvValues = {0F, 0F, 0F};
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            float hue = hsvValues[0];

            if (hue > 120 && hue < 160) {
                telemetry.addData("DETECTED", "GREEN");
            } else if (hue > 210 && hue < 270) {
                telemetry.addData("DETECTED", "Purple");
            } else {
                telemetry.addData("DETECTED", "None");
            }

            telemetry.addData("Hue Value", hue);
            telemetry.update();
        }
    }
}