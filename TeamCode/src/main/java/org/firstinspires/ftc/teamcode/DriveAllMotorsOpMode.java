package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DriveAllMotorsOpMode", group = "Linear Opmode")
public class DriveAllMotorsOpMode extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intake1, intake2;

    // Safe hardware initialization helper
    private DcMotor safeGetMotor(String name) {
        try {
            DcMotor motor = hardwareMap.get(DcMotor.class, name);
            telemetry.addData("Motor Loaded", name);
            return motor;
        } catch (Exception e) {
            telemetry.addData("⚠ Missing Motor", name);
            return null;
        }
    }

    @Override
    public void runOpMode() {
        // Initialize motors safely
        frontLeft = safeGetMotor("frontLeft");
        frontRight = safeGetMotor("frontRight");
        backLeft = safeGetMotor("backLeft");
        backRight = safeGetMotor("backRight");
        intake1 = safeGetMotor("intake1");
        intake2 = safeGetMotor("intake2");

        // --- Motor Directions ---
        if (frontLeft != null) frontLeft.setDirection(DcMotor.Direction.REVERSE);
        if (backLeft != null) backLeft.setDirection(DcMotor.Direction.REVERSE);
        if (frontRight != null) frontRight.setDirection(DcMotor.Direction.FORWARD);
        if (backRight != null) backRight.setDirection(DcMotor.Direction.FORWARD);

        if (intake1 != null) intake1.setDirection(DcMotor.Direction.FORWARD);
        if (intake2 != null) intake2.setDirection(DcMotor.Direction.REVERSE);

        // --- Zero Power Behavior ---
        DcMotor.ZeroPowerBehavior BRAKE = DcMotor.ZeroPowerBehavior.BRAKE;
        if (frontLeft != null) frontLeft.setZeroPowerBehavior(BRAKE);
        if (frontRight != null) frontRight.setZeroPowerBehavior(BRAKE);
        if (backLeft != null) backLeft.setZeroPowerBehavior(BRAKE);
        if (backRight != null) backRight.setZeroPowerBehavior(BRAKE);
        if (intake1 != null) intake1.setZeroPowerBehavior(BRAKE);
        if (intake2 != null) intake2.setZeroPowerBehavior(BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Mecanum Drive ---
            double y = -gamepad1.left_stick_y;  // forward/backward
            double x = -gamepad1.left_stick_x;  // strafe
            double rx = gamepad1.right_stick_x; // rotate

            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

            // Normalize powers
            double max = Math.max(
                    Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                    Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))
            );
            if (max > 1.0) {
                frontLeftPower /= max;
                backLeftPower /= max;
                frontRightPower /= max;
                backRightPower /= max;
            }

            if (frontLeft != null) frontLeft.setPower(frontLeftPower);
            if (backLeft != null) backLeft.setPower(backLeftPower);
            if (frontRight != null) frontRight.setPower(frontRightPower);
            if (backRight != null) backRight.setPower(backRightPower);

            // --- Intake Controls (Trigger-Analog) ---
            double forward = gamepad1.right_trigger;  // 0.0 → 1.0
            double reverse = gamepad1.left_trigger;   // 0.0 → 1.0

            double intakePower = forward - reverse;   // net power (-1.0 to 1.0)

            if (intake1 != null) intake1.setPower(intakePower);
            if (intake2 != null) intake2.setPower(intakePower);

            // --- Telemetry ---
            telemetry.addData("FL", frontLeftPower);
            telemetry.addData("FR", frontRightPower);
            telemetry.addData("BL", backLeftPower);
            telemetry.addData("BR", backRightPower);
            telemetry.addData("Intake Power", intakePower);
            telemetry.addData("RT", forward);
            telemetry.addData("LT", reverse);
            telemetry.update();
        }

        // Stop motors on exit
        if (intake1 != null) intake1.setPower(0);
        if (intake2 != null) intake2.setPower(0);
    }
}
