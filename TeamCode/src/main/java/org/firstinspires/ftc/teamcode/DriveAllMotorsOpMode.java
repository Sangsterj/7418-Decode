package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DriveAllMotorsOpMode", group = "Linear Opmode")
public class DriveAllMotorsOpMode extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intake1, intake2;

    // Helper to safely initialize motors
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

        // Set directions (only for existing motors)
        if (frontLeft != null) frontLeft.setDirection(DcMotor.Direction.REVERSE);
        if (backLeft != null) backLeft.setDirection(DcMotor.Direction.REVERSE);
        if (frontRight != null) frontRight.setDirection(DcMotor.Direction.FORWARD);
        if (backRight != null) backRight.setDirection(DcMotor.Direction.FORWARD);

        if (intake1 != null) intake1.setDirection(DcMotor.Direction.FORWARD);
        if (intake2 != null) intake2.setDirection(DcMotor.Direction.REVERSE);

        // Zero power behavior
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

        boolean spinningForward = false;
        boolean spinningBackward = false;
        boolean rtPressedLast = false;
        boolean ltPressedLast = false;

        double targetIntakePower = 0.0;
        double currentIntakePower = 0.0;
        double rampSpeed = 0.05;

        while (opModeIsActive()) {
            // --- Mecanum Drive ---
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

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

            // --- Toggle Intake Controls ---
            boolean rtPressedNow = gamepad1.right_trigger > 0.5;
            boolean ltPressedNow = gamepad1.left_trigger > 0.5;

            if (rtPressedNow && !rtPressedLast) {
                if (spinningForward) {
                    spinningForward = false;
                    targetIntakePower = 0.0;
                } else {
                    spinningForward = true;
                    spinningBackward = false;
                    targetIntakePower = 1.0;
                }
            }

            if (ltPressedNow && !ltPressedLast) {
                if (spinningBackward) {
                    spinningBackward = false;
                    targetIntakePower = 0.0;
                } else {
                    spinningBackward = true;
                    spinningForward = false;
                    targetIntakePower = -1.0;
                }
            }

            rtPressedLast = rtPressedNow;
            ltPressedLast = ltPressedNow;

            // Smooth ramping
            if (Math.abs(targetIntakePower - currentIntakePower) > 0.01) {
                if (currentIntakePower < targetIntakePower)
                    currentIntakePower += rampSpeed;
                else
                    currentIntakePower -= rampSpeed;

                currentIntakePower = Math.max(-1.0, Math.min(1.0, currentIntakePower));
            }

            if (intake1 != null) intake1.setPower(currentIntakePower);
            if (intake2 != null) intake2.setPower(currentIntakePower);

            telemetry.addData("FL", frontLeftPower);
            telemetry.addData("FR", frontRightPower);
            telemetry.addData("BL", backLeftPower);
            telemetry.addData("BR", backRightPower);
            telemetry.addData("Target Intake", targetIntakePower);
            telemetry.addData("Current Intake", currentIntakePower);
            telemetry.addData("Forward?", spinningForward);
            telemetry.addData("Backward?", spinningBackward);
            telemetry.update();
        }

        if (intake1 != null) intake1.setPower(0);
        if (intake2 != null) intake2.setPower(0);
    }
}
