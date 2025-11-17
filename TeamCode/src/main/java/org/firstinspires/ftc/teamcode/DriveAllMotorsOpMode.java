package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DriveAllMotorsOpMode")
public class DriveAllMotorsOpMode extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor outtake1;
    private DcMotor outtake2;
    private DcMotor intake;

    @Override
    public void runOpMode() {
        // --- Initialize drive motors (safe mapping) ---
        try { frontLeft = hardwareMap.get(DcMotor.class, "frontLeft"); } catch (Exception e) {}
        try { frontRight = hardwareMap.get(DcMotor.class, "frontRight"); } catch (Exception e) {}
        try { backLeft = hardwareMap.get(DcMotor.class, "backLeft"); } catch (Exception e) {}
        try { backRight = hardwareMap.get(DcMotor.class, "backRight"); } catch (Exception e) {}

        // --- Outtake motors ---
        try { outtake1 = hardwareMap.get(DcMotor.class, "outtake1"); } catch (Exception e) {}
        try { outtake2 = hardwareMap.get(DcMotor.class, "outtake2"); } catch (Exception e) {}

        // --- Intake motor ---
        try { intake = hardwareMap.get(DcMotor.class, "intake"); } catch (Exception e) {}

        // --- Motor directions ---
        if (frontLeft != null) frontLeft.setDirection(DcMotor.Direction.REVERSE);
        if (backLeft != null) backLeft.setDirection(DcMotor.Direction.REVERSE);
        if (frontRight != null) frontRight.setDirection(DcMotor.Direction.FORWARD);
        if (backRight != null) backRight.setDirection(DcMotor.Direction.FORWARD);

        if (frontLeft != null) frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (frontRight != null) frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (backLeft != null) backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (backRight != null) backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Mechanum Drive ---
            double y = -gamepad1.left_stick_y;  // Forward/backward
            double x = -gamepad1.left_stick_x;  // Strafe
            double rx = gamepad1.right_stick_x; // Rotate

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

            // --- Outtake Motors (Triggers + Deadband) ---
            double forward = gamepad1.right_trigger;
            double reverse = gamepad1.left_trigger;
            double outtakePower = forward - reverse;

            // Deadband
            if (Math.abs(outtakePower) < 0.05) outtakePower = 0;

            if (outtake1 != null) outtake1.setPower(outtakePower);
            if (outtake2 != null) outtake2.setPower(outtakePower);

            // --- Intake Motor (A and Y Buttons) ---
            double intakePower = 0;
            if (gamepad1.a) {
                intakePower = -1.0;   // Forward
            } else if (gamepad1.y) {
                intakePower = 1.0;  // Reverse
            }

            if (intake != null) intake.setPower(intakePower);

            // --- Telemetry ---
            telemetry.addData("Drive", "FL %.2f | FR %.2f | BL %.2f | BR %.2f",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("Outtake Power", outtakePower);
            telemetry.addData("Intake Power", intakePower);
            telemetry.update();
        }

        // Stop motors
        if (outtake1 != null) outtake1.setPower(0);
        if (outtake2 != null) outtake2.setPower(0);
        if (intake != null) intake.setPower(0);
    }
}
