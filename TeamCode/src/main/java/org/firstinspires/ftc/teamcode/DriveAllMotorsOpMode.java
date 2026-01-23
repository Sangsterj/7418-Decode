package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "DriveAllMotorsOpMode")
public class DriveAllMotorsOpMode extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx outtake;
    private DcMotor intake;
    private CRServo spinner;
    private DcMotor transfer;

    @Override
    public void runOpMode() {

        // ----------------------
        // SAFE HARDWARE MAPPING
        // ----------------------
        try { frontLeft = hardwareMap.get(DcMotor.class, "frontLeft"); } catch (Exception ignored) {}
        try { frontRight = hardwareMap.get(DcMotor.class, "frontRight"); } catch (Exception ignored) {}
        try { backLeft = hardwareMap.get(DcMotor.class, "backLeft"); } catch (Exception ignored) {}
        try { backRight = hardwareMap.get(DcMotor.class, "backRight"); } catch (Exception ignored) {}

        try { spinner = hardwareMap.get(CRServo.class, "spinner"); } catch (Exception ignored) {}
        try { outtake = (DcMotorEx) hardwareMap.get(DcMotor.class, "outtake"); } catch (Exception ignored) {}
        try { intake = hardwareMap.get(DcMotor.class, "intake"); } catch (Exception ignored) {}
        try { transfer = hardwareMap.get(DcMotor.class, "transfer"); } catch (Exception ignored) {}

        // ----------------------
        // MOTOR DIRECTIONS
        // ----------------------
        if (frontLeft != null) frontLeft.setDirection(DcMotor.Direction.REVERSE);
        if (backLeft != null) backLeft.setDirection(DcMotor.Direction.REVERSE);

        if (frontLeft != null) frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (frontRight != null) frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (backLeft != null) backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (backRight != null) backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ----------------------
            // MECANUM DRIVE
            // ----------------------
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double flPower = y + x + rx;
            double blPower = y - x + rx;
            double frPower = y - x - rx;
            double brPower = y + x - rx;

            double max = Math.max(
                    Math.max(Math.abs(flPower), Math.abs(blPower)),
                    Math.max(Math.abs(frPower), Math.abs(brPower))
            );

            if (max > 1) {
                flPower /= max;
                blPower /= max;
                frPower /= max;
                brPower /= max;
            }

            if (frontLeft != null) frontLeft.setPower(flPower);
            if (backLeft != null) backLeft.setPower(blPower);
            if (frontRight != null) frontRight.setPower(frPower);
            if (backRight != null) backRight.setPower(brPower);

            // ----------------------
            // OUTTAKE MOTOR
            // ----------------------
            double outtakeInput = gamepad1.right_trigger - gamepad1.left_trigger;
            if (Math.abs(outtakeInput) < 0.05) outtakeInput = 0;

            if (outtake != null) outtake.setPower(outtakeInput);

            // ----------------------
            //  SPINNER
            // ----------------------
            double spinnerPower = 0;

            if (gamepad1.left_bumper) {
                spinnerPower = 1.0;     // forward
            } else if (gamepad1.right_bumper) {
                spinnerPower = -1.0;    // backward
            }

            if (spinner != null) spinner.setPower(spinnerPower);

            // ----------------------
            // INTAKE MOTOR
            // ----------------------
            double intakePower = 0;
            if (gamepad1.a) intakePower = -1;
            else if (gamepad1.y) intakePower = 1;

            if (intake != null) intake.setPower(intakePower);

            // ----------------------
            // TRANSFER MOTOR
            // ----------------------
            double transferPower = 0;
            if (gamepad1.x) transferPower = -1;
            else if (gamepad1.b) transferPower = 1;

            if (transfer != null) transfer.setPower(transferPower);

            // ----------------------
            // TELEMETRY
            // ----------------------
            telemetry.addData("Spinner Power", spinnerPower);
            telemetry.update();
        }
    }
}
