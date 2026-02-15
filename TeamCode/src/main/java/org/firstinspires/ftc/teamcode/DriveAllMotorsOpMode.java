package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "DriveAllMotorsOpMode")
public class DriveAllMotorsOpMode extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx outtake1;
    private DcMotorEx outtake2;
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
        try { outtake1 = (DcMotorEx) hardwareMap.get(DcMotor.class, "outtake1"); } catch (Exception ignored) {}
        try { outtake2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "outtake2"); } catch (Exception ignored) {}
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

        // Optional: reverse spinner if needed
        if (spinner != null) spinner.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // ----------------------
        // START SPINNER CONTINUOUSLY
        // ----------------------
        if (spinner != null) spinner.setPower(1.0); // continuous forward spin

        while (opModeIsActive()) {

            // ----------------------
            // MECANUM DRIVE
            // ----------------------
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
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
            // OUTTAKE MOTORS
            // ----------------------
            double outtakeInput1 = 0.0;
            double outtakeInput2 = 0.0;
            double transferpower = 0;

            if (gamepad1.left_trigger > 0) {
                outtakeInput1 = 1;
                outtakeInput2 = 1;
                transferpower = 1;
            } else if (gamepad1.right_trigger > 0) {
                outtakeInput1 = 0.75;
                outtakeInput2 = 0.75;
                transferpower = 1;

            } else if (gamepad1.dpad_up){ // testing
                outtakeInput1 = 1;
                outtakeInput2 = 0.5;
                transferpower = 1;
            }else if (gamepad1.dpad_down){ // testing
                outtakeInput1 = 0.5;
                outtakeInput2 = 1;
                transferpower = 1;
            }else if (gamepad1.dpad_left){ // testing
                outtakeInput1 = 1;
                outtakeInput2 = 0.7;
                transferpower = 1;
            }else if (gamepad1.dpad_right){ // testing
                outtakeInput1 = 0.7;
                outtakeInput2 = 1;
                transferpower = 1;
            }


            if (transfer != null) transfer.setPower(transferpower);
            if (outtake1 != null) outtake1.setPower(outtakeInput1);
            if (outtake2 != null) outtake2.setPower(outtakeInput2 * -1);

            telemetry.addData("Front outtake power", outtake1 != null ? outtake1.getPower() : 0);


            telemetry.addData("back outtake power", outtake2 != null ? outtake2.getPower() : 0);

            // ----------------------
            // OPTIONAL: override spinner with buttons
            // ----------------------
            if (spinner != null) {
                if (gamepad1.left_bumper) {
                    spinner.setPower(1.0);     // forward
                } else if (gamepad1.right_bumper) {
                    spinner.setPower(-0.4);    // backward
                }
                // Otherwise, keep spinning continuously (already set above)
            }

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
            telemetry.addData("Spinner Power", spinner != null ? spinner.getPower() : 0);
            telemetry.update();
        }

        // ----------------------
        // STOP ALL MOTORS
        // ----------------------
        if (outtake1 != null) outtake1.setPower(0);
        if (outtake2 != null) outtake2.setPower(0);
        if (intake != null) intake.setPower(0);
        if (transfer != null) transfer.setPower(0);
        if (frontLeft != null) frontLeft.setPower(0);
        if (frontRight != null) frontRight.setPower(0);
        if (backLeft != null) backLeft.setPower(0);
        if (backRight != null) backRight.setPower(0);
        if (spinner != null) spinner.setPower(0);
    }
}
