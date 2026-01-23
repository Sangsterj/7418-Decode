package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DriveAllMotorsOpMode")
public class DriveAllMotorsOpMode extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx outtake;
    private DcMotor intake;
    private Servo spinner;
    private DcMotor transfer;

    @Override
    public void runOpMode() {

        // ----------------------
        // SAFE MOTOR MAPPING
        // ----------------------
        try { frontLeft = hardwareMap.get(DcMotor.class, "frontLeft"); } catch (Exception ignored) {}
        try { frontRight = hardwareMap.get(DcMotor.class, "frontRight"); } catch (Exception ignored) {}
        try { backLeft = hardwareMap.get(DcMotor.class, "backLeft"); } catch (Exception ignored) {}
        try { backRight = hardwareMap.get(DcMotor.class, "backRight"); } catch (Exception ignored) {}

        try { spinner = hardwareMap.servo.get("spinner"); } catch (Exception ignored) {}

        try { outtake = (DcMotorEx) hardwareMap.get(DcMotor.class, "outtake"); } catch (Exception ignored) {}

        try { intake = hardwareMap.get(DcMotor.class, "intake"); } catch (Exception ignored) {}

        try { transfer = hardwareMap.get(DcMotor.class, "transfer");} catch (Exception ignored){}
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

            double max = Math.max(Math.max(Math.abs(flPower), Math.abs(blPower)),
                    Math.max(Math.abs(frPower), Math.abs(brPower)));

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
            // OUTTAKE MOTORS (OVERCLOCKED)
            // ----------------------
            double forward = gamepad1.right_trigger; // 0..1
            double reverse = gamepad1.left_trigger;  // 0..1
            double input = forward - reverse;        // -1..1

            // Deadband
            if (Math.abs(input) < 0.05) input = 0;

            if (outtake != null) {
                double ticksPerRev = 28;      // REV HD Hex encoder
                double overclockRPM = 20000;  // extreme speed for testing
                double targetRPM = input * overclockRPM;
                double targetTPS = (targetRPM / 60.0) * ticksPerRev;

                outtake.setVelocity(targetTPS);

                telemetry.addData("Outtake RPM", targetRPM);
            }

            //-----------------------
            // SERVO MOTOR
            //-----------------------
            spinner.setPosition(0.5);
            boolean leftSpin = gamepad1.right_bumper;
            boolean rightSpin = gamepad1.left_bumper;

            if (leftSpin == true){
                spinner.setPosition(0);
            } else if (rightSpin == true) {
                spinner.setPosition(1);
            } else {
                spinner.setPosition(0);
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
            telemetry.addData("Drive", "FL %.2f | FR %.2f | BL %.2f | BR %.2f",
                    flPower, frPower, blPower, brPower);
            telemetry.addData("Outtake Power Raw", input);
            telemetry.addData("Intake Power", intakePower);
            telemetry.addData("Transfer Power", transferPower);
            telemetry.update();
        }

        // ----------------------
        // STOP ALL MOTORS
        // ----------------------
        if (outtake != null) outtake.setVelocity(0);
        if (intake != null) intake.setPower(0);
        if (transfer != null) transfer.setPower(0);
        if (frontLeft != null) frontLeft.setPower(0);
        if (frontRight != null) frontRight.setPower(0);
        if (backLeft != null) backLeft.setPower(0);
        if (backRight != null) backRight.setPower(0);
    }
}
