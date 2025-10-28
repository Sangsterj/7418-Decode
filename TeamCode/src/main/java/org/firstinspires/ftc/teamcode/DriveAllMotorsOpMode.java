package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DriveAllMotorsOpMode")
public class DriveAllMotorsOpMode extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor auxMotor; // REV HD Hex Motor on an auxiliary port

    @Override
    public void runOpMode() {
        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Initialize auxiliary motor
        auxMotor = hardwareMap.get(DcMotor.class, "auxMotor");

        // Motor direction setup
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        auxMotor.setDirection(DcMotor.Direction.FORWARD); // adjust if needed

        // Brake mode
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        auxMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // --- Motor control state variables ---
        boolean spinningForward = false;
        boolean spinningBackward = false;
        boolean rtPressedLast = false;
        boolean ltPressedLast = false;

        while (opModeIsActive()) {
            // --- Mecanum Drive ---
            double y = -gamepad1.left_stick_y;  // forward/backward
            double x = -gamepad1.left_stick_x;  // strafe
            double rx = gamepad1.right_stick_x; // rotation

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

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // --- Toggle-based Hex Motor Control ---
            boolean rtPressedNow = gamepad1.right_trigger > 0.5;
            boolean ltPressedNow = gamepad1.left_trigger > 0.5;

            // Detect rising edge for RT
            if (rtPressedNow && !rtPressedLast) {
                if (spinningForward) {
                    spinningForward = false;
                } else {
                    spinningForward = true;
                    spinningBackward = false;
                }
            }

            // Detect rising edge for LT
            if (ltPressedNow && !ltPressedLast) {
                if (spinningBackward) {
                    spinningBackward = false;
                } else {
                    spinningBackward = true;
                    spinningForward = false;
                }
            }

            rtPressedLast = rtPressedNow;
            ltPressedLast = ltPressedNow;

            // Apply motor power
            if (spinningForward) {
                auxMotor.setPower(1.0);
            } else if (spinningBackward) {
                auxMotor.setPower(-1.0);
            } else {
                auxMotor.setPower(0.0);
            }

            // --- Telemetry ---
            telemetry.addData("FL", frontLeftPower);
            telemetry.addData("FR", frontRightPower);
            telemetry.addData("BL", backLeftPower);
            telemetry.addData("BR", backRightPower);
            telemetry.addData("Aux Motor Power", auxMotor.getPower());
            telemetry.addData("Forward Spin", spinningForward);
            telemetry.addData("Backward Spin", spinningBackward);
            telemetry.update();
        }

        // Stop everything on end
        auxMotor.setPower(0);
    }
}
