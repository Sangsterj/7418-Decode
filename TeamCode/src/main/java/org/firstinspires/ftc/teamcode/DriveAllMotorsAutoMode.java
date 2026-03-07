package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "DriveAllMotorsAutoMode")
public class DriveAllMotorsAutoMode extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx outtake1;
    private DcMotorEx outtake2;
    private DcMotor intake;
    private CRServo spinner;
    private DcMotor transfer;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        // ----------------------
        // SAFE HARDWARE MAPPING
        // ----------------------
        try {
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        } catch (Exception ignored) {
        }
        try {
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        } catch (Exception ignored) {
        }
        try {
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        } catch (Exception ignored) {
        }
        try {
            backRight = hardwareMap.get(DcMotor.class, "backRight");
        } catch (Exception ignored) {
        }

        try {
            spinner = hardwareMap.get(CRServo.class, "spinner");
        } catch (Exception ignored) {
        }
        try {
            outtake1 = (DcMotorEx) hardwareMap.get(DcMotor.class, "outtake1");
        } catch (Exception ignored) {
        }
        try {
            outtake2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "outtake2");
        } catch (Exception ignored) {
        }
        try {
            intake = hardwareMap.get(DcMotor.class, "intake");
        } catch (Exception ignored) {
        }
        try {
            transfer = hardwareMap.get(DcMotor.class, "transfer");
        } catch (Exception ignored) {
        }

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



        waitForStart();

        if (isStopRequested()){
            return;
        }

        timer.reset();
        // ----------------------
        // START SPINNER CONTINUOUSLY
        // ----------------------
        if (spinner != null) spinner.setPower(1.0); // continuous forward spin




            waitForStart();

            if (opModeIsActive()) {

                // Drive forward
                frontLeft.setPower(0.5);
                frontRight.setPower(0.5);
                backLeft.setPower(0.5);
                backRight.setPower(0.5);

                sleep(2000); // drive for 2 seconds

                // Stop
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }

            // ----------------------
            // STOP ALL MOTORS
            // ----------------------
        }
    }

