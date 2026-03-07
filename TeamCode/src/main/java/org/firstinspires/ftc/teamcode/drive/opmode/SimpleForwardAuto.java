package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//TODO: Work on getting that damn auto for shooting down. IDS 20 for blue, 24 for red.
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.drive.GoBildaPinpointDriver; // Import from your actual package

@Autonomous(name = "Simple Forward Auto", group = "Auto")
public class SimpleForwardAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the drive system with Pinpoint localizer
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set starting position (x, y, heading)
        // (0,0) is typically the center of the field tile you start on
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        // Build a simple trajectory to move forward 24 inches (2 tiles)
        Trajectory forwardTrajectory = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(24) // Move forward 24 inches
                .build();

        telemetry.addData("Status", "Waiting for start");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.addData("Status", "Moving forward...");
        telemetry.update();

        // Follow the trajectory
        drive.followTrajectory(forwardTrajectory);

        telemetry.addData("Status", "Finished!");
        telemetry.addData("Final X", drive.getPoseEstimate().getX());
        telemetry.addData("Final Y", drive.getPoseEstimate().getY());
        telemetry.update();

        // Keep robot still at end
        while (opModeIsActive()) {
            drive.setMotorPowers(0, 0, 0, 0);
            idle();
        }
    }
}