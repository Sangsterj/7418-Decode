package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.List;

/**
 * Combined OpMode: AprilTag + Color Sensor
 */
@TeleOp(name = "Concept: AprilTag + Color", group = "Concept")
public class TestingStuff extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;

    private AprilTagProcessor aprilTag;
    private PredominantColorProcessor colorSensor;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        try {
            // Initialize AprilTag processor
            aprilTag = new AprilTagProcessor.Builder()
                    .setDrawTagOutline(true)
                    .build();

            // Initialize Color Sensor processor
            colorSensor = new PredominantColorProcessor.Builder()
                    .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                    .setSwatches(
                            PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                            PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                            PredominantColorProcessor.Swatch.RED,
                            PredominantColorProcessor.Swatch.BLUE,
                            PredominantColorProcessor.Swatch.YELLOW,
                            PredominantColorProcessor.Swatch.BLACK,
                            PredominantColorProcessor.Swatch.WHITE)
                    .build();

            // Build VisionPortal
            VisionPortal.Builder builder = new VisionPortal.Builder();

            if (USE_WEBCAM) {
                builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            } else {
                // If using phone camera
                // builder.setCamera(BuiltinCameraDirection.BACK);
            }

            builder.setCameraResolution(new Size(640, 480));
            builder.enableLiveView(true);

            builder.addProcessor(aprilTag);
            builder.addProcessor(colorSensor);

            visionPortal = builder.build();

            telemetry.addData("Status", "Vision Portal initialized");
            telemetry.update();

        } catch (Exception e) {
            telemetry.addData("ERROR", "Initialization failed: " + e.getMessage());
            telemetry.update();
            sleep(5000);
            return;
        }

        telemetry.addData("Status", "Press START");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // ----- AprilTag telemetry -----
            if (aprilTag != null) {
                List<AprilTagDetection> detections = aprilTag.getDetections();
                telemetry.addData("AprilTags Detected", detections.size());
                for (AprilTagDetection detection : detections) {
                    if (detection.metadata != null) {
                        telemetry.addLine(String.format("Tag %d: %s", detection.id, detection.metadata.name));
                        telemetry.addLine(String.format("XYZ: %.1f, %.1f, %.1f", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                        telemetry.addLine(String.format("PRY: %.1f, %.1f, %.1f", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    } else {
                        telemetry.addLine(String.format("Tag %d: Unknown", detection.id));
                    }
                }
            }

            // ----- Color Sensor telemetry -----
            if (colorSensor != null) {
                PredominantColorProcessor.Result result = colorSensor.getAnalysis();
                telemetry.addData("Color Match", result.closestSwatch);
                telemetry.addLine(String.format("RGB: %3d, %3d, %3d", result.RGB[0], result.RGB[1], result.RGB[2]));
                telemetry.addLine(String.format("HSV: %3d, %3d, %3d", result.HSV[0], result.HSV[1], result.HSV[2]));
                telemetry.addLine(String.format("YCrCb: %3d, %3d, %3d", result.YCrCb[0], result.YCrCb[1], result.YCrCb[2]));
            }

            telemetry.update();
            sleep(50);
        }

        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
