package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Arrays;
import java.util.List;

/*
 * Two wheel localizer using the goBILDA Pinpoint odometry computer.
 * The dead wheels are connected directly to the Pinpoint.
 */
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {

    // Conversion: 13.26291192 ticks/mm (Swingarm) * 25.4 mm/in = 336.877962768 ticks/in
    public static double TICKS_PER_INCH = 336.877962768;

    private GoBildaPinpointDriver pinpoint;

    // TODO: Measure these in mm from the center of the robot
    public static double PINPOINT_X_OFFSET = 0; 
    public static double PINPOINT_Y_OFFSET = 0; 

    // Wheel positions relative to robot center (for Road Runner math)
    public static double PARALLEL_X = 0;      
    public static double PARALLEL_Y = 6;      

    public static double PERPENDICULAR_X = -6; 
    public static double PERPENDICULAR_Y = -6; 

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive sampleMecanumDrive) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),                    
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90)) 
        ));

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        
        pinpoint.setOffsets(PINPOINT_X_OFFSET, PINPOINT_Y_OFFSET);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        pinpoint.resetPosAndIMU();
    }

    @Override
    public double getHeading() {
        pinpoint.update();
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }

    @Override
    public Double getHeadingVelocity() {
        pinpoint.update();
        return pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        pinpoint.update();
        // Convert raw ticks to inches
        return Arrays.asList(
                pinpoint.getEncoderX() / TICKS_PER_INCH,
                pinpoint.getEncoderY() / TICKS_PER_INCH
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        pinpoint.update();
        // Pinpoint provides robot velocity in specified units
        return Arrays.asList(
                pinpoint.getVelX(DistanceUnit.INCH),
                pinpoint.getVelY(DistanceUnit.INCH)
        );
    }

    public void recalibrateIMU() {
        pinpoint.recalibrateIMU();
    }

    public GoBildaPinpointDriver.DeviceStatus getDeviceStatus() {
        pinpoint.update();
        return pinpoint.getDeviceStatus();
    }
}
