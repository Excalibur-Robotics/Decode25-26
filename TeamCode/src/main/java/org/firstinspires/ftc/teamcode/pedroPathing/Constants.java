package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.7) // robot mass in kg - need to measure
            .forwardZeroPowerAcceleration(-29.67254318817769)
            .lateralZeroPowerAcceleration(-63.223690012824356)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0, 0, 0.04))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0, 0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015, 0, 0, 0.6, 0.01))
            .centripetalScaling(0.0005);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.9, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frwheel")
            .rightRearMotorName("brwheel")
            .leftRearMotorName("blwheel")
            .leftFrontMotorName("flwheel")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(62.016429030050446) // use ForwardVelocityTuner to tune
            .yVelocity(49.42297074926182); // use LateralVelocityTuner to tune

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(1.093) // y offset in inches
            .strafePodX(-5.69) // x offset in inches
            .distanceUnit(DistanceUnit.INCH) // units
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}