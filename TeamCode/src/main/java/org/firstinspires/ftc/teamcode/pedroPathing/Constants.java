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
            .forwardZeroPowerAcceleration(-57.3529403763483)
            .lateralZeroPowerAcceleration(-81.35358357682023)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.12, 0, 0, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0, 0, 0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0, 0.6, 0.01))
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
            .xVelocity(53.5705100232222) // use ForwardVelocityTuner to tune
            .yVelocity(38.65271452655942); // use LateralVelocityTuner to tune

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