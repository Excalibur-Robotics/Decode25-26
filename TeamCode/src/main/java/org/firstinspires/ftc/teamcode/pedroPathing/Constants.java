package org.firstinspires.ftc.teamcode.pedroPathing;

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
            .mass(5); // robot mass in kg - need to measure
            //.forwardZeroPowerAcceleration(5) // use ForwardZeroPowerAcceleration to tune
            //.lateralZeroPowerAcceleration(5) // use LateralZeroPowerAcceleration to tune
            //.translationalPIDFCoefficients(new PIDFCoefficients())
            //.headingPIDFCoefficients(new PIDFCoefficients())
            //.drivePIDFCoefficients(new FilteredPIDFCoefficients())
            //.centripetalScaling();

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

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
            .xVelocity(5) // use ForwardVelocityTuner to tune
            .yVelocity(5); // use LateralVelocityTuner to tune

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-1.093) // y offset in inches
            .strafePodX(-5.69) // x offset in inches
            .distanceUnit(DistanceUnit.INCH) // units
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}