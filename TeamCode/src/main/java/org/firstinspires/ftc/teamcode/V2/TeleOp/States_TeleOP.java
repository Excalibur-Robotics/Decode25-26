package org.firstinspires.ftc.teamcode.V2.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.V2.Commands.ActivateFlywheel;
import org.firstinspires.ftc.teamcode.V2.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.V2.Commands.ShootArtifact;
import org.firstinspires.ftc.teamcode.V2.Commands.ShootColor;
import org.firstinspires.ftc.teamcode.V2.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.EndgameSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;

@Disabled
@TeleOp
public class States_TeleOP extends CommandOpMode {
    IntakeSubsystem intake;
    SpindexerSubsystem spindexer;
    OuttakeSubsystem outtake;
    DrivetrainSubsystem drivetrain;
    EndgameSubsystem kickstand;
    GamepadEx gp1;

    Trigger leftTrigger;
    Trigger rightTrigger;
    Button rightBumper;
    Button X;
    Button B;
    Button A;
    Trigger spindexerRotating;

    private boolean onRedTeam = true;
    private Follower follower;
    private final Pose startPose = new Pose(9, 39, 0); // just for testing

    @Override
    public void initialize() {
        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);
        drivetrain = new DrivetrainSubsystem(hardwareMap);
        kickstand = new EndgameSubsystem(hardwareMap);

        gp1 = new GamepadEx(gamepad1);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // initialize triggers
        leftTrigger = new Trigger(() -> gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);
        rightTrigger = new Trigger(() -> gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);
        X = new GamepadButton(gp1, GamepadKeys.Button.X);
        B = new GamepadButton(gp1, GamepadKeys.Button.B);
        A = new GamepadButton(gp1, GamepadKeys.Button.A);
        spindexerRotating = new Trigger(() -> spindexer.getSpindexerPower() > 0.05);

        leftTrigger.whileActiveOnce(new IntakeCommand(intake, spindexer));
        rightTrigger.whenActive(new ActivateFlywheel(outtake, gamepad1));
        X.whenPressed(new ConditionalCommand(
                new ShootArtifact(outtake, spindexer),
                new InstantCommand(),
                () -> outtake.getFlywheelSpeed() > outtake.getTargetSpeed() - 30));
        A.whenPressed(new ConditionalCommand(
                new ShootColor(outtake, spindexer, "green"),
                new InstantCommand(),
                () -> outtake.getFlywheelSpeed() > outtake.getTargetSpeed() - 30));
        B.whenPressed(new ConditionalCommand(
                new ShootColor(outtake, spindexer, "purple"),
                new InstantCommand(),
                () -> outtake.getFlywheelSpeed() > outtake.getTargetSpeed() - 30));
        spindexerRotating.whenActive(new InstantCommand(() -> intake.activateIntake()))
                .whenInactive(new InstantCommand(() -> intake.stopIntake()));
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        drivetrain.teleOpDrive(gamepad1);
        outtake.calculateLaunch(); // set hood angle and target flywheel speed based on apriltag
        outtake.calculateTurretLL(outtake.getTX()); // aim turret at apriltag
        //outtake.aimTurret(follower.getPose());
        spindexer.powerSpindexer();



        telemetry.addData("kickstand position", kickstand.getServoPos());
        ArrayList<String> indexer = spindexer.getIndexerState();
        telemetry.addData("spindexer" , spindexer.inOuttakeMode() ? "  " +
                indexer.get(2).charAt(0) : " " + indexer.get(2).charAt(0) + " " + indexer.get(1).charAt(0));
        telemetry.addData("state        ", spindexer.inOuttakeMode() ? " " + indexer.get(0).charAt(0)
                + " " + indexer.get(1).charAt(0) : "   " + indexer.get(0).charAt(0));
        telemetry.addData("# artifacts", spindexer.getNumArtifacts());
        telemetry.addLine();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addLine();
        telemetry.addData("purple pixels", spindexer.getPurplePixels());
        telemetry.addData("green pixels", spindexer.getGreenPixels());
        telemetry.addData("Artifact in intake slot", spindexer.detectsArtifact());
        FtcDashboard.getInstance().startCameraStream(spindexer.LT, 0);
        telemetry.addLine();
        telemetry.addData("spindexer position", spindexer.getSpindexerAngle());
        telemetry.addData("spindexer target position", spindexer.getTargetAngle());
        telemetry.addData("spindexer error", Math.abs(spindexer.getSpindexerAngle()-spindexer.getTargetAngle()));
        telemetry.addData("spindexer power", spindexer.getSpindexerPower());
        telemetry.addData("spindexer mode", spindexer.inOuttakeMode() ? "outtake" : "intake");
        telemetry.addLine();
        telemetry.addData("flywheel speed", outtake.getFlywheelSpeed()); // in rpm
        telemetry.addData("target speed", outtake.getTargetSpeed());
        telemetry.addData("flywheel error", Math.abs(outtake.getFlywheelSpeed() -outtake.getTargetSpeed()));
        telemetry.addData("flywheel power", outtake.flywheel.getPower());
        telemetry.addData("kicker position", outtake.getKickerPos());
        telemetry.addData("hood angle", outtake.getHoodAngle());
        telemetry.addData("turret position", outtake.getTurretPos());
        telemetry.addLine();
        telemetry.addData("team color", onRedTeam ? "RED" : "BLUE");
        telemetry.addData("tx", outtake.getTX());
        telemetry.addData("ta", outtake.getTA());
        telemetry.addData("apriltag ID", outtake.getApriltagID());
    }


}

