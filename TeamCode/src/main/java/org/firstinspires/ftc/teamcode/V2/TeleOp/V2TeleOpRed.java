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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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

/*
This is the OpMode for our TeleOp for V1. It uses a command based system with
FTCLib. The robot can intake artifacts, which are detected by a color sensor
and stored in the spindexer, and launch artifacts of a specified color by
automatically rotating the spindexer to the correct position
 */

@TeleOp
public class V2TeleOpRed extends CommandOpMode {
    // subsystems
    IntakeSubsystem intake;
    SpindexerSubsystem spindexer;
    OuttakeSubsystem outtake;
    DrivetrainSubsystem drivetrain;
    EndgameSubsystem endgame;

    // gamepads
    GamepadEx gp1;
    GamepadEx gp2;

    // buttons/triggers
    Trigger leftTrigger;
    Trigger rightTrigger;
    Button rightBumper;
    Button X;
    Button B;
    Button A;
    Trigger spindexerRotating;

    private boolean onRedTeam = true;

    ElapsedTime timer;

    private Follower follower;
    private final Pose startPose = new Pose(9, 39, 0); // just for testing

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        // initialize gamepads
        gp1 = new GamepadEx(gamepad1); // outtake
        gp2 = new GamepadEx(gamepad2); // driving, intake, kickstand

        // initialize subsystems
        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);
        drivetrain = new DrivetrainSubsystem(hardwareMap);
        endgame = new EndgameSubsystem(hardwareMap);

        // set buttons/triggers
        leftTrigger = new Trigger(() -> gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);
        rightTrigger = new Trigger(() -> gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);
        rightBumper = new GamepadButton(gp1, GamepadKeys.Button.RIGHT_BUMPER);
        X = new GamepadButton(gp1, GamepadKeys.Button.X);//transfer
        B = new GamepadButton(gp1, GamepadKeys.Button.B);//Shoot one color
        A = new GamepadButton(gp1, GamepadKeys.Button.A);//shoot one color
        spindexerRotating = new Trigger(() -> spindexer.getSpindexerPower() > 0.05);

        // Bind buttons/triggers with commands
        leftTrigger.whileActiveOnce(new ConditionalCommand(
                new IntakeCommand(intake, spindexer),
                new InstantCommand(),
                () -> /*spindexer.getNumArtifacts() < 3 &&*/
                        outtake.getKickerPos() == outtake.getKickerDown()));
        rightTrigger.whenActive(new ActivateFlywheel(outtake, gamepad1));
        /*X.whenPressed(new ConditionalCommand(
                new InstantCommand(() -> new ShootArtifact(outtake, spindexer).schedule(false)),
                new InstantCommand(),
                () -> outtake.getFlywheelSpeed() > outtake.getTargetSpeed() - 30));*/
        X.whenPressed(new ShootArtifact(outtake, spindexer));
        B.whenPressed(new ConditionalCommand(
                new InstantCommand(() -> new ShootColor(outtake, spindexer, "purple").schedule(false)),
                new InstantCommand(),
                () -> spindexer.getIndexerState().contains("purple") &&
                        outtake.getFlywheelSpeed() > outtake.getTargetSpeed() - 30));
        A.whenPressed(new ConditionalCommand(
                new InstantCommand(() -> new ShootColor(outtake, spindexer, "green").schedule(false)),
                new InstantCommand(),
                () -> spindexer.getIndexerState().contains("green") &&
                        outtake.getFlywheelSpeed() > outtake.getTargetSpeed() - 30));
        // automatically activate intake when spindexer is spinning
        spindexerRotating.whenActive(new InstantCommand(() -> intake.activateIntake()))
                         .whenInactive(new InstantCommand(() -> intake.stopIntake()));

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();

        outtake.setTeam(onRedTeam);
        outtake.startLL();
        timer = new ElapsedTime();

        while(!isStarted() && !isStopRequested()) {
            telemetry.addData("press right bumper to reset spindexer encoder", "");
            if(gamepad1.rightBumperWasPressed()) {
                spindexer.resetSpindexEncoder();
                telemetry.addData("press right bumper to reset spindexer encoder",
                        "spindexer encoder reset");
            }
            telemetry.addData("spindexer position", spindexer.getSpindexerAngle());
            telemetry.addData("spindexer target position", spindexer.getTargetAngle());
            telemetry.update();
            outtake.resetTurretEncoder();
        }
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        follower.update();
        //if(outtake.getMegaTagPos() != null)
        //    follower.setPose(outtake.getMegaTagPos());

        drivetrain.teleOpDrive(gamepad2);//moving the robot
        outtake.calculateLaunch(); // set hood angle and target flywheel speed based on apriltag
        outtake.calculateTurretLL(outtake.getTX()); // aim turret at apriltag
        //outtake.aimTurret(follower.getPose());
        spindexer.powerSpindexer();

        /*
        // move hood up - dpad up
        if(gamepad1.dpad_up) {
            outtake.setHood(outtake.getHoodFar());
        }

        // move hood down - dpad down
        if(gamepad1.dpad_down) {
            outtake.setHood(outtake.getHoodClose());
        }
         */
        // manual spindexer controls if something goes wrong with the automated actions
        if(gamepad1.rightBumperWasPressed()) {
            spindexer.rotateCCW();
        }
        if(gamepad1.leftBumperWasPressed()) {
            spindexer.rotateCW();
        }
        if(gamepad1.yWasPressed()) {
            if(spindexer.inOuttakeMode())
                spindexer.setToIntakeMode();
            else
                spindexer.setToOuttakeMode();
        }

        if(gamepad2.dpad_up) {
            endgame.activateEndgame();
        }
        if(gamepad2.dpad_down) {
            endgame.resetServos();
        }



        telemetry.addData("kickstand position", endgame.getServoPos());
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
        telemetry.addData("loop time", timer.milliseconds());
        timer.reset();

        telemetry.update();
    }
}