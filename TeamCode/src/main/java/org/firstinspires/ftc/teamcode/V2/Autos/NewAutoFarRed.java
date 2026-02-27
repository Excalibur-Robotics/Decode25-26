package org.firstinspires.ftc.teamcode.V2.Autos;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.V2.Commands.ActivateFlywheel;
import org.firstinspires.ftc.teamcode.V2.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.V2.Commands.ShootArtifact;
import org.firstinspires.ftc.teamcode.V2.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="FarRed")
public class NewAutoFarRed extends CommandOpMode {
    Follower follower;
    IntakeSubsystem intake;
    SpindexerSubsystem spindexer;
    OuttakeSubsystem outtake;

    Pose startPose;
    Pose shootPose;
    Pose beforeFirstIntake;
    Pose afterFirstIntake;
    Pose beforeSecondIntake;
    Pose afterSecondIntake;
    Pose beforeThirdIntake;
    Pose afterThirdIntake;

    PathChain toFirstShoot;
    PathChain toFirstIntake;
    PathChain intakeFirstBalls;
    PathChain toSecondShoot;
    PathChain toSecondIntake;
    PathChain intakeSecondBalls;
    PathChain toThirdShoot;
    PathChain toThirdIntake;
    PathChain intakeThirdBalls;

    private int pathState;
    private ElapsedTime opModeTimer, pathTimer;
    private boolean motifSeen = false;
    private boolean onRedTeam = true;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);

        startPose = new Pose(86, 8, Math.PI/2);
        shootPose = new Pose(88, 12, Math.PI/2);
        beforeFirstIntake = new Pose(96, 36, 0);
        afterFirstIntake = new Pose(128, 36, 0);
        beforeSecondIntake = new Pose(96, 60, 0);
        afterSecondIntake = new Pose(128, 60, 0);
        beforeThirdIntake = new Pose(96, 84, 0);
        afterThirdIntake = new Pose(128, 84, 0);

        toFirstIntake = follower.pathBuilder()
                .addPath(new BezierLine(startPose, beforeFirstIntake))
                .setLinearHeadingInterpolation(startPose.getHeading(), beforeFirstIntake.getHeading())
                .build();
        intakeFirstBalls = follower.pathBuilder()
                .addPath(new BezierLine(beforeFirstIntake, afterFirstIntake))
                .setConstantHeadingInterpolation(beforeFirstIntake.getHeading())
                .build();
        toFirstShoot = follower.pathBuilder()
                .addPath(new BezierLine(afterFirstIntake, shootPose))
                .setLinearHeadingInterpolation(afterFirstIntake.getHeading(), shootPose.getHeading())
                .build();
        toSecondIntake = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, beforeSecondIntake))
                .setLinearHeadingInterpolation(shootPose.getHeading(), beforeSecondIntake.getHeading())
                .build();
        intakeSecondBalls = follower.pathBuilder()
                .addPath(new BezierLine(beforeSecondIntake, afterSecondIntake))
                .setConstantHeadingInterpolation(beforeFirstIntake.getHeading())
                .build();
        toSecondShoot = follower.pathBuilder()
                .addPath(new BezierLine(afterSecondIntake, shootPose))
                .setLinearHeadingInterpolation(afterSecondIntake.getHeading(), shootPose.getHeading())
                .build();
        toThirdIntake = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, beforeThirdIntake))
                .setLinearHeadingInterpolation(shootPose.getHeading(), beforeThirdIntake.getHeading())
                .build();
        intakeThirdBalls = follower.pathBuilder()
                .addPath(new BezierLine(beforeThirdIntake, afterThirdIntake))
                .setConstantHeadingInterpolation(beforeFirstIntake.getHeading())
                .build();
        toThirdShoot = follower.pathBuilder()
                .addPath(new BezierLine(afterThirdIntake, shootPose))
                .setLinearHeadingInterpolation(afterThirdIntake.getHeading(), shootPose.getHeading())
                .build();

        pathState = 0;
        opModeTimer = new ElapsedTime();
        pathTimer = new ElapsedTime();

        outtake.setTeam(onRedTeam);
        outtake.setLLPipeline(0);
        outtake.resetTurretEncoder();
        spindexer.resetSpindexEncoder();
        follower.setStartingPose(startPose);
        outtake.startLL();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        follower.update();
        autoPathUpdate();

        spindexer.powerSpindexer();
        //outtake.calculateTurretLL(outtake.getTX());
        if(motifSeen)
            outtake.aimTurret(follower.getPose());
        outtake.calculateLaunch();
        if(Math.abs(spindexer.getSpindexerPower()) > 0.1) {
            intake.activateIntake();
        }

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path state", pathState);
        telemetry.addData("number of artifacts", spindexer.getNumArtifacts());
        telemetry.addLine();
        telemetry.addData("flywheel speed", outtake.getFlywheelSpeed()); // in rpm
        telemetry.addData("target speed", outtake.getTargetSpeed());
        telemetry.addData("flywheel error", Math.abs(outtake.getFlywheelSpeed() - outtake.getTargetSpeed()));
        telemetry.addData("flywheel power", outtake.flywheel.getPower());
        telemetry.addData("kicker position", outtake.getKickerPos());
        telemetry.addData("hood angle", outtake.getHoodAngle());
        telemetry.addData("turret position", outtake.getTurretPos());
        telemetry.addLine();
        telemetry.addData("tx", outtake.getTX());
        telemetry.addData("ta", outtake.getTA());
        telemetry.addData("OpMode loop time", opModeTimer.milliseconds());
        opModeTimer.reset();
        telemetry.update();
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                new ActivateFlywheel(outtake).schedule();
                pathState = 1;
                break;
            case 1:
                if(!outtake.atTargetSpeed()) {
                    if (!motifSeen && outtake.getApriltagID() > 20 && outtake.getApriltagID() < 24) {
                        spindexer.sort(outtake.getApriltagID());
                        motifSeen = true;
                        outtake.setTeam(onRedTeam);
                    }
                }
                else {
                    motifSeen = true;
                    new ShootArtifact(outtake, spindexer).schedule(false);
                    if(spindexer.getNumArtifacts() == 0) {
                        follower.followPath(toFirstIntake);
                        pathState = 2;
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    new IntakeCommand(intake, spindexer).schedule();
                    follower.followPath(intakeFirstBalls);
                    pathState = 3;
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    new ActivateFlywheel(outtake).schedule();
                    follower.followPath(toFirstShoot);
                    pathState = 4;
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    if(outtake.atTargetSpeed()) {
                        new ShootArtifact(outtake, spindexer).schedule(false);
                        if (spindexer.getNumArtifacts() == 0) {
                            follower.followPath(toSecondIntake);
                            pathState = 5;
                        }
                    }
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    new IntakeCommand(intake, spindexer).schedule();
                    follower.followPath(intakeSecondBalls);
                    pathState = 6;
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    new ActivateFlywheel(outtake).schedule();
                    follower.followPath(toSecondShoot);
                    pathState = 7;
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    if(outtake.atTargetSpeed()) {
                        new ShootArtifact(outtake, spindexer).schedule(false);
                        if (spindexer.getNumArtifacts() == 0) {
                            follower.followPath(toThirdIntake);
                            pathState = 8;
                        }
                    }
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    new IntakeCommand(intake, spindexer).schedule();
                    follower.followPath(intakeThirdBalls);
                    pathState = 9;
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    new ActivateFlywheel(outtake).schedule();
                    follower.followPath(toThirdShoot);
                    pathState = 10;
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    if(outtake.atTargetSpeed()) {
                        new ShootArtifact(outtake, spindexer).schedule(false);
                        if (spindexer.getNumArtifacts() == 0) {
                            pathState = -1;
                        }
                    }
                }
                break;
        }
    }
}
