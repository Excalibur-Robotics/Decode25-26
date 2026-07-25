package org.firstinspires.ftc.teamcode.V2.Autos;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.V2.Commands.ActivateFlywheel;
import org.firstinspires.ftc.teamcode.V2.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.V2.Commands.ShootAll;
import org.firstinspires.ftc.teamcode.V2.Commands.ShootArtifact;
import org.firstinspires.ftc.teamcode.V2.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.V2.TeleOp.V2TeleOpBlue;
import org.firstinspires.ftc.teamcode.V2.TeleOp.V2TeleOpRed;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;

@Autonomous(name = "CloseBlue")
public class CloseBlueAuto extends CommandOpMode {
    Follower follower;
    IntakeSubsystem intake;
    SpindexerSubsystem spindexer;
    OuttakeSubsystem outtake;

    Pose startPose;
    Pose firstShootPose;
    Pose beforeFirstIntake;
    Pose afterFirstIntake;
    Pose beforeSecondIntake;
    Pose afterSecondIntake;
    Pose shootPose;
    Pose gatePose;
    Pose controlPoint;
    Pose finalPose;

    PathChain toFirstShoot;
    PathChain toFirstRow;
    PathChain intakeFirstRow;
    PathChain toSecondShoot;
    PathChain toSecondRow;
    PathChain intakeSecondRow;
    PathChain toThirdShoot;
    PathChain toGate;
    PathChain toShoot;
    PathChain toFinalPose;

    private int pathState;
    private ElapsedTime opModeTimer, pathTimer;
    private boolean motifSeen;
    private int id = 0;
    private boolean onRedTeam = false;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);

        startPose = new Pose(20.1, 123.1, Math.toRadians(144));
        firstShootPose = new Pose(49, 101, Math.toRadians(136));
        beforeFirstIntake = new Pose(48, 84.0, Math.PI);
        afterFirstIntake = new Pose(24, 84.0, Math.PI);
        beforeSecondIntake = new Pose(50, 60.0, Math.PI);
        afterSecondIntake = new Pose(18, 58.0, Math.PI);
        shootPose = new Pose(50, 94, Math.PI);
        gatePose = new Pose(15, 57, Math.toRadians(145));
        controlPoint = new Pose(34, 58);
        finalPose = new Pose(20, 92, Math.PI);


        toFirstShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, firstShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstShootPose.getHeading())
                .build();
        toFirstRow = follower.pathBuilder()
                .addPath(new BezierLine(firstShootPose, beforeFirstIntake))
                .setLinearHeadingInterpolation(firstShootPose.getHeading(), beforeFirstIntake.getHeading())
                .build();
        intakeFirstRow = follower.pathBuilder()
                .addPath(new BezierLine(beforeFirstIntake, afterFirstIntake))
                .setConstantHeadingInterpolation(beforeFirstIntake.getHeading())
                .build();
        toSecondShoot = follower.pathBuilder()
                .addPath(new BezierLine(afterFirstIntake, firstShootPose))
                .setLinearHeadingInterpolation(afterFirstIntake.getHeading(), firstShootPose.getHeading())
                .build();
        toSecondRow = follower.pathBuilder()
                .addPath(new BezierLine(firstShootPose, beforeSecondIntake))
                .setLinearHeadingInterpolation(firstShootPose.getHeading(), beforeSecondIntake.getHeading())
                .build();
        intakeSecondRow = follower.pathBuilder()
                .addPath(new BezierLine(beforeSecondIntake, afterSecondIntake))
                .setConstantHeadingInterpolation(beforeSecondIntake.getHeading())
                .build();
        toThirdShoot = follower.pathBuilder()
                .addPath(new BezierCurve(afterSecondIntake, controlPoint, shootPose))
                .setLinearHeadingInterpolation(afterSecondIntake.getHeading(), shootPose.getHeading())
                .build();
        toGate = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, controlPoint, gatePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), gatePose.getHeading())
                .build();
        toShoot = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, shootPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), shootPose.getHeading())
                .build();
        toFinalPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, finalPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();

        pathState = 0;
        opModeTimer = new ElapsedTime();
        pathTimer = new ElapsedTime();

        outtake.setTeam(onRedTeam);
        //outtake.setLLPipeline(0); // uncomment if trying to scan motif
        motifSeen = true; // make false if trying to scan motif
        outtake.resetTurretEncoder();
        spindexer.resetSpindexEncoder();
        ArrayList<String> spindexerState = new ArrayList<String>();
        spindexerState.add("purple");
        spindexerState.add("purple");
        spindexerState.add("green");
        spindexer.setIndexerState(spindexerState);
        follower.setStartingPose(startPose);
        outtake.startLL();

        telemetry.addLine("initialized");
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        follower.update();
        autoPathUpdate();

        spindexer.powerSpindexer();
        if(motifSeen) {
            if (outtake.getTX() == 0)
                outtake.aimTurret(follower.getPose());
            else
                outtake.calculateTurretLL(outtake.getTX());
        }
        else {
            outtake.scanMotif(follower.getPose());
        }
        outtake.calculateHood(follower.getPose());
        outtake.calculateFlywheel(follower.getPose());
        if(Math.abs(spindexer.getSpindexerPower()) > 0.1) {
            intake.activateIntake();
        }


        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path state", pathState);
        telemetry.addData("number of artifacts", spindexer.getNumArtifacts());
        telemetry.addData("OpMode loop time", opModeTimer.milliseconds());
        opModeTimer.reset();
        telemetry.addLine();
        telemetry.addData("motif ID", id);
        telemetry.addLine();
        ArrayList<String> indexer = spindexer.getIndexerState();
        telemetry.addData("spindexer", spindexer.inOuttakeMode() ? "  " +
                indexer.get(2).charAt(0) : " " + indexer.get(2).charAt(0) + " " + indexer.get(1).charAt(0));
        telemetry.addData("state        ", spindexer.inOuttakeMode() ? " " + indexer.get(0).charAt(0)
                + " " + indexer.get(1).charAt(0) : "   " + indexer.get(0).charAt(0));
        telemetry.addData("# artifacts", spindexer.getNumArtifacts());
        telemetry.update();
    }

    @Override
    public void reset() {
        CommandScheduler.getInstance().reset();
        V2TeleOpBlue.indexer = spindexer.getIndexerState();
        V2TeleOpRed.indexer = spindexer.getIndexerState();

        V2TeleOpRed.motifID = id;
        V2TeleOpBlue.motifID = id;
        if(id != 0) {
            V2TeleOpRed.motifFromAuto = true;
            V2TeleOpBlue.motifFromAuto = true;
        }

        V2TeleOpRed.startingSpindexAngle = spindexer.getTargetAngle();
        V2TeleOpBlue.startingSpindexAngle = spindexer.getTargetAngle();

        V2TeleOpRed.startX = follower.getPose().getX();
        V2TeleOpRed.startY = follower.getPose().getY();
        V2TeleOpRed.startHeading = Math.toDegrees(follower.getPose().getHeading());

        V2TeleOpBlue.startX = follower.getPose().getX();
        V2TeleOpBlue.startY = follower.getPose().getY();
        V2TeleOpBlue.startHeading = Math.toDegrees(follower.getPose().getHeading());
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                new ActivateFlywheel(outtake).schedule();
                follower.followPath(toFirstShoot);
                pathState = 1;
                break;
            case 1:
                if(!follower.isBusy()) {
                    if(id == 0 && outtake.getApriltagID() > 20 && outtake.getApriltagID() < 24) {
                        id = outtake.getApriltagID();
                    }
                    outtake.setTeam(onRedTeam);
                    motifSeen = true;
                    if(outtake.atTargetSpeed()) {
                        new ShootAll(outtake, spindexer, id).schedule(false);
                        if (spindexer.getNumArtifacts() == 0) {
                            follower.followPath(toFirstRow);
                            pathState = 2;
                        }
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    new IntakeCommand(intake, spindexer).schedule();
                    follower.followPath(intakeFirstRow, 0.5, false);
                    pathState = 3;
                    pathTimer.reset();
                }
                break;
            case 3:
                //if(spindexer.getNumArtifacts() == 3 || pathTimer.milliseconds() > 3000) {
                if(!follower.isBusy()) {
                    follower.followPath(toSecondShoot, 1, false);
                    pathState = 4;
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    if(outtake.atTargetSpeed()) {
                        new ShootAll(outtake, spindexer, id).schedule(false);
                        if (spindexer.getNumArtifacts() == 0) {
                            follower.followPath(toSecondRow);
                            pathState = 5;
                        }
                    }
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    new IntakeCommand(intake, spindexer).schedule();
                    follower.followPath(intakeSecondRow, 0.5, false);
                    pathState = 6;
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(toThirdShoot, 1, false);
                    pathState = 7;
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    if(outtake.atTargetSpeed()) {
                        new ShootAll(outtake, spindexer, id).schedule(false);
                        if (spindexer.getNumArtifacts() == 0) {
                            follower.followPath(toFinalPose);
                            //new IntakeCommand(intake, spindexer).schedule();
                            pathState = -1;
                            pathTimer.reset();
                        }
                    }
                }
                break;
            case 8:
                if(spindexer.getNumArtifacts() == 3 || pathTimer.milliseconds() > 4000) {
                    follower.followPath(toShoot);
                    pathState = 7;
                }
        }
    }


}
