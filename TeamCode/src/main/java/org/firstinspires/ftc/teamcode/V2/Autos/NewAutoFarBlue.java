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
public class NewAutoFarBlue extends CommandOpMode {
    Follower follower;
    IntakeSubsystem intake;
    SpindexerSubsystem spindexer;
    OuttakeSubsystem outtake;

    Pose startPose;
    Pose shootPose;
    Pose beforeIntakePose;
    Pose afterIntakePose;

    PathChain goToShoot;
    PathChain goToIntake;
    PathChain intakeBalls;
    PathChain secondShoot;

    private int pathState;
    private ElapsedTime opModeTimer, pathTimer;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);

        startPose = new Pose(57, 9, Math.PI/2);
        shootPose = new Pose(57, 12, Math.PI/2);
        beforeIntakePose = new Pose(44, 36, 0);
        afterIntakePose = new Pose(18, 36, 0);

        goToIntake = follower.pathBuilder()
                .addPath(new BezierLine(startPose, beforeIntakePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), beforeIntakePose.getHeading())
                .build();
        intakeBalls = follower.pathBuilder()
                .addPath(new BezierLine(beforeIntakePose, afterIntakePose))
                .setConstantHeadingInterpolation(0)
                .build();
        goToShoot = follower.pathBuilder()
                .addPath(new BezierLine(afterIntakePose, shootPose))
                .setLinearHeadingInterpolation(afterIntakePose.getHeading(), shootPose.getHeading())
                .build();

        pathState = 0;
        opModeTimer = new ElapsedTime();
        pathTimer = new ElapsedTime();

        outtake.setTeam(false);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        follower.update();
        autoPathUpdate();

        spindexer.powerSpindexer();
        outtake.calculateTurretLL(outtake.getTX());
        outtake.calculateLaunch();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path state", pathState);
        telemetry.addData("number of artifacts", spindexer.getNumArtifacts());
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
                if(outtake.atTargetSpeed()) {
                    new ShootArtifact(outtake, spindexer).schedule(false);
                    if(spindexer.getNumArtifacts() == 0) {

                        follower.followPath(goToIntake);
                        pathState = 2;
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    new IntakeCommand(intake, spindexer).schedule();
                    follower.followPath(intakeBalls);
                    pathState = 3;
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    new ActivateFlywheel(outtake).schedule();
                    follower.followPath(goToShoot);
                    pathState = 4;
                }
                break;
            case 4:
                if(!follower.isBusy()) {

                }
        }
    }


}
