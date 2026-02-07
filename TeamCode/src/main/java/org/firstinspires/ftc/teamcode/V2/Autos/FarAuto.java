package org.firstinspires.ftc.teamcode.V2.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.V2.Commands.ActivateFlywheel;
import org.firstinspires.ftc.teamcode.V2.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.V2.Commands.ShootArtifact;
import org.firstinspires.ftc.teamcode.V2.Commands.ShootColor;
import org.firstinspires.ftc.teamcode.V2.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.SpindexerSubsystem;

/*
This contains the poses and paths for the far auto routine and the state
machine that determines the current path to follow.
So far this only contains the raw pathing of the robot, not shooting,
limelight, or indexing.

Routine:
The robot shoots the preloads from the start position, goes to pick up
the closest three artifacts, returns to the launch zone, and shoots those three.
*/

public class FarAuto extends AutoRoutine{
    // declare poses
    public Pose startPose;
    public Pose beforePickup;
    public Pose afterPickup;
    public Pose shootPose;

    // declare paths
    public PathChain goToPickup;
    public PathChain intakeBalls;
    public PathChain goToSecondShoot;

    // declare path state, used for the state machine
    private int pathState;

    public IntakeSubsystem intake;
    public SpindexerSubsystem spindexer;
    public OuttakeSubsystem outtake;

    public ActivateFlywheel activateFlywheel;
    public IntakeCommand activateIntake;
    public ShootColor shootPurple;
    public ShootColor shootGreen;
    public ShootArtifact shootArtifact;

    // constructor initializes poses and paths
    public FarAuto(Follower follower, IntakeSubsystem intake,
                   SpindexerSubsystem spindexer, OuttakeSubsystem outtake, boolean redTeam) {
        super(follower, intake, spindexer, outtake, redTeam, false);

        // sets poses based on if we are red or blue
        if(redTeam) {
            startPose = new Pose(87, 9, Math.PI/2);
            beforePickup = new Pose(100, 36, 0);
            afterPickup = new Pose(126, 36, 0);
            shootPose = new Pose(87, 12, 0);
        }
        else {
            startPose = new Pose(57, 9, Math.PI/2);
            beforePickup = new Pose(44, 36, Math.PI);
            afterPickup = new Pose(18, 36, Math.PI);
            shootPose = new Pose(57, 12, Math.PI);
        }

        // set paths, same for both colors
        goToPickup = follower.pathBuilder()
                .addPath(new BezierLine(startPose, beforePickup))
                .setLinearHeadingInterpolation(startPose.getHeading(), beforePickup.getHeading())
                .build();
        intakeBalls = follower.pathBuilder()
                .addPath(new BezierLine(beforePickup, afterPickup))
                .setConstantHeadingInterpolation(beforePickup.getHeading())
                .build();
        goToSecondShoot = follower.pathBuilder()
                .addPath(new BezierLine(afterPickup, shootPose))
                .setConstantHeadingInterpolation(beforePickup.getHeading())
                .build();
    }

    /*
    State machine to determine the current path
    Each time the robot starts on a new path, it changes the path state and waits
    for the robot to reach the target pose, then starts the next path and repeats
    */
    public void autoPathUpdate(Follower follower) {
        switch (pathState) {
            case 0:
                activateFlywheel.schedule();
                if(spindexer.getNumArtifacts() > 0) {
                    shootArtifact.schedule();
                }
                else {
                    follower.followPath(goToPickup);
                    setPathState(1);
                }
                break;
            case 1:
                if(!follower.isBusy()) {
                    activateIntake.schedule();
                    follower.followPath(intakeBalls);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    activateFlywheel.schedule();
                    follower.followPath(goToSecondShoot);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    if(spindexer.getNumArtifacts() > 0) {
                        shootArtifact.schedule();
                    }
                    else {
                        setPathState(-1);
                    };
                }
                break;
        }
    }
}
