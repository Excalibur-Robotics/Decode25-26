package org.firstinspires.ftc.teamcode.V2.Autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.V2.Commands.ActivateFlywheel;
import org.firstinspires.ftc.teamcode.V2.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.V2.Commands.ShootArtifact;
import org.firstinspires.ftc.teamcode.V2.Commands.ShootColor;
import org.firstinspires.ftc.teamcode.V2.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.V2.Subsystems.SpindexerSubsystem;

/*
This contains the poses and paths for the close auto routine and the state
machine that determines the current path to follow.
So far this only contains the raw pathing of the robot, not shooting,
limelight, or indexing.

Routine:
The robot moves back to the shooting position, shoots the preloads, goes to
pick up the closest three artifacts, returns to the shooting position, and
shoots those three.
*/

public class CloseAuto {
    // declare poses
    public Pose startPose;
    public Pose shootPose;
    public Pose beforePickup;
    public Pose afterPickup;

    // declare paths
    public PathChain goToFirstShoot;
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
    public CloseAuto(Follower follower, IntakeSubsystem intake,
                     SpindexerSubsystem spindexer, OuttakeSubsystem outtake, boolean redTeam) {
        // sets poses based on if we are red or blue
        if(redTeam) {
            startPose = new Pose(122.0, 125.0, Math.toRadians(36));
            shootPose = new Pose(100.0, 107.0, Math.toRadians(44));
            beforePickup = new Pose(100.0, 84.0, 0);
            afterPickup = new Pose(126.0, 84.0, 0);
        }
        else {
            startPose = new Pose(22.0, 125.0, Math.toRadians(144));
            shootPose = new Pose(44.0, 107.0, Math.toRadians(136));
            beforePickup = new Pose(44.0, 84.0, Math.PI);
            afterPickup = new Pose(18.0, 84.0, Math.PI);
        }

        // set paths, same for both colors
        goToFirstShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        goToPickup = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, beforePickup))
                .setLinearHeadingInterpolation(shootPose.getHeading(), beforePickup.getHeading())
                .build();
        intakeBalls = follower.pathBuilder()
                .addPath(new BezierLine(beforePickup, afterPickup))
                .setConstantHeadingInterpolation(beforePickup.getHeading())
                .build();
        goToSecondShoot = follower.pathBuilder()
                .addPath(new BezierLine(afterPickup, shootPose))
                .setConstantHeadingInterpolation(beforePickup.getHeading())
                .build();

        // set path state to 0
        setPathState(0);

        this.intake = intake;
        this.spindexer = spindexer;
        this.outtake = outtake;

        activateFlywheel = new ActivateFlywheel(outtake, spindexer, 500);
        activateIntake = new IntakeCommand(intake, spindexer);
        shootPurple = new ShootColor(outtake, spindexer, "purple");
        shootGreen = new ShootColor(outtake, spindexer, "green");
        shootArtifact = new ShootArtifact(outtake, spindexer);
    }

    // method to change path state
    public void setPathState(int state) {
        pathState = state;
    }

    // method to get the path state - used for debugging
    public int getPathState() {
        return pathState;
    }

    /*
    State machine to determine the current path
    Each time the robot starts on a new path, it changes the path state and waits
    for the robot to reach the target pose, then starts the next path and repeats
    */
    public void autoPathUpdate(Follower follower) {
        switch (pathState) {
            case 0:
                follower.followPath(goToFirstShoot);
                activateFlywheel.schedule();
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    if(spindexer.getNumArtifacts() > 0) {
                        shootArtifact.schedule();
                    }
                    else {
                        follower.followPath(goToPickup);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    activateIntake.schedule();
                    follower.followPath(intakeBalls);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    activateFlywheel.schedule();
                    follower.followPath(goToSecondShoot);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    if(spindexer.getNumArtifacts() > 0) {
                        shootArtifact.schedule();
                    }
                    else {
                        setPathState(-1);
                    }
                }
        }
    }
}
