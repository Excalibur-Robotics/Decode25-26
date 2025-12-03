package org.firstinspires.ftc.teamcode.V1;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

/*
This contains the poses and paths for the far auto routine and the state
machine that determines the current path to follow.
So far this only contains the raw pathing of the robot, not shooting,
limelight, or indexing.

Routine:
The robot shoots the preloads from the start position, goes to pick up
the closest three artifacts, returns to the launch zone, and shoots those three.
*/

public class FarAuto {
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

    // constructor initializes poses and paths
    public FarAuto(Follower follower, boolean redTeam) {
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

        // set path state to 0
        setPathState(0);
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
                if(!follower.isBusy()) {
                    follower.followPath(goToPickup);
                    setPathState(1);
                }
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(intakeBalls);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(goToSecondShoot);
                    setPathState(-1);
                }
                break;
        }
    }
}
