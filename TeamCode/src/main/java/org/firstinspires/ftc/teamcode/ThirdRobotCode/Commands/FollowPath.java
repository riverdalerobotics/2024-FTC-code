package org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

public class FollowPath extends CommandBase {
    PathBuilder pathBuilder;
    PathChain path;
    Follower follower;
    Pose startPos;
    MultipleTelemetry telemetry;
    public FollowPath(PathBuilder pathBuilder, Follower follower, Pose startPos, MultipleTelemetry telemetry){
        this.telemetry = telemetry;
        this.follower = follower;
        this.startPos = startPos;
        this.pathBuilder = pathBuilder;
    }

    @Override
    public void initialize(){
        super.initialize();
        path = pathBuilder.build();
        follower.followPath(path);
        follower.setStartingPose(startPos);

    }

    @Override
    public void execute() {
        super.execute();
        follower.update();
        follower.telemetryDebug(telemetry);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(follower.getVelocity().getMagnitude())<1 && follower.atParametricEnd();
    }
}
