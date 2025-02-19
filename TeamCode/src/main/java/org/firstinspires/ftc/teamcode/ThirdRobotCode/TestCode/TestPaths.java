package org.firstinspires.ftc.teamcode.ThirdRobotCode.TestCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.ArmSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands.FollowPath;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands.MoveArm;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Constants;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.DefaultCommands.ChassisDefaultCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

/**
 * This is the Circle autonomous OpMode. It runs the robot in a PathChain that's actually not quite
 * a circle, but some Bezier curves that have control points set essentially in a square. However,
 * it turns enough to tune your centripetal force correction and some of your heading. Some lag in
 * heading is to be expected.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "TestPathinggAuto", group = "Autonomous Pathing Tuning")
public class TestPaths extends CommandOpMode {
    ArmSubsystem arm;
    PathBuilder builder;
    ChassisSubsystem chassis;
    private Follower follower;
    Pose startPos = new Pose(10, 110, 180);
    MultipleTelemetry telemetryA;
    FtcDashboard dashboard;
    DcMotorEx armMotor;

    @Override
    public void initialize() {



        dashboard = FtcDashboard.getInstance();
        builder = new PathBuilder();
        builder
                    .addPath(
                            // Line 1
                            new BezierCurve(
                                    new Point(10.000, 110.000, Point.CARTESIAN),
                                    new Point(19.691, 114.762, Point.CARTESIAN),
                                    new Point(13.923, 135.077, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));
        follower = new Follower(hardwareMap);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        armMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        arm = new ArmSubsystem(armMotor, telemetryA);
    }

    @Override
    public void run(){
        super.run();
        schedule(new FollowPath(builder, follower, startPos, telemetryA).andThen(new MoveArm(arm, telemetryA, Constants.ArmConstants.armPID, 90)));
        stop();
    }

}
