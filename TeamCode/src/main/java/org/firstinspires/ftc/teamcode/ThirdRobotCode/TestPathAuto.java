//package org.firstinspires.ftc.teamcode.ThirdRobotCode;
//
//
//import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.Localizer;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers.OTOSLocalizer;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
//
//@Autonomous(name = "Pinkie Pie testpathing", group = "Linear OpMode")
//public class TestPathAuto extends LinearOpMode{
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        PathChain pathChain;
//        Follower follower;
//        Pose startPose = new Pose(9.746, 107.602, Math.PI);
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPose);
//        pathChain = follower.pathBuilder()        .addPath(
//                        // Line 1
//                        new BezierCurve(
//                                new Point(2.785, 109.790, Point.CARTESIAN),
//                                new Point(20.486, 121.326, Point.CARTESIAN),
//                                new Point(15.912, 132.066, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
//                .addPath(
//                        // Line 2
//                        new BezierLine(
//                                new Point(15.912, 132.066, Point.CARTESIAN),
//                                new Point(29.039, 132.464, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
//                .addPath(
//                        // Line 3
//                        new BezierLine(
//                                new Point(29.039, 132.464, Point.CARTESIAN),
//                                new Point(16.110, 132.464, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
//                .addPath(
//                        // Line 4
//                        new BezierCurve(
//                                new Point(16.110, 132.464, Point.CARTESIAN),
//                                new Point(6.365, 118.541, Point.CARTESIAN),
//                                new Point(27.448, 120.530, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
//                .addPath(
//                        // Line 5
//                        new BezierLine(
//                                new Point(27.448, 120.530, Point.CARTESIAN),
//                                new Point(16.309, 132.464, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
//                .build();
//        waitForStart();
//        follower.followPath(pathChain);
//        while(opModeIsActive()){
//
//            follower.update();
//        }
//    }
//}
