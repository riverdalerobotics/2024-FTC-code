package org.firstinspires.ftc.teamcode.ThirdRobotCode;


import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;

@TeleOp(name = "Vision Test Op Mode", group = "Linear OpMode")

    public class VisionTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .build();


        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "webcam 1"))
                .build();


        waitForStart();
        double poseX, poseY, poseZ, poseAX, poseAY, poseAZ;
        while(opModeIsActive()){
            for (AprilTagDetection detection : aprilTagProcessor.getDetections())  {

                Orientation rot = Orientation.getOrientation(detection.rawPose.R, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Original source data
                 poseX = detection.rawPose.x;
                 poseY = detection.rawPose.y;
                 poseZ = detection.rawPose.z;

                 poseAX = rot.firstAngle;
                 poseAY = rot.secondAngle;
                 poseAZ = rot.thirdAngle;
                telemetry.addData("xPos", poseX);
                telemetry.addData("yPos", poseY);
                telemetry.addData("zPos", poseZ);
                telemetry.update();
            }



        }
    }
}
