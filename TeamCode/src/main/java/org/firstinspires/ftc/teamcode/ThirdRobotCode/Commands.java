package org.firstinspires.ftc.teamcode.ThirdRobotCode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


class moveSlides extends Thread{
    double distance;
    ArmSubsystem arm;
    SlideSubsystem slides;
    /**
     * <p>A tread that allows the slides to go to a position using pid while other commands or functions
     * are running</p>
     * @param distance the desired distance for the slides
     */
    public moveSlides(double distance, ArmSubsystem armSubsystem, SlideSubsystem slideSubsystem){
        this.distance = distance;
        this.arm = armSubsystem;
        this.slides = slideSubsystem;
    }
    public void run() {
        slides.rightSlideExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.leftSlideExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.newGoToPos(50);
        while(slides.pidfController.getLastError()>=0.5||slides.pidfController.getLastError()<=-0.5) {
            slides.newGoToPos(50);
            telemetry.addData("Slides power", slides.leftSlideExtend.getPower());
            telemetry.addData("Slides goal", slides.pidfController.getLastError());
            telemetry.addData("Slides output", slides.pidfController.getTargetPosition());
        }
        slides.leftSlideExtend.setTargetPosition(slides.leftSlideExtend.getCurrentPosition());
        slides.rightSlideExtend.setTargetPosition(slides.rightSlideExtend.getCurrentPosition());
        slides.moveSlide(0.2);
        slides.rightSlideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.leftSlideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }


public class Commands {
    ArmSubsystem arm;
    SlideSubsystem slideSubsystem;
    ChassisSubsystem chassis;
    IntakeSubsystem intakeSubsystem;
    PIDFCoefficients pidf;

    public Commands(ArmSubsystem arm, SlideSubsystem slideSubsystem, ChassisSubsystem chassis, PIDFCoefficients pidf) {
        this.chassis = chassis;
        this.slideSubsystem = slideSubsystem;
        this.arm = arm;
        //this.intakeSubsystem = intake;
        this.pidf = pidf;
    }

    moveSlides slides;

    Thread slideThread;
    public void moveSlides(double target) {
        slides = new moveSlides(target, arm, slideSubsystem);
        slides.start();
    }

    public void climbPtOne() {
        moveSlides(Constants.SlideConstants.CLIMB_UP);
    }

//        public void climbPtTwo(IMU gyro) {
//            boolean isClimbing = false;
//            arm.pivotArm(Constants.ArmConstants.CLIMB_UP_ANGLE, Constants.ArmConstants.CLIMB_SPEED);
//            double pitch = chassis.pitch();
//
//            double startPitch = pitch;
//            while (pitch < 45 + startPitch) {
//                pitch = chassis.pitch();
//            }
//            arm.pivotArm(Constants.ArmConstants.CLIMB_DOWN_ANGLE, Constants.ArmConstants.CLIMB_SPEED);
//            telemetry.addData("I'm waiting to climb", isClimbing);
//            while (arm.getPos() < Constants.ArmConstants.START_CLIMB) {
//                isClimbing = true;
//                telemetry.update();
//            }
//            isClimbing = false;
//            telemetry.addLine("IM CLIMBING");
//            telemetry.update();
//            moveSlides(Constants.SlideConstants.CLIMB_DOWN);
//            while(slideThread.isAlive()){}
//        }

    public void scoreBucket() {
        slideSubsystem.goToPosWithSpeed(0, 1);
        arm.pivotArmUsingBuiltInStuffs(95, 1, pidf);
        while(arm.getPos()<45){}
        slideSubsystem.goToPosWithSpeed(90, 1);
    }

    public void intake(char teamColour) throws InterruptedException {
        slideSubsystem.goToPosWithSpeed(0, 1);
        arm.pivotArmUsingBuiltInStuffs(0, 1, pidf);
        while(arm.getPos()>45){}
        slideSubsystem.goToPosWithSpeed(50, 1);

    }}