package org.firstinspires.ftc.teamcode.ThirdRobotCode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


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
    ElapsedTime stopwatch = new ElapsedTime();

    public Commands(ArmSubsystem arm, SlideSubsystem slideSubsystem, ChassisSubsystem chassis, IntakeSubsystem intake, PIDFCoefficients pidf) {
        this.chassis = chassis;
        this.slideSubsystem = slideSubsystem;
        this.arm = arm;
        this.intakeSubsystem = intake;
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
    public void spit(){
        intakeSubsystem.spinIntake(0);
        intakeSubsystem.pivotIntake(0.6);
        char colour = intakeSubsystem.getColour();
        while(colour != 'w'){
            chassis.fieldOriented(0, 0, 0, 0);
            intakeSubsystem.spinIntake(-0.25);
            colour = intakeSubsystem.getColour();
        }
        intakeSubsystem.spinIntake(0);
    }
    public void goToZero(){
        intakeSubsystem.pivotIntake(0.5);
//        intakeSubsystem.spinIntake(0);
        slideSubsystem.goToPosWithSpeed(0.5, 1);
        while(slideSubsystem.getSlidePos()>=20){
            chassis.fieldOriented(0, 0, 0, 0);
        }
        intakeSubsystem.pivotIntake(0.6);
        arm.pivotArm(0, 1, pidf);
    }
    public void scoreBucket() {
        intakeSubsystem.pivotIntake(0.5);
        slideSubsystem.goToPosWithSpeed(0, 1);
        arm.pivotArm(95, 1, pidf);
        while(arm.getPos()<45){
            chassis.fieldOriented(0, 0, 0, 0);
        }
        slideSubsystem.goToPosWithSpeed(90, 1);

    }

    public void intake(char teamColour, Gamepad gamepad2, Gamepad gamepad1) throws InterruptedException {
        slideSubsystem.goToPosWithSpeed(0, 1);
        SparkFunOTOS.Pose2D robotPos = chassis.myAwtos.getPosition();
        arm.pivotArm(0, 1, pidf);
        while(arm.getPos()>45){
            chassis.fieldOriented(robotPos.h, -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
        slideSubsystem.goToPosWithSpeed(50, 0.5);
        while(slideSubsystem.getSlidePos()<20){ //changed from 10
            chassis.fieldOriented(robotPos.h, -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
        intakeSubsystem.pivotIntake(0.16);
        while(intakeSubsystem.getColour() != teamColour && intakeSubsystem.getColour() != 'y'&& !(gamepad2.right_trigger>=0.3)){
            chassis.fieldOriented(robotPos.h, -gamepad1.left_stick_y*0.3, gamepad1.left_stick_x*0.3, gamepad1.right_stick_x*0.3);
            intakeSubsystem.spinIntake(-0.15);
            arm.pivotArm(0, 0);
        }
        intakeSubsystem.spinIntake(0);
        intakeSubsystem.pivotIntake(0.6);
        goToZero();
    }
    public void goToScore(double xPos, double yPos, double heading, double rotKp){
            chassis.goToPosition(xPos, yPos, heading, 0.03, rotKp, 45, -20, 135);
    }
    public void climb(){
        intakeSubsystem.pivotIntake(0.5);
        arm.pivotArm(90, 0.75, pidf);
        slideSubsystem.goToPosWithSpeed(40, 1); //increase later?
        while(slideSubsystem.leftSlideExtend.isBusy()){}
        arm.pivotArm(95, 0.5, pidf);
        slideSubsystem.goToPosWithSpeed(10, 0.5);
        while(slideSubsystem.leftSlideExtend.isBusy()){
        }
        arm.pivotArm(45, 1);
    }

    public void autoScore(double xPos, double yPos, double heading, double rotKp, boolean canScore){
        goToScore(xPos, yPos, heading, rotKp);
        if(canScore) {
            scoreBucket();
            while (slideSubsystem.leftSlideExtend.isBusy()) {
            }
            while(intakeSubsystem.getColour()!='w'){
                intakeSubsystem.spinIntake(-1);
            }
            intakeSubsystem.spinIntake(0);
        }

    }
}