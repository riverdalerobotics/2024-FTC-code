package org.firstinspires.ftc.teamcode.ThirdRobotCode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
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
        char colour = intakeSubsystem.getColour();
        while(colour == 'b'){
            intakeSubsystem.spinIntake(1);
            colour = intakeSubsystem.getColour();
        }
        intakeSubsystem.spinIntake(0);
    }
    public void goToZero(){
        intakeSubsystem.pivotIntake(0);
//        intakeSubsystem.spinIntake(0);
        slideSubsystem.goToPosWithSpeed(0, 1);
        while(slideSubsystem.rightSlideExtend.isBusy()){}
        arm.pivotArm(0, 1, pidf);
    }
    public void scoreBucket() {
        intakeSubsystem.pivotIntake(0);
        slideSubsystem.goToPosWithSpeed(0, 1);
        arm.pivotArm(95, 1, pidf);
        while(arm.getPos()<45){}
        intakeSubsystem.pivotIntake(-0.5);
        slideSubsystem.goToPosWithSpeed(90, 1);
    }

    public void intake(char teamColour) throws InterruptedException {
        slideSubsystem.goToPosWithSpeed(0, 1);
        arm.pivotArm(0, 1, pidf);
        while(arm.getPos()>45){}
        slideSubsystem.goToPosWithSpeed(50, 0.3);
        while(slideSubsystem.getSlidePos()<6){
        }
        intakeSubsystem.pivotIntake(0.5);
        while(intakeSubsystem.getColour() != teamColour && intakeSubsystem.getColour() != 'y'){
            intakeSubsystem.spinIntake(-0.25);
        }
        intakeSubsystem.spinIntake(0);
        goToZero();
    }
    public void goToScore(double xPos, double yPos, double heading, double rotKp){
            chassis.goToPosition(xPos, yPos, heading, 0.03, rotKp, 45, -20, 135);
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