package org.firstinspires.ftc.teamcode.ThirdRobotCode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;


class moveSlides implements  Runnable{
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
    public void run(){
        slides.newGoToPos(distance);
        while(slides.leftSlideExtend.isBusy()) {
            if (slides.pidfController.getTargetPosition() > slides.slideLimit(arm.getPos())) {
                slides.newGoToPos(slides.slideLimit(arm.getPos()));
            }
            else if(slides.pidfController.getTargetPosition() > Constants.SlideConstants.ABSOLUTE_LIMIT){
                slides.newGoToPos(Constants.SlideConstants.ABSOLUTE_LIMIT);
            }
            else{
                slides.newGoToPos(distance);
            }
        }
    }
}
public class Commands {
    ArmSubsystem arm;
    SlideSubsystem slideSubsystem;
    ChassisSubsystem chassis;
    IntakeSubsystem intakeSubsystem;
    public Commands(ArmSubsystem arm, SlideSubsystem slideSubsystem, ChassisSubsystem chassis, IntakeSubsystem intake){
        this.chassis = chassis;
        this.slideSubsystem = slideSubsystem;
        this.arm = arm;
        this.intakeSubsystem = intake;
    }

    class teleopCommands{
        moveSlides slides;
        Thread slideThread = new Thread(slides);

        public void moveSlides(double target){
            slides = new moveSlides(target, arm, slideSubsystem);
            slideThread.start();
        }

        public  void climbPtOne() {
            moveSlides(Constants.SlideConstants.CLIMB_UP);
        }

        public void climbPtTwo(IMU gyro) {
            boolean isClimbing = false;
            arm.pivotArm(Constants.ArmConstants.CLIMB_UP_ANGLE, Constants.ArmConstants.CLIMB_SPEED);
            double pitch = chassis.pitch();

            double startPitch = pitch;
            while (pitch < 45 + startPitch) {
                pitch = chassis.pitch();
            }
            arm.pivotArm(Constants.ArmConstants.CLIMB_DOWN_ANGLE, Constants.ArmConstants.CLIMB_SPEED);
            telemetry.addData("I'm waiting to climb", isClimbing);
            while (arm.getPos() < Constants.ArmConstants.START_CLIMB) {
                isClimbing = true;
                telemetry.update();
            }
            isClimbing = false;
            telemetry.addLine("IM CLIMBING");
            telemetry.update();
            moveSlides(Constants.SlideConstants.CLIMB_DOWN);
            while(slideThread.isAlive()){}
        }

        public void scoreBucket() {
            telemetry.addLine("Im Scoring");
            telemetry.update();
            arm.pivotArmUsingBuiltInStuffs(Constants.ArmConstants.BUCKET_ANGLE, Constants.ArmConstants.SCORE_SPEED);
            moveSlides(Constants.SlideConstants.SCORE_BUCKET);
            while(slideThread.isAlive()){}
            intakeSubsystem.pivotIntake(Constants.IntakeConstants.SCORE_POSITION);
            intakeSubsystem.spinIntake(Constants.IntakeConstants.SCORE_SPEED);
            telemetry.addLine("SCORE!!!!!");
            telemetry.update();
            moveSlides(Constants.SlideConstants.INTAKE_POSITION);
            arm.pivotArmUsingBuiltInStuffs(Constants.ArmConstants.INTAKE_ANGLE, Constants.ArmConstants.INTAKE_SPEED);
        }

        public void intake(char teamColour) throws InterruptedException {
            arm.pivotArmUsingBuiltInStuffs(Constants.ArmConstants.INTAKE_ANGLE, Constants.ArmConstants.INTAKE_SPEED);
            slideSubsystem.newGoToPos(slideSubsystem.slideLimit(arm.getPos()));
            slideSubsystem.pidfController.setOutputBounds(Constants.SlideConstants.INTAKE_MIN, Constants.SlideConstants.INTAKE_MAX);
            intakeSubsystem.pivotIntake(Constants.IntakeConstants.INTAKE_POSITION);
            while ((intakeSubsystem.getColour() != teamColour || intakeSubsystem.getColour() != 'y')&&slideSubsystem.leftSlideExtend.isBusy()) {
                intakeSubsystem.spinIntake(Constants.IntakeConstants.INTAKE_SPEED);
                slideSubsystem.newGoToPos(slideSubsystem.slideLimit(arm.getPos()));
                //This avoids noise wait time should be relativity small
                if(intakeSubsystem.getColour() == teamColour || intakeSubsystem.getColour() == 'y'){
                    Thread.sleep(Constants.IntakeConstants.WAIT_TIME);
                }
            }
            intakeSubsystem.spinIntake(0);
            moveSlides(Constants.SlideConstants.INTAKE_POSITION);
            arm.pivotArmUsingBuiltInStuffs(Constants.ArmConstants.BUCKET_ANGLE, Constants.ArmConstants.SCORE_SPEED);
            telemetry.addLine("I HAVE A PIECE");
            slideSubsystem.pidfController.setOutputBounds(1,1);
        }



    }
    class Autos{
        public final double[] SCORE_POS = Constants.AutoConstants.SCORE_POS;
        Vector2d scorePos = new Vector2d(SCORE_POS[0], SCORE_POS[1]);
        Pose2d score = new Pose2d(scorePos, SCORE_POS[2]);
        public void autoGoToScore(){
            chassis.trajectoryBuilder(chassis.getBotPos())
                    .splineTo(scorePos, score.getHeading())
                    .build();
        }
    }
}
