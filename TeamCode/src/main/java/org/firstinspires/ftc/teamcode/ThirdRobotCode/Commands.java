package org.firstinspires.ftc.teamcode.ThirdRobotCode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.IMU;


class moveSlides implements  Runnable{
    double distance;
    /**
     * <p>A tread that allows the slides to go to a position using pid while other commands or functions
     * are running</p>
     * @param distance the desired distance for the slides
     */
    public moveSlides(double distance){
        this.distance = distance;
    }
    public void run(){
        SlideSubsystem.newGoToPos(distance);
        while(SlideSubsystem.leftSlideExtend.isBusy()) {
            if (SlideSubsystem.pidfController.getTargetPosition() > SlideSubsystem.slideLimit(ArmSubsystem.getPos())) {
                SlideSubsystem.newGoToPos(SlideSubsystem.slideLimit(ArmSubsystem.getPos()));
            }
            else if(SlideSubsystem.pidfController.getTargetPosition() > Constants.SlideConstants.ABSOLUTE_LIMIT){
                SlideSubsystem.newGoToPos(Constants.SlideConstants.ABSOLUTE_LIMIT);
            }
            else{
                SlideSubsystem.newGoToPos(distance);
            }
        }
    }
}
public class Commands {

    static class teleopCommands{
        static moveSlides slides;
        static Thread slideThread = new Thread(slides);

        public static void moveSlides(double target){
            slides = new moveSlides(target);
            slideThread.start();
        }

        public static void climbPtOne() {
            moveSlides(Constants.SlideConstants.CLIMB_UP);
        }

        public static void climbPtTwo(IMU gyro) {
            boolean isClimbing = false;
            ArmSubsystem.pivotArm(Constants.ArmConstants.CLIMB_UP_ANGLE, Constants.ArmConstants.CLIMB_SPEED);
            double pitch = ChassisSubsystem.pitch(gyro);

            double startPitch = pitch;
            while (pitch < 45 + startPitch) {
                pitch = ChassisSubsystem.pitch(gyro);
            }
            ArmSubsystem.pivotArm(Constants.ArmConstants.CLIMB_DOWN_ANGLE, Constants.ArmConstants.CLIMB_SPEED);
            telemetry.addData("I'm waiting to climb", isClimbing);
            while (ArmSubsystem.getPos() < Constants.ArmConstants.START_CLIMB) {
                isClimbing = true;
                telemetry.update();
            }
            isClimbing = false;
            telemetry.addLine("IM CLIMBING");
            telemetry.update();
            moveSlides(Constants.SlideConstants.CLIMB_DOWN);
            while(slideThread.isAlive()){}
        }

        public static void scoreBucket() {
            telemetry.addLine("Im Scoring");
            telemetry.update();
            ArmSubsystem.pivotArmUsingBuiltInStuffs(Constants.ArmConstants.BUCKET_ANGLE, Constants.ArmConstants.SCORE_SPEED);
            moveSlides(Constants.SlideConstants.SCORE_BUCKET);
            while(slideThread.isAlive()){}
            IntakeSubsystem.pivotIntake(Constants.IntakeConstants.SCORE_POSITION);
            IntakeSubsystem.spinIntake(Constants.IntakeConstants.SCORE_SPEED);
            telemetry.addLine("SCORE!!!!!");
            telemetry.update();
            moveSlides(Constants.SlideConstants.INTAKE_POSITION);
            ArmSubsystem.pivotArmUsingBuiltInStuffs(Constants.ArmConstants.INTAKE_ANGLE, Constants.ArmConstants.INTAKE_SPEED);
        }

        public static void intake(char teamColour) throws InterruptedException {
            ArmSubsystem.pivotArmUsingBuiltInStuffs(Constants.ArmConstants.INTAKE_ANGLE, Constants.ArmConstants.INTAKE_SPEED);
            SlideSubsystem.newGoToPos(SlideSubsystem.slideLimit(ArmSubsystem.getPos()));
            SlideSubsystem.pidfController.setOutputBounds(Constants.SlideConstants.INTAKE_MIN, Constants.SlideConstants.INTAKE_MAX);
            IntakeSubsystem.pivotIntake(Constants.IntakeConstants.INTAKE_POSITION);
            while ((IntakeSubsystem.getColour() != teamColour || IntakeSubsystem.getColour() != 'y')&&SlideSubsystem.leftSlideExtend.isBusy()) {
                IntakeSubsystem.spinIntake(Constants.IntakeConstants.INTAKE_SPEED);
                SlideSubsystem.newGoToPos(SlideSubsystem.slideLimit(ArmSubsystem.getPos()));
                //This avoids noise wait time should be relativity small
                if(IntakeSubsystem.getColour() == teamColour || IntakeSubsystem.getColour() == 'y'){
                    Thread.sleep(Constants.IntakeConstants.WAIT_TIME);
                }
            }
            IntakeSubsystem.spinIntake(0);
            moveSlides(Constants.SlideConstants.INTAKE_POSITION);
            ArmSubsystem.pivotArmUsingBuiltInStuffs(Constants.ArmConstants.BUCKET_ANGLE, Constants.ArmConstants.SCORE_SPEED);
            telemetry.addLine("I HAVE A PIECE");
            SlideSubsystem.pidfController.setOutputBounds(1,1);
        }



    }
    static class Autos{
        public static final double[] SCORE_POS = Constants.AutoConstants.SCORE_POS;
        static Vector2d scorePos = new Vector2d(SCORE_POS[0], SCORE_POS[1]);
        static Pose2d score = new Pose2d(scorePos, SCORE_POS[2]);
        public static void autoGoToScore(){
            ChassisSubsystem.trajectoryBuilder(ChassisSubsystem.getBotPos())
                    .splineTo(scorePos, score.getHeading())
                    .build();
        }
    }
}
