package org.firstinspires.ftc.teamcode.ThirdRobotCode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.IMU;

public class Commands {
    static class teleopCommands {
        public static void climbPtOne() {
            SlideSubsystem.goToPos(Constants.SlideConstants.CLIMB_UP);
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
            SlideSubsystem.goToPos(Constants.SlideConstants.CLIMB_DOWN);
        }

        public static void scoreBucket() {
            telemetry.addLine("Im Scoring");
            telemetry.update();
            ArmSubsystem.pivotArm(Constants.ArmConstants.BUCKET_ANGLE, Constants.ArmConstants.SCORE_SPEED);
            while (SlideSubsystem.rightSlideExtend.isBusy()) {
                if (SlideSubsystem.rightSlideExtend.getTargetPosition() > SlideSubsystem.slideLimit(ArmSubsystem.getPos())) {
                    SlideSubsystem.goToPos(SlideSubsystem.slideLimit(ArmSubsystem.getPos()) * Constants.SlideConstants.GEARDIAMETER);
                }
            }
            IntakeSubsystem.pivotIntake(Constants.IntakeConstants.SCORE_POSITION);
            IntakeSubsystem.spinIntake(Constants.IntakeConstants.SCORE_SPEED);
            telemetry.addLine("SCORE!!!!!");
            telemetry.update();
        }

        public static void intake(char teamColour) {
            ArmSubsystem.pivotArm(Constants.ArmConstants.INTAKE_ANGLE, Constants.ArmConstants.INTAKE_SPEED);
            SlideSubsystem.goToPosWithSpeed(SlideSubsystem.slideLimit(ArmSubsystem.getPos()), Constants.SlideConstants.INTAKE_SPEED);
            IntakeSubsystem.pivotIntake(Constants.IntakeConstants.INTAKE_POSITION);
            while ((IntakeSubsystem.getColour() == teamColour || IntakeSubsystem.getColour() == 'y')&&SlideSubsystem.leftSlideExtend.isBusy()) {
                IntakeSubsystem.spinIntake(Constants.IntakeConstants.INTAKE_SPEED);
            }
            IntakeSubsystem.spinIntake(0);
            SlideSubsystem.goToPos(SlideSubsystem.getSlidePos());
            ArmSubsystem.pivotArm(Constants.ArmConstants.BUCKET_ANGLE, Constants.ArmConstants.SCORE_SPEED);
            telemetry.addLine("I HAVE A PIECE");
        }
    }
}
