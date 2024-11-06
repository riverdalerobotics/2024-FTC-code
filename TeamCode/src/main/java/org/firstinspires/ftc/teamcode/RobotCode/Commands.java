package org.firstinspires.ftc.teamcode.RobotCode;

public class Commands {
    static class teleopCommands {
        public static void scoreBucket() {
            //code to score bucket
            //TODO: Instead of 90 make the desired angle a constant in Constants.java-Nicolas
            ArmSubsystem.moveArm(Constants.ArmConstants.BUCKET_ANGLE);
            IntakeSubsystem.intakePowerOn(Constants.IntakeConstants.RELEASE_SAMPLE);
        }

        public static void scoreBar() {

            ArmSubsystem.moveArm(Constants.ArmConstants.BAR_ANGLE);
            IntakeSubsystem.intakePowerOn(Constants.IntakeConstants.RELEASE_SAMPLE);

        }


        public static void intake() {
            ArmSubsystem.moveArm(Constants.ArmConstants.INTAKE_ANGLE);
            IntakeSubsystem.intakePowerOn(Constants.IntakeConstants.INTAKE_SAMPLE);
        }


    }
}

