package org.firstinspires.ftc.teamcode.SecondRobotCode;

public class Command {

    //Arm Commands
    public static void armGoesUp() {
        ArmSubsytem.pivotArmUP((int) Constants.ArmConstants.ARMANGLEUP);
    }
    public static void armGoesDown() {
        ArmSubsytem.pivotArmtoINTAKE((int) Constants.ArmConstants.ARMANGLEDOWN);
    }

    //Intake Commands
    //yummy, consume the piece

    public static void intake() throws InterruptedException {
        ArmSubsytem.pivotArmtoINTAKE((int) Constants.ArmConstants.ARMANGLEDOWN);
        Thread.sleep(50);
        IntakeSubsystem.intakeSpin(1);
        Thread.sleep(50);
        IntakeSubsystem.intakeSpin(0);
        Thread.sleep(50);
        ArmSubsytem.pivotArmUP((int) Constants.ArmConstants.ARMANGLEUP);
        Thread.sleep(50);
        IntakeSubsystem.intakeSpin(-1);
        Thread.sleep(50);
        IntakeSubsystem.intakeSpin(0);
    }


    public static void score() throws InterruptedException {
        armExtender.armExtends();

        ArmSubsytem.dumpyBucketDeposit();

        ArmSubsytem.dumpyBucketBackUp();

        armExtender.armUnextends();
    }

    //ArmExtender Commands


}
