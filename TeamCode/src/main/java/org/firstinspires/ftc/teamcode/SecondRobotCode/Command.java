package org.firstinspires.ftc.teamcode.SecondRobotCode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.net.CacheRequest;

public class Command {
    DcMotor arm;
    CRServo intake;
    CRServo upDown;

    public Command(DcMotor arm, CRServo intake, CRServo upDown){
        this.arm = arm;
        this.intake = intake;
        this.upDown = upDown;
    }
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem(intake, upDown);
    ArmSubsytem armSubsytem = new ArmSubsytem(arm);
    //Arm Commands
    public void armGoesUp() {
        armSubsytem.pivotArmUP((int) Constants.ArmConstants.ARMANGLEUP);
    }
    public void armGoesDown() {
        armSubsytem.pivotArmtoINTAKE((int) Constants.ArmConstants.ARMANGLEDOWN);
    }

    //Intake Commands
    //yummy, consume the piece

    public void intake() throws InterruptedException {
        armSubsytem.pivotArmtoINTAKE((int) Constants.ArmConstants.ARMANGLEDOWN);
        Thread.sleep(50);
        intakeSubsystem.intakeSpin(1);
        Thread.sleep(50);
        intakeSubsystem.intakeSpin(0);
        Thread.sleep(50);
        armSubsytem.pivotArmUP((int) Constants.ArmConstants.ARMANGLEUP);
        Thread.sleep(50);
        intakeSubsystem.intakeSpin(-1);
        Thread.sleep(50);
        intakeSubsystem.intakeSpin(0);
    }


//    public static void score() throws InterruptedException {
//        armExtender.armExtends();
//
//        ArmSubsytem.dumpyBucketDeposit();
//
//        ArmSubsytem.dumpyBucketBackUp();
//
//        armExtender.armUnextends();
//    }

    //ArmExtender Commands


}
