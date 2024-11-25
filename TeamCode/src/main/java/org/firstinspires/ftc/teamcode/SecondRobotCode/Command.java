//package org.firstinspires.ftc.teamcode.SecondRobotCode;
//
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//public class Command {
//    DcMotor arm;
//    CRServo intake;
//    CRServo upDown;
//
//    public Command(DcMotor arm, CRServo intake, CRServo upDown){
//        this.arm = arm;
//        this.intake = intake;
//        this.upDown = upDown;
//    }
//
//    public double angleConverter(double angle) {
//        angle = angle/360;
//        angle = angle*Constants.ArmConstants.GEARRATIO;
//        return angle;
//    }
//
//    IntakeSubsystem intakeSubsystem = new IntakeSubsystem(intake, upDown);
//
//
//    ArmSubsystem armSubsystem = new ArmSubsystem(arm);
//    //Arm Commands
//    public void armGoesUp() {
//       double newAngle = angleConverter(Constants.ArmConstants.ARMANGLEUP);
//
//        armSubsystem.pivotArmUP((int) newAngle);
//    }
//    public void armGoesDown() {
//        double newAngle = angleConverter(Constants.ArmConstants.ARMANGLEDOWN);
//        armSubsystem.pivotArmtoINTAKE((int) newAngle);
//    }
//
//    //Intake Commands
//    //yummy, consume the piece
//
//    //public void intake() throws InterruptedException {
//      //  armSubsystem.pivotArmtoINTAKE((int) Constants.ArmConstants.ARMANGLEDOWN);
//        //Thread.sleep(50);
//        //intakeSubsystem.intakeSpin(1);
//        //Thread.sleep(50);
//        //intakeSubsystem.intakeSpin(0);
//        //Thread.sleep(50);
//        //armSubsystem.pivotArmUP((int) Constants.ArmConstants.ARMANGLEUP);
//        //Thread.sleep(50);
//        //intakeSubsystem.intakeSpin(-1);
//        //Thread.sleep(50);
//        //intakeSubsystem.intakeSpin(0);
//    //}
//
//
////    public static void score() throws InterruptedException {
////        armExtender.armExtends();
////
////        ArmSubsystem.dumpyBucketDeposit();
////
////        ArmSubsystem.dumpyBucketBackUp();
////
////        armExtender.armUnextends();
////    }
//
//    //ArmExtender Commands
//
//
//}
