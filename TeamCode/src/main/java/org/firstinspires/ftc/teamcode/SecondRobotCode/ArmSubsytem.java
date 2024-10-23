package org.firstinspires.ftc.teamcode.SecondRobotCode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.teamcode.HelperFunctions;
import org.firstinspires.ftc.teamcode.SecondRobotCode.Constants.ArmExtender;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
public class ArmSubsytem {
 static DcMotor armMotor;
 static CRServo dumpyMotor;

    public double getPos() {
         return armMotor.getCurrentPosition()/360*Constants.ArmConstants.GEARRATIO;
     }


         public static void pivotArmUP(int angle) {
             double ARM_TICK_PER_DEGREE = Constants.ArmConstants.ENCODERTICKPERROTATION*Constants.ArmConstants.GEARRATIO*Constants.ArmConstants.GEARREDUCTION*1/360;
             armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             armMotor.setTargetPosition((int) ARM_TICK_PER_DEGREE*angle);
             armMotor.setPower(ARM_TICK_PER_DEGREE*angle);
         }

         public static void pivotArmtoINTAKE(int angle) {
                 double ARM_TICK_PER_DEGREE = Constants.ArmConstants.ENCODERTICKPERROTATION*Constants.ArmConstants.GEARRATIO*Constants.ArmConstants.GEARREDUCTION*1/360;
                 armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 armMotor.setTargetPosition((int) ARM_TICK_PER_DEGREE*angle);
                 armMotor.setPower(ARM_TICK_PER_DEGREE*angle);

             }


          public static void dumpyBucketDeposit() {
            dumpyMotor.setPower(0.5);
          }
          public static void dumpyBucketBackUp() {
            dumpyMotor.setPower(-0.5);
          }


 //public void gotoPos(double angle) {
  //   double pos = getPos();
 //    double error = angle - pos;
 //    while(-Constants.ArmSubsystem.TOLERANCE<=error&& error<=Constants.ArmSubsystem.TOLERANCE){
 //        pos = getPos();
 //        error = angle - pos;
 //        double speed = HelperFunctions.piController(Constants.ArmSubsystem.Kp, Constants.ArmSubsystem.Ki, getPos(), angle);
 //        armMotor.setPower(speed);
     }





