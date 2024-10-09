package org.firstinspires.ftc.teamcode.SecondRobotCode;

import org.firstinspires.ftc.teamcode.HelperFunctions;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmSubsytem {
 DcMotor armMotor;

 public double getPos() {
     return armMotor.getCurrentPosition()/360*Constants.ArmSubsystem.GEARRATIO;
 }

 public void gotoPos(double desiredAngle) {
     double pos = getPos();
     double error = desiredAngle - pos;
     while(-Constants.ArmSubsystem.TOLERANCE<=error&& error<=Constants.ArmSubsystem.TOLERANCE){
         pos = getPos();
         error = desiredAngle - pos;
         double speed = HelperFunctions.piController(Constants.ArmSubsystem.Kp, Constants.ArmSubsystem.Ki, getPos(), desiredAngle);
         armMotor.setPower(speed);
     }
 }


}
