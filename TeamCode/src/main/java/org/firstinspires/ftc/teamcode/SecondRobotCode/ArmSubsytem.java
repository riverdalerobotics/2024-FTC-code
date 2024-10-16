package org.firstinspires.ftc.teamcode.SecondRobotCode;

import org.firstinspires.ftc.teamcode.HelperFunctions;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmSubsytem {
 DcMotor armMotor;

 public double getPos() {
     return armMotor.getCurrentPosition()/360*Constants.ArmConstants.GEARRATIO;
 }

 public void gotoPos(double desiredAngle) {
     double pos = getPos();
     double error = desiredAngle - pos;
     while(-Constants.ArmConstants.TOLERANCE<=error&& error<=Constants.ArmConstants.TOLERANCE){
         pos = getPos();
         error = desiredAngle - pos;
         double speed = HelperFunctions.piController(Constants.ArmConstants.Kp, Constants.ArmConstants.Ki, getPos(), desiredAngle);
         armMotor.setPower(speed);
     }
 }


}
