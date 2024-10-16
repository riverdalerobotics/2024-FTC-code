package org.firstinspires.ftc.teamcode.RobotCode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ArmSubsystem {

    static DcMotorEx armMotor;
    int armMotorPosition;

    public ArmSubsystem(DcMotorEx arm){
        this.armMotor = arm;
    }

    public void armUp() {

        armMotor.setPower(1);
    }
    public void armDown(){
        armMotor.setPower(-1);
    }

    public static void moveArm(double angle){
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        armMotor.setTargetPosition((int)angle);
    }
//
//    public void armReset(){
//        //reset arm position
//    }
//    public void armBasketPosition(){
//        //set arm position to score basket
//    }
//    public double getEncoders(){
//        return armMotor.getCurrentPosition();
//
//    }
//    public void resetEncoders(){
//
//    }
//

}
