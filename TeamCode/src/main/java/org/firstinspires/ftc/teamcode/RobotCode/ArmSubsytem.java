package org.firstinspires.ftc.teamcode.RobotCode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ArmSubsytem {

    DcMotor armMotor;

public void armUp() {

    armMotor.setPower(1);
    }
public void armDown(){
    armMotor.setPower(-1);
}

public void armReset(){
    //reset arm position
}
public void armBasketPosition(){
    //set arm position to score basket
}
public double getEncoders(){
    return armMotor.getCurrentPosition();

}
public void resetEncoders(){

}


}
