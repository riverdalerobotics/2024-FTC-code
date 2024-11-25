package org.firstinspires.ftc.teamcode.SecondRobotCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.HelperFunctions;

public class ArmSubsystem {
    DcMotor armMotor;

    public ArmSubsystem(DcMotor armMotor){
        this.armMotor = armMotor;
    }

    //getPosition methods
    public double getPosInDegrees() {
        return armMotor.getCurrentPosition() * 360 / Constants.ArmConstants.GEAR_RATIO;
    }
    public double getPositionInTicks(){
        return armMotor.getCurrentPosition();
    }

    //unit conversions used in setting angle
    public double degToTicks(double deg){
        return deg/360*Constants.ArmConstants.GEAR_RATIO;
    }

    public void setArmAngle(double angleInDegrees){
    armMotor.setPower(0.3);
    armMotor.setTargetPosition((int)degToTicks(angleInDegrees));
    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetEncoders(){
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void emergencyStop() {
        armMotor.setPower(0);
    }
}



//TODO: delete this


//
//public void pivotArmUP(int angle) {
//    double ARM_TICK_PER_DEGREE = Constants.ArmConstants.ENCODER_TICKS_PER_ROTATION * Constants.ArmConstants.GEAR_RATIO * Constants.ArmConstants.GEAR_REDUCTION * 1 / 360;
//    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    armMotor.setTargetPosition((int) ARM_TICK_PER_DEGREE * angle);
//    armMotor.setPower(ARM_TICK_PER_DEGREE * angle);
//}
//
//public void pivotArmtoINTAKE(int angle) {
//    double ARM_TICK_PER_DEGREE = Constants.ArmConstants.ENCODER_TICKS_PER_ROTATION * Constants.ArmConstants.GEAR_RATIO * Constants.ArmConstants.GEAR_REDUCTION * 1 / 360;
//    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    armMotor.setTargetPosition((int) ARM_TICK_PER_DEGREE * angle);
//    armMotor.setPower(ARM_TICK_PER_DEGREE * angle);
//
//}


//    public void moveArmTest(double angle){
//        armMotor.setPower(0.5);
//        armMotor.setTargetPosition((int)degToRotation(angle));
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }


//    public double getDegreesToArmTicks(double degrees) {
//        return degrees
//                * Constants.ArmConstants.ENCODERTICKPERROTATION // number of encoder ticks per rotation of the bare motor
//                * Constants.ArmConstants.GEARRATIO // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
//                * Constants.ArmConstants.GEARREDUCTION// This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
//                * 1 / 360.0; // we want ticks per degree, not per rotation
//    }




