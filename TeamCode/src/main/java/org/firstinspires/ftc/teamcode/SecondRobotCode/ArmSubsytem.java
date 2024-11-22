package org.firstinspires.ftc.teamcode.SecondRobotCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.HelperFunctions;

//TODO: WHO EVER SEES THIS NEXT please chnage the file to "ArmSybsystem" - Sana :)
public class ArmSubsytem {
    static DcMotor armMotor;

    public ArmSubsytem(DcMotor armMotor){
      this.armMotor = armMotor;
    }

    public double getPos() {
        return armMotor.getCurrentPosition() * 360 / Constants.ArmConstants.GEARRATIO;
    }



    public void moveArm(double angle){
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        armMotor.setTargetPosition((int)angle);
    }

    public static void pivotArmUP(int angle) {
        double ARM_TICK_PER_DEGREE = Constants.ArmConstants.ENCODERTICKPERROTATION * Constants.ArmConstants.GEARRATIO * Constants.ArmConstants.GEARREDUCTION * 1 / 360;
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition((int) ARM_TICK_PER_DEGREE * angle);
        armMotor.setPower(ARM_TICK_PER_DEGREE * angle);
    }

    public static void pivotArmtoINTAKE(int angle) {
        double ARM_TICK_PER_DEGREE = Constants.ArmConstants.ENCODERTICKPERROTATION * Constants.ArmConstants.GEARRATIO * Constants.ArmConstants.GEARREDUCTION * 1 / 360;
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition((int) ARM_TICK_PER_DEGREE * angle);
        armMotor.setPower(ARM_TICK_PER_DEGREE * angle);

    }
//    public double degToRotation(double deg){
//
//        return deg/360*1425*5;
//    }

    public void moveArmTest(double angle){
        armMotor.setPower(0.5);
        armMotor.setTargetPosition((int)degToRotation(angle));
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    public double degToRotation(double deg){

        return deg/360*1425*5;
    }

    public double getDegreesToArmTicks(double degrees) {
        return degrees
                * Constants.ArmConstants.ENCODERTICKPERROTATION // number of encoder ticks per rotation of the bare motor
                * Constants.ArmConstants.GEARRATIO // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                * Constants.ArmConstants.GEARREDUCTION// This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                * 1 / 360.0; // we want ticks per degree, not per rotation
    }


//    public double getPositionInTicks(){
//        return armMotor.getCurrentPosition();
//    }

    public void resetEncoders(){
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }




    public void emergencyStop() {
        armMotor.setPower(0);
    }




}




