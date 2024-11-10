package org.firstinspires.ftc.teamcode.SecondRobotCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.HelperFunctions;

public class ArmSubsytem {
    DcMotor armMotor;
    public ArmSubsytem(DcMotor armMotor){
        this.armMotor = armMotor;
    }
    public double getPos() {
        return armMotor.getCurrentPosition() / 360 * Constants.ArmConstants.GEARRATIO;
    }


    public void pivotArmUP(int angle) {
        double ARM_TICK_PER_DEGREE = Constants.ArmConstants.ENCODERTICKPERROTATION * Constants.ArmConstants.GEARRATIO * Constants.ArmConstants.GEARREDUCTION * 1 / 360;
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition((int) ARM_TICK_PER_DEGREE * angle);
        armMotor.setPower(ARM_TICK_PER_DEGREE * angle);
    }

    public void pivotArmtoINTAKE(int angle) {
        double ARM_TICK_PER_DEGREE = Constants.ArmConstants.ENCODERTICKPERROTATION * Constants.ArmConstants.GEARRATIO * Constants.ArmConstants.GEARREDUCTION * 1 / 360;
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition((int) ARM_TICK_PER_DEGREE * angle);
        armMotor.setPower(ARM_TICK_PER_DEGREE * angle);

    }

    public void emergencyStop() {
        armMotor.setPower(0);
    }


    public void gotoPos(double angle) {
        double pos = getPos();
        double error = angle - pos;
        while (-Constants.ArmConstants.TOLERANCE <= error && error <= Constants.ArmConstants.TOLERANCE) {
            pos = getPos();
            error = angle - pos;
            double speed = HelperFunctions.piController(Constants.ArmConstants.Kp, Constants.ArmConstants.Ki, getPos(), angle);
            armMotor.setPower(speed);
        }
    }
}




