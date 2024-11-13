package org.firstinspires.ftc.teamcode.ThirdRobotCode;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


public class ArmSubsystem {
    DcMotorEx armPivotMotor;

     double integralSum = 0;
     double Kp = 0.1;
     double Ki = 0;

     double Kd = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    int armPivotMotorPosition;
    PIDFCoefficients values = new PIDFCoefficients(Kp, Ki, Kd, 0.7);

    /**
     * The subsystem of the arm
     * @param armPivot the arm motor, must be static and DcMotor<b>EX</b>
     */
    public ArmSubsystem(DcMotorEx armPivot) {
        this.armPivotMotor = armPivot;
    }

    public void pivotArm(double armPivotInput, double power) {

        armPivotMotorPosition += (int) armPivotInput;
        armPivotMotor.setPower(power);
        armPivotMotor.setTargetPosition(armPivotMotorPosition);
    }
// Not necessary
//    public double PIDControl(double reference, double state) {
//        double error = reference - state;
//        integralSum += error * timer.seconds();
//        double derivitave = (error - lastError) / timer.seconds();
//
//        timer.reset();
//
//        return (error * Kp) + (derivitave * Kd) + (integralSum * Ki);
//
//    }
    public double getPos(){
        return armPivotMotor.getCurrentPosition()* Constants.ArmConstants.GEAR_RATIO /360;
    }
    public void pivotArmUsingBuiltInStuffs(double angle, double speed){
        double rotations = angle*(int) Constants.ArmConstants.GEAR_RATIO /360;
        armPivotMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armPivotMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, values);
        armPivotMotor.setTargetPosition((int)rotations);
        armPivotMotor.setPower(speed);
    }

}