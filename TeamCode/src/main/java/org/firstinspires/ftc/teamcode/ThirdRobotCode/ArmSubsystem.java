package org.firstinspires.ftc.teamcode.ThirdRobotCode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


public class ArmSubsystem {
    DcMotorEx armPivotMotor;


     double integralSum = 0;
     //double Kp = 0.0006; moved to constants
     //double Ki = 0;

     //double Kd = 0;
     //double Kf = 1;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    int armPivotMotorPosition;
    PIDFCoefficients values;

    /**
     * The subsystem of the arm
     * @param armPivot the arm motor, must be static and DcMotor<b>EX</b>
     */
    public ArmSubsystem(DcMotorEx armPivot) {
        values = new PIDFCoefficients(Constants.ArmConstants.kp, Constants.ArmConstants.ki, Constants.ArmConstants.kd, Constants.ArmConstants.kf);
        this.armPivotMotor = armPivot;
        armPivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        return 360*armPivotMotor.getCurrentPosition()/Constants.ArmConstants.GEAR_RATIO;
    }
    public void pivotArm(double angle, double speed, PIDFCoefficients pidfCoefficients){
        double rotations = angle*Constants.ArmConstants.GEAR_RATIO/360;
        armPivotMotor.setPower(speed);
        armPivotMotor.setTargetPosition((int)rotations);
        armPivotMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, pidfCoefficients);
        armPivotMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

    }

}