package org.firstinspires.ftc.teamcode.ThirdRobotCode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/* cos(pivot angle) = limit/arm
*
*/
public class ArmSubsystem {
    DcMotorEx armPivotMotor;

    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    int armPivotMotorPosition;
    PIDFCoefficients values = new PIDFCoefficients(Kp, Ki, Kd, 0.7);
    public ArmSubsystem(DcMotorEx armPivot) {
        this.armPivotMotor = armPivot;
    }

    public void pivotArm(double armPivotInput, double power) {

        armPivotMotorPosition += (int) armPivotInput;
        armPivotMotor.setPower(power);
        armPivotMotor.setTargetPosition(armPivotMotorPosition);
    }

    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivitave = (error - lastError) / timer.seconds();

        timer.reset();

        return (error * Kp) + (derivitave * Kd) + (integralSum * Ki);

    }

    public void pivotArmUsingBuiltInStuffs(int angle){
        int rotations = angle*(int) Constants.ArmConstants.GEARRATIO/360;
        armPivotMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armPivotMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, values);
        armPivotMotor.setTargetPosition(rotations);
        armPivotMotor.setPower(1);
    }

}