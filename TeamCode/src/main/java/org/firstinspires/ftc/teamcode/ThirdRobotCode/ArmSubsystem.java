package org.firstinspires.ftc.teamcode.ThirdRobotCode;


import static org.firstinspires.ftc.teamcode.ThirdRobotCode.Constants.ArmSubsystem.CLAW_SERVO_START_POSITION;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/* cos(pivot angle) = limit/arm
*
*/
public class ArmSubsystem {
    DcMotor armPivotMotor;

    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    int armPivotMotorPosition;

    public ArmSubsystem(DcMotor armPivot) {
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

        double output = (error * Kp) + (derivitave * Kd) + (integralSum * Ki);
        return output;

    }
}