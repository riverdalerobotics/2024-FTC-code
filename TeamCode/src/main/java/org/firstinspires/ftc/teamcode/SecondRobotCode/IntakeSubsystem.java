package org.firstinspires.ftc.teamcode.SecondRobotCode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;
public class IntakeSubsystem {

    Continou
    Servo upDownMotor;

    double servoGoesUp = Constants.intakeConstants.MAX_UP_POSITION;
    double servoGoesDown = Constants.intakeConstants.MIN_DOWN_POSITION;
    public void intakeSpin() {
        intakeMotor.setDirection(Servo.Direction.FORWARD);
    }
    public void intakeNonoSpin() {
        intakeMotor.setDirection();
    }
    public void Up() {
        upDownMotor.setPosition(servoGoesUp);
        upDownMotor.setDirection(Servo.Direction.FORWARD);

    }
    public void Down() {
        upDownMotor.setPosition(servoGoesDown);
        upDownMotor.setDirection(Servo.Direction.REVERSE);

    }
}
