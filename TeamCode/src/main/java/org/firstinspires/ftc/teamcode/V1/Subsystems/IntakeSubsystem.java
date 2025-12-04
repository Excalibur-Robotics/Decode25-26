package org.firstinspires.ftc.teamcode.V1.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {
    public DcMotor intakeMotor;

    public IntakeSubsystem(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void activateIntake() {
        intakeMotor.setPower(1);
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
    }
}
