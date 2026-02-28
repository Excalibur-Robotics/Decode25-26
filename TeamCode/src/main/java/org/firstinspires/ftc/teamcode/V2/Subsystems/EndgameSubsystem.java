package org.firstinspires.ftc.teamcode.V2.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Engagable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class EndgameSubsystem extends SubsystemBase {
    public Servo servo1;
    public Servo servo2;
    public static double angle = 0.25;

    public EndgameSubsystem(HardwareMap hwMap) {
        servo1 = hwMap.get(Servo.class, "endgame1");
        servo2 = hwMap.get(Servo.class, "endgame2");

        servo1.setDirection(Servo.Direction.REVERSE);
        servo2.setDirection(Servo.Direction.FORWARD);
    }

    public void activateEndgame() {
        servo1.setPosition(angle);
        servo2.setPosition(angle);
    }

    public void resetServos() {
        servo1.setPosition(0);
        servo2.setPosition(0);
    }

    public double getServoPos() {
        return servo1.getPosition();
    }
}
