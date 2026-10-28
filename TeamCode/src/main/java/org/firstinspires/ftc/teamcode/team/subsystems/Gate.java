package org.firstinspires.ftc.teamcode.team.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class Gate {
    // -----------------------------------------------
    static final double ClosePosition = 0;
    static final double OpenPosition = 0.5;

    // -----------------------------------------------

    private final Servo gate;
    static boolean isGateOpen = false;

    public Gate(Servo gate) {
        this.gate = gate;
    }
    public void openGate() {
        if (!isGateOpen){
            isGateOpen = true;
            gate.setPosition(OpenPosition);
        }
    }
    public void closeGate() {
        if (isGateOpen) {
            isGateOpen = false;
            gate.setPosition(ClosePosition);
        }
    }
    public void forceCloseGate() {
        gate.setPosition(ClosePosition);
    }

}
