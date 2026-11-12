package org.firstinspires.ftc.teamcode.team.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class gate {
    // -----------------------------------------------
    static final double LClosePosition = 0.45;
    static final double LOpenPosition = 0.28;

    static final double RClosePosition = 0.18;
    static final double ROpenPosition = 0.35;

    // -----------------------------------------------

    private final Servo gateL;
    private final Servo gateR;
    static boolean isGateOpen = false;

    public gate(Servo gateL, Servo gateR) {
        this.gateL = gateL;
        this.gateR = gateR;
    }
    public void openGate() {
        if (!isGateOpen){
            isGateOpen = true;
            gateL.setPosition(LOpenPosition);
            gateR.setPosition(ROpenPosition);
        }
    }
    public void closeGate() {
        if (isGateOpen) {
            isGateOpen = false;
            gateL.setPosition(LClosePosition);
            gateR.setPosition(RClosePosition);
        }
    }
    public void forceCloseGate() {
        gateL.setPosition(LClosePosition);
        gateR.setPosition(RClosePosition);
    }

}
