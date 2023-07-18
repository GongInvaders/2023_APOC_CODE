package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Subsystem;
import org.firstinspires.ftc.teamcode.teamUtil.ConfigNames;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConstants;

public class Guide extends Subsystem {

    RobotConfig r;

    public enum GuidePos {
        ACTIVE(RobotConstants.guideActive),
        STOW(RobotConstants.guideStow);

        private GuidePos(double position){
            this.position = position;
        }

        private final double position;

        public double getPosition() {
            return position;
        }
    }

    Servo guide;
    private double guidePosition;

    public Guide(RobotConfig r) {
        this.r = r;
    }
    public Guide(){
        r = RobotConfig.getInstance();
    }

    /**
     sets the wrist target position for the wrist servo.
     robotConstants.wristFront and robotConstants.wristBack have been set to 0 and 1.0 respectively
     */
    public void freeTargetPosition(double targetPos){
        guidePosition = targetPos;
    }

    public void presetTargetPosition(GuidePos guidePos){
        this.guidePosition = guidePos.getPosition();
    }


    @Override
    public void init() {
        guide = r.opMode.hardwareMap.get(Servo.class, ConfigNames.guide);
        presetTargetPosition(GuidePos.STOW);
        update();
    }

    @Override
    public void read() {

    }

    @Override
    public void update(){
        guide.setPosition(guidePosition);
    }

    @Override
    public void close() {

    }

}
