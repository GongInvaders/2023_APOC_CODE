package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Subsystem;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;

public class BackBrace extends Subsystem {
    RobotConfig r;

    private DcMotor brace;
    private static double bracePos;

    private double previousTime;
    private double previousError;

    private double P;
    private double I; //needs to be given a value of 0 at the start
    private double D;
    private int targetPos = BracePositions.UP.encoderValue; //needs to be set whenever you need it to be changed
    private double braceHoldPosition = 0;
    private boolean braceHold = false;

    double bracePositioner;
    boolean limitIsPressed;
    BracePositions targetHeight = BracePositions.UP;


    public BackBrace(RobotConfig r) {
        this.r = r;
    }
    public BackBrace(){
        this(RobotConfig.getInstance());
    }


    @Override
    public void init() {
        brace = r.opMode.hardwareMap.get(DcMotor.class, "backbrace");
        previousTime = 0;
        previousError = 0;
        targetPos = 0;
        braceHoldPosition = 0;
        braceHold = false;
        brace.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brace.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brace.setDirection(DcMotorSimple.Direction.REVERSE);
        brace.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void read() {
        double bracePos = brace.getCurrentPosition();

    }
    public int getEncoder() {
        return brace.getCurrentPosition();
    }
    double cachedPower;
    
    @Override
    public void update(){
        double currentTime = RobotConfig.elapsedTime.time();
        
        brace.setTargetPosition(targetPos);
        brace.setPower(1);
        brace.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    @Override
    public void close() {

    }

    private void positionControl(double bracePosition, BracePositions poleHeight){
        switch (poleHeight){
            case UP:
                targetPos = BracePositions.UP.getEncoderValue();
                braceHoldPosition = BracePositions.UP.getEncoderValue();
                break;
            case DOWN:
                targetPos = BracePositions.DOWN.getEncoderValue();
                braceHoldPosition = BracePositions.DOWN.getEncoderValue();
                break;
            default:
                break;
        }

    }

    private BracePositions buttonAnalysis(boolean up, boolean down){
        if (up) return BracePositions.UP;
        if (down) return BracePositions.DOWN;
        return BracePositions.UP;
    }

    public void braceInputs(boolean up, boolean down){
        targetHeight = buttonAnalysis(up, down);
    }
    
    public void presetbracePosition(BracePositions bracePos){
        targetPos = bracePos.getEncoderValue();
        bracePositioner = 0;
    }
    
    public void limitSwitchInput(boolean limitIsPressed){
        this.limitIsPressed = limitIsPressed;
    }

    
    public enum BracePositions {
        UP(0),
        DOWN(100),
        FLIP(80),
        MID(-50);

        BracePositions(int encoderValue){
            this.encoderValue = encoderValue;
        }

        private final int encoderValue;

        public int getEncoderValue() {
            return encoderValue;
        }
    }
}
