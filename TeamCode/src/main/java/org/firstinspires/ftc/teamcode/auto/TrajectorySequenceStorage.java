package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.drive.SampleMecanumDrive.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.drive.SampleMecanumDrive.getVelocityConstraint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.BackBrace;
import org.firstinspires.ftc.teamcode.subsystems.FrontBrace;
import org.firstinspires.ftc.teamcode.subsystems.Guide;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.webcam.Webcam;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

public class TrajectorySequenceStorage {
	
	private TrajectorySequence[] trajectorySequences;
	private ArrayList<TrajectorySequence> trajectorySequenceArrayList;
	private int sequenceIndex;
	private RobotConfig r;
	private SampleMecanumDrive drive;
	private Lift lift;
	private Intake intake;
	private Arm arm;
	private Wrist wrist;
	private Guide guide;
	private BackBrace backBrace;
	private FrontBrace frontBrace;
	
	public static final Pose2d startPoseRight = new Pose2d (34, -65, Math.toRadians(90));

	public static final Pose2d startPoseLeft = new Pose2d (-34, -64.5, Math.toRadians(90));

	private TrajectorySequence rightRiley(){
		drive.setPoseEstimate(startPoseRight);
		return drive.trajectorySequenceBuilder(startPoseRight)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
					r.setPickup(true);
					guide.presetTargetPosition(Guide.GuidePos.STOW);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HOVER);
				})
				.splineTo(new Vector2d(34, -12), Math.toRadians(90))//drive forward //was 33.5 for x
				.UNSTABLE_addTemporalMarkerOffset(-2, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
				})
				.UNSTABLE_addTemporalMarkerOffset(-1, () -> {
					guide.presetTargetPosition(Guide.GuidePos.ACTIVE);
				})
				.UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM_DROP);
					arm.presetTargetPosition(Arm.ArmPos.FRONT);
				})
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
					guide.presetTargetPosition(Guide.GuidePos.STOW);
				})
				.turn(Math.toRadians(90))
				.build();
	}


	private TrajectorySequence rightStartMediumPark(){
		drive.setPoseEstimate(startPoseRight);
		return drive.trajectorySequenceBuilder(startPoseRight)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
					r.setPickup(true);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HOVER);
				})
				.strafeTo(new Vector2d(34, -36.5))//drive forward //was 33.5 for x
				.strafeTo(new Vector2d(13, -36.5))//turn //TODO change
				.UNSTABLE_addTemporalMarkerOffset(-2, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
				})
				.waitSeconds(0)
				.UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
					arm.presetTargetPosition(Arm.ArmPos.FRONT);
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM_DROP);
				})
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.build();
	}

	private TrajectorySequence leftStartMediumPark(){
		drive.setPoseEstimate(startPoseLeft);
		return drive.trajectorySequenceBuilder(startPoseLeft)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
					r.setPickup(true);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HOVER);
				})
				.strafeTo(new Vector2d(-34.5, -36.5))//drive forward //was 33.5 for x
				.strafeTo(new Vector2d(-22.0, -36.5))//turn //TODO change
				.UNSTABLE_addTemporalMarkerOffset(-2, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
				})
				.waitSeconds(0)
				.UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM_DROP);
				})
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.build();
	}
	
	private TrajectorySequence rightStartMediumPole(){
		drive.setPoseEstimate(startPoseRight);
		return drive.trajectorySequenceBuilder(startPoseRight)
				.setConstraints(getVelocityConstraint(35, Math.toRadians(60),12.76), getAccelerationConstraint(35))
				.UNSTABLE_addDisplacementMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);

				})
				.splineTo(new Vector2d(36, -24),  Math.toRadians(90))//drive forward //was 33.5 for x
				.UNSTABLE_addDisplacementMarkerOffset(-5, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					r.setPickup(true);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);

				})
				.UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM);
				})

				.splineTo(new Vector2d(40, -12),  Math.toRadians(30))//drive forward //was 33.5 for x
				.resetConstraints()
				.setReversed(true)
				.lineToLinearHeading(new Pose2d(32, -16.5,  Math.toRadians(20)))//drive forward //was 33.5 for x

				.UNSTABLE_addTemporalMarkerOffset(-2, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM);
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
					arm.presetTargetPosition(Arm.ArmPos.BACK_DELIVERY);
				})
				.UNSTABLE_addTemporalMarkerOffset(-1, () -> {
					guide.presetTargetPosition(Guide.GuidePos.ACTIVE);
				})
				.setReversed(false)
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HIGH_DROP);
				})


				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					arm.presetTargetPosition(Arm.ArmPos.BACK);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
					guide.presetTargetPosition(Guide.GuidePos.STOW);
				})
				.build();
	}
	
	private TrajectorySequence rightMediumPoleToStack(Lift.PoleHeights poleHeight, Vector2d offset){
		return drive.trajectorySequenceBuilder(trajectorySequenceArrayList.get(trajectorySequenceArrayList.size()-1).end())
				.addTemporalMarker(0.2, () -> {

					lift.presetLiftPosition(poleHeight);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					arm.presetTargetPosition(Arm.ArmPos.FRONT);
				})
				.lineTo(new Vector2d(57, -12).plus(offset))

				.UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				})
				.build();
	}
	
	private TrajectorySequence rightStackToMediumPole(Vector2d offset){
		return drive.trajectorySequenceBuilder(trajectorySequenceArrayList.get(trajectorySequenceArrayList.size()-1).end())
				.UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HOVER);

				})
				.waitSeconds(0.05)

				.lineTo(new Vector2d(33, -16).plus(offset))
				.UNSTABLE_addTemporalMarkerOffset(-2.1, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
				})
				.UNSTABLE_addTemporalMarkerOffset(-1.7, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM);
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
					arm.presetTargetPosition(Arm.ArmPos.BACK_DELIVERY);
				})
				.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
					guide.presetTargetPosition(Guide.GuidePos.ACTIVE);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM_DROP);
					arm.presetTargetPosition(Arm.ArmPos.BACK);
				})
				.waitSeconds(0.15)
				.UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
					guide.presetTargetPosition(Guide.GuidePos.STOW);
				})
				.build();
	}

	private TrajectorySequence rightStartMidlineHighPole(){
		drive.setPoseEstimate(startPoseRight);
		return drive.trajectorySequenceBuilder(startPoseRight)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
					r.setPickup(true);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HOVER);
				})

				.splineTo(new Vector2d(34.0, -31.0),  Math.toRadians(90))//drive forward
				.splineTo(new Vector2d(29,-8), Math.toRadians(162))//turn //TODO change
				.UNSTABLE_addTemporalMarkerOffset(-2, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HIGH);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
				})
				.UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
					guide.presetTargetPosition(Guide.GuidePos.ACTIVE);
				})
				.waitSeconds(0.5)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					arm.presetTargetPosition(Arm.ArmPos.FRONT);
				})
				.UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HIGH_DROP);
				})
				.waitSeconds(0.15)
				.UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
					guide.presetTargetPosition(Guide.GuidePos.STOW);
				})
				.build();
	}

	private TrajectorySequence rightMidlineHighPoleToStack(Lift.PoleHeights poleHeight, Vector2d offset){
		return drive.trajectorySequenceBuilder(trajectorySequenceArrayList.get(trajectorySequenceArrayList.size()-1).end())
				.waitSeconds(0.05)
				.UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
					lift.presetLiftPosition(poleHeight);
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
					arm.presetTargetPosition(Arm.ArmPos.BACK);
				})

				.lineTo(new Vector2d(58, -14).plus(offset))

				.UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				})
				.build();
	}

	private TrajectorySequence rightStackToMidlineHighPole(Vector2d offset){
		return drive.trajectorySequenceBuilder(trajectorySequenceArrayList.get(trajectorySequenceArrayList.size()-1).end())
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HIGH);
				})
				.waitSeconds(0.2)
				.UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
				})
				.lineTo(new Vector2d(29, -8).plus(offset))
				.addDisplacementMarker(5, () -> {
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
					//lift.presetLiftPosition(Lift.PoleHeights.HIGH);
				})
				.addDisplacementMarker(15, () -> {
					guide.presetTargetPosition(Guide.GuidePos.ACTIVE);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					arm.presetTargetPosition(Arm.ArmPos.FRONT);
				})
				.UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HIGH_DROP);
				})
				.waitSeconds(0.15)
				.UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
					guide.presetTargetPosition(Guide.GuidePos.STOW);
				})
				.build();
	}
	
	private TrajectorySequence leftStartMediumPole(){
		drive.setPoseEstimate(startPoseLeft);
		return drive.trajectorySequenceBuilder(startPoseLeft)
				.setConstraints(getVelocityConstraint(35, Math.toRadians(90),12.76), getAccelerationConstraint(35))
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
					r.setPickup(true);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HOVER);
				})
				.splineTo(new Vector2d(-34, -16),  Math.toRadians(90))//drive forward
				.turn(Math.toRadians(-110))
				.UNSTABLE_addTemporalMarkerOffset(-1.6, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
				})
				.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
					guide.presetTargetPosition(Guide.GuidePos.ACTIVE);

				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM_DROP);
					arm.presetTargetPosition(Arm.ArmPos.FRONT);
				})
				.waitSeconds(0.25)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
					guide.presetTargetPosition(Guide.GuidePos.STOW);
				})
				.build();
	}
	
	private TrajectorySequence leftMediumPoleToStack(Lift.PoleHeights poleHeight, Vector2d offset){
		return drive.trajectorySequenceBuilder(trajectorySequenceArrayList.get(trajectorySequenceArrayList.size()-1).end())

				.UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
					lift.presetLiftPosition(poleHeight);
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
					arm.presetTargetPosition(Arm.ArmPos.BACK);
				})
				.lineTo(new Vector2d(-57, -13).plus(offset))
				.waitSeconds(0)
				.UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				})
				.build();
	}
	
	private TrajectorySequence leftStackToMediumPole(Vector2d offset){
		return drive.trajectorySequenceBuilder(trajectorySequenceArrayList.get(trajectorySequenceArrayList.size()-1).end())
				.setConstraints(getVelocityConstraint(29, Math.toRadians(90),12.76), getAccelerationConstraint(35))
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM);
				})
				.waitSeconds(0.2)
				.UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
				})
				.lineTo(new Vector2d(-34, -14).plus(offset))
				.UNSTABLE_addTemporalMarkerOffset(-1.2, () -> {
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
				})
				.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
					guide.presetTargetPosition(Guide.GuidePos.ACTIVE);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM_DROP);
					arm.presetTargetPosition(Arm.ArmPos.FRONT);
				})
				.waitSeconds(0.25)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
					guide.presetTargetPosition(Guide.GuidePos.STOW);
				})
				.build();
	}

	public TrajectorySequenceStorage rightRiley(
			RobotConfig r,
			SampleMecanumDrive drive,
			Lift lift,
			Intake intake,
			Arm arm,
			Wrist wrist,
			Webcam webcam,
			Guide guide
	){
		this.r = r;
		this.drive = drive;
		this.lift = lift;
		this.intake = intake;
		this.arm = arm;
		this.wrist = wrist;
		this.guide = guide;

		this.sequenceIndex = 0;

		trajectorySequenceArrayList = new ArrayList<>();
		trajectorySequenceArrayList.add(rightRiley());

		trajectorySequenceArrayList.add(parkingPlaceholder());

		trajectorySequences = new TrajectorySequence[trajectorySequenceArrayList.size()];
		for (int i = 0; i < trajectorySequenceArrayList.size(); i++) {
			trajectorySequences[i] = trajectorySequenceArrayList.get(i);
		}

		return this;
	}

	
	public TrajectorySequenceStorage sampleSequenceStorage(
			RobotConfig r,
			SampleMecanumDrive drive,
			Lift lift,
			Intake intake,
			Arm arm,
			Wrist wrist,
			Webcam webcam
		){
		this.r = r;
		this.drive = drive;
		this.lift = lift;
		this.intake = intake;
		this.arm = arm;
		this.wrist = wrist;
		this.sequenceIndex = 0;
		
		trajectorySequenceArrayList = new ArrayList<>();
		
		//use trajectorySequenceArrayList.add(<sequence>); here to sequence them up
		
		trajectorySequenceArrayList.add(parkingPlaceholder());
		
		trajectorySequences = new TrajectorySequence[trajectorySequenceArrayList.size()];
		for (int i = 0; i < trajectorySequenceArrayList.size(); i++) {
			trajectorySequences[i] = trajectorySequenceArrayList.get(i);
		}
		
		return this;
	}

	public TrajectorySequenceStorage rightMediumPark(
			RobotConfig r,
			SampleMecanumDrive drive,
			Lift lift,
			Intake intake,
			Arm arm,
			Wrist wrist,
			Webcam webcam
	){
		this.r = r;
		this.drive = drive;
		this.lift = lift;
		this.intake = intake;
		this.arm = arm;
		this.wrist = wrist;
		this.sequenceIndex = 0;

		trajectorySequenceArrayList = new ArrayList<>();
		trajectorySequenceArrayList.add(rightStartMediumPark());


		trajectorySequenceArrayList.add(parkingPlaceholder());

		trajectorySequences = new TrajectorySequence[trajectorySequenceArrayList.size()];
		for (int i = 0; i < trajectorySequenceArrayList.size(); i++) {
			trajectorySequences[i] = trajectorySequenceArrayList.get(i);
		}

		return this;
	}

	public TrajectorySequenceStorage leftMediumPark(
			RobotConfig r,
			SampleMecanumDrive drive,
			Lift lift,
			Intake intake,
			Arm arm,
			Wrist wrist,
			Webcam webcam
	){
		this.r = r;
		this.drive = drive;
		this.lift = lift;
		this.intake = intake;
		this.arm = arm;
		this.wrist = wrist;
		this.sequenceIndex = 0;

		trajectorySequenceArrayList = new ArrayList<>();
		trajectorySequenceArrayList.add(leftStartMediumPark());


		trajectorySequenceArrayList.add(parkingPlaceholder());

		trajectorySequences = new TrajectorySequence[trajectorySequenceArrayList.size()];
		for (int i = 0; i < trajectorySequenceArrayList.size(); i++) {
			trajectorySequences[i] = trajectorySequenceArrayList.get(i);
		}

		return this;
	}


	public TrajectorySequenceStorage leftMedium5(
			RobotConfig r,
			SampleMecanumDrive drive,
			Intake intake,
			Lift lift,
			Arm arm,
			Wrist wrist,
			Guide guide,
			Webcam webcam
	){
		this.r = r;
		this.drive = drive;
		this.lift = lift;
		this.intake = intake;
		this.arm = arm;
		this.wrist = wrist;
		this.guide = guide;
		this.sequenceIndex = 0;
		
		trajectorySequenceArrayList = new ArrayList<>();
		intake.presetTargetPosition(Intake.IntakePos.CLOSED);
		trajectorySequenceArrayList.add(leftStartMediumPole());
		
		trajectorySequenceArrayList.add(leftMediumPoleToStack(Lift.PoleHeights.STACK4, new Vector2d(0, 0)));
		trajectorySequenceArrayList.add(leftStackToMediumPole(new Vector2d(0.0, -0)));

		
		trajectorySequenceArrayList.add(leftMediumPoleToStack(Lift.PoleHeights.STACK3, new Vector2d(0, 0)));
		trajectorySequenceArrayList.add(leftStackToMediumPole(new Vector2d(0, -0)));
		
		trajectorySequenceArrayList.add(leftMediumPoleToStack(Lift.PoleHeights.STACK2, new Vector2d(0, 0)));
		trajectorySequenceArrayList.add(leftStackToMediumPole(new Vector2d(0, -0)));
		
		trajectorySequenceArrayList.add(leftMediumPoleToStack(Lift.PoleHeights.STACK1, new Vector2d(0, 0)));
		trajectorySequenceArrayList.add(leftStackToMediumPole(new Vector2d(0, -0)));
		
		//trajectorySequenceArrayList.add(leftMediumPoleToStack(Lift.PoleHeights.STACK0, new Vector2d(0, 0)));
		//trajectorySequenceArrayList.add(leftStackToMediumPole(new Vector2d(0, -0)));
		
		trajectorySequenceArrayList.add(parkingPlaceholder());
		
		trajectorySequences = new TrajectorySequence[trajectorySequenceArrayList.size()];
		for (int i = 0; i < trajectorySequenceArrayList.size(); i++) {
			trajectorySequences[i] = trajectorySequenceArrayList.get(i);
		}
		
		return this;
	}

	public TrajectorySequenceStorage rightMedium5(
			RobotConfig r,
			SampleMecanumDrive drive,
			Intake intake,
			Arm arm,
			Lift lift,
			Wrist wrist,
			Guide guide,
			Webcam webcam,
			BackBrace backBrace,
			FrontBrace frontBrace){
		this.r = r;
		this.drive = drive;
		this.lift = lift;
		this.intake = intake;
		this.arm = arm;
		this.guide = guide;
		this.wrist = wrist;
		this.backBrace = backBrace;
		this.frontBrace = frontBrace;
		this.sequenceIndex = 0;
		
		trajectorySequenceArrayList = new ArrayList<>();
		intake.presetTargetPosition(Intake.IntakePos.CLOSED);

		trajectorySequenceArrayList.add(rightStartMediumPole());
		trajectorySequenceArrayList.add(rightMediumPoleToStack(Lift.PoleHeights.STACK4, new Vector2d(0, 0))); //X offset was 0
		trajectorySequenceArrayList.add(rightStackToMediumPole(new Vector2d(0, 0)));
		
		trajectorySequenceArrayList.add(rightMediumPoleToStack(Lift.PoleHeights.STACK3, new Vector2d(0, 0))); //X offset was 0
		trajectorySequenceArrayList.add(rightStackToMediumPole(new Vector2d(0, 0)));
		
		trajectorySequenceArrayList.add(rightMediumPoleToStack(Lift.PoleHeights.STACK2, new Vector2d(0, 0)));
		trajectorySequenceArrayList.add(rightStackToMediumPole(new Vector2d(0, 0)));
		
		trajectorySequenceArrayList.add(rightMediumPoleToStack(Lift.PoleHeights.STACK1, new Vector2d(0, 0)));
		trajectorySequenceArrayList.add(rightStackToMediumPole(new Vector2d(0, 0)));
		
		//trajectorySequenceArrayList.add(rightMediumPoleToStack(Lift.PoleHeights.STACK0, new Vector2d(0, 0)));
		//trajectorySequenceArrayList.add(rightStackToMediumPole(new Vector2d(0, 0)));
		
		trajectorySequenceArrayList.add(parkingPlaceholder());
		
		trajectorySequences = new TrajectorySequence[trajectorySequenceArrayList.size()];
		for (int i = 0; i < trajectorySequenceArrayList.size(); i++) {
			trajectorySequences[i] = trajectorySequenceArrayList.get(i);
		}

		return this;
	}

	public TrajectorySequenceStorage rightMidlineHigh5(
			RobotConfig r,
			SampleMecanumDrive drive,
			Intake intake,
			Lift lift,
			Arm arm,
			Wrist wrist,
			Guide guide, BackBrace backBrace, Webcam webcam
	){
		this.r = r;
		this.drive = drive;
		this.lift = lift;
		this.intake = intake;
		this.arm = arm;
		this.wrist = wrist;
		this.guide = guide;
		this.sequenceIndex = 0;

		trajectorySequenceArrayList = new ArrayList<>();
		intake.presetTargetPosition(Intake.IntakePos.CLOSED);
		trajectorySequenceArrayList.add(rightStartMidlineHighPole());

		trajectorySequenceArrayList.add(rightMidlineHighPoleToStack(Lift.PoleHeights.STACK4, new Vector2d(0.0, 0.0)));
		trajectorySequenceArrayList.add(rightStackToMidlineHighPole(new Vector2d(0.0, 0.0)));

		trajectorySequenceArrayList.add(rightMidlineHighPoleToStack(Lift.PoleHeights.STACK3, new Vector2d(0.0, 0.0)));
		trajectorySequenceArrayList.add(rightStackToMidlineHighPole(new Vector2d(0.0, 0.0)));

		trajectorySequenceArrayList.add(rightMidlineHighPoleToStack(Lift.PoleHeights.STACK2, new Vector2d(0.0, 0.0)));
		trajectorySequenceArrayList.add(rightStackToMidlineHighPole(new Vector2d(-0.2, 0.2)));

		trajectorySequenceArrayList.add(rightMidlineHighPoleToStack(Lift.PoleHeights.STACK1, new Vector2d(0.0, 0.0)));
		trajectorySequenceArrayList.add(rightStackToMidlineHighPole(new Vector2d(-0.4, 0.4)));

		//trajectorySequenceArrayList.add(rightMidlineHighPoleToStack(Lift.PoleHeights.STACK0, new Vector2d(0.0, 0.0)));
		//trajectorySequenceArrayList.add(rightStackToMidlineHighPole(new Vector2d(-0.6, 0.6)));

		trajectorySequenceArrayList.add(parkingPlaceholder());

		trajectorySequences = new TrajectorySequence[trajectorySequenceArrayList.size()];
		for (int i = 0; i < trajectorySequenceArrayList.size(); i++) {
			trajectorySequences[i] = trajectorySequenceArrayList.get(i);
		}

		return this;
	}


	public void followSetSequenceAsync(){
		if(!drive.isBusy()){
			if(sequenceIndex >= trajectorySequences.length - 1) {
				return;
			}
			drive.followTrajectorySequenceAsync(trajectorySequences[++sequenceIndex]);
			
		}
		drive.update();
	}
	
	public void startFollowSetSequenceAsync(){
		if(sequenceIndex >= trajectorySequences.length){
			return;
		}
		drive.followTrajectorySequenceAsync(trajectorySequences[sequenceIndex]);
	}

	public void addShortLeftParkSequence(int tagid){
		switch (tagid){
			case 1:
				trajectorySequences[trajectorySequences.length-1] = shortLeftPark1(trajectorySequences[trajectorySequences.length-2].end());
				break;
			case 3:
				trajectorySequences[trajectorySequences.length-1] = shortLeftPark3(trajectorySequences[trajectorySequences.length-2].end());
				break;
			default:
				trajectorySequences[trajectorySequences.length-1] = shortLeftPark2(trajectorySequences[trajectorySequences.length-2].end());
				break;
		}
	}

	public void addShortRightParkSequence(int tagid){
		switch (tagid){
			case 1:
				trajectorySequences[trajectorySequences.length-1] = shortRightPark1(trajectorySequences[trajectorySequences.length-2].end());
				break;
			case 3:
				trajectorySequences[trajectorySequences.length-1] = shortRightPark3(trajectorySequences[trajectorySequences.length-2].end());
				break;
			default:
				trajectorySequences[trajectorySequences.length-1] = shortRightPark2(trajectorySequences[trajectorySequences.length-2].end());
				break;
		}
	}
	
	public void addRightParkSequence(int tagid){
		switch (tagid){
			case 1:
				trajectorySequences[trajectorySequences.length-1] = rightMediumPark1(trajectorySequences[trajectorySequences.length-2].end());
				break;
			case 3:
				trajectorySequences[trajectorySequences.length-1] = rightMediumPark3(trajectorySequences[trajectorySequences.length-2].end());
				break;
			default:
				trajectorySequences[trajectorySequences.length-1] = rightMediumPark2(trajectorySequences[trajectorySequences.length-2].end());
				break;
		}
	}
	
	public void addLeftParkSequence(int tagid){
		switch (tagid){
			case 1:
				trajectorySequences[trajectorySequences.length-1] = leftPark1(trajectorySequences[trajectorySequences.length-2].end());
				break;
			case 3:
				trajectorySequences[trajectorySequences.length-1] = leftPark3(trajectorySequences[trajectorySequences.length-2].end());
				break;
			default:
				trajectorySequences[trajectorySequences.length-1] = leftPark2(trajectorySequences[trajectorySequences.length-2].end());
				break;
		}
	}
	
	public void addRightHighParkSequence(int tagid){
		switch (tagid){
			case 1:
				trajectorySequences[trajectorySequences.length-1] = rightHighPark1(trajectorySequences[trajectorySequences.length-2].end());
				break;
			case 3:
				trajectorySequences[trajectorySequences.length-1] = rightHighPark3(trajectorySequences[trajectorySequences.length-2].end());
				break;
			default:
				trajectorySequences[trajectorySequences.length-1] = rightHighPark2(trajectorySequences[trajectorySequences.length-2].end());
				break;
		}
	}
	
	private TrajectorySequence parkingPlaceholder(){
		return drive.trajectorySequenceBuilder(trajectorySequenceArrayList.get(trajectorySequenceArrayList.size()-1).end())
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.back(1)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					arm.presetTargetPosition(Arm.ArmPos.HALF);
				})
				.build();
	}

	private TrajectorySequence shortRightPark1(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				})
				.lineToLinearHeading(new Pose2d(6, -37.5, Math.toRadians(-89)))
				.lineTo(new Vector2d(6, -32))
				.build();
	}

	private TrajectorySequence shortRightPark2(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				})
				.lineToLinearHeading(new Pose2d(36, -37.5, Math.toRadians(-89)))
				.lineTo(new Vector2d(36, -32))
				.build();
	}

	private TrajectorySequence shortRightPark3(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				})
				.lineToLinearHeading(new Pose2d(62, -37.5, Math.toRadians(-89)))
				.lineTo(new Vector2d(62, -32))
				.build();
	}

	private TrajectorySequence shortLeftPark1(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				})
				.lineToLinearHeading(new Pose2d(-64, -36, Math.toRadians(90)))
				.build();
	}

	private TrajectorySequence shortLeftPark2(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				})
				.lineToLinearHeading(new Pose2d(-37, -36, Math.toRadians(90)))
				.build();
	}

	private TrajectorySequence shortLeftPark3(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				})
				.lineToLinearHeading(new Pose2d(-6, -36, Math.toRadians(90)))
				.build();
	}



	private TrajectorySequence rightMediumPark1(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
					backBrace.presetbracePosition(BackBrace.BracePositions.UP);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				})
				.splineTo(new Vector2d(50,-12), Math.toRadians(0))
				.setReversed(true)
				.splineTo(new Vector2d(24,-12),Math.toRadians(180))
				.splineTo(new Vector2d(12,-24),Math.toRadians(270))
				.build();
	}
	
	private TrajectorySequence rightMediumPark2(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				/*.addTemporalMarker(0.2, () -> {

					lift.presetLiftPosition(Lift.PoleHeights.STACK0);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					arm.presetTargetPosition(Arm.ArmPos.FRONT);
				})
				.lineTo(new Vector2d(57, -12))

				.UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				})
				.waitSeconds(0.5)
				.UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HOVER);

				})
				.waitSeconds(0.05)

				.lineTo(new Vector2d(34, -15))
				.UNSTABLE_addTemporalMarkerOffset(-2.1, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
				})
				.UNSTABLE_addTemporalMarkerOffset(-1.7, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM);
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
					arm.presetTargetPosition(Arm.ArmPos.BACK_DELIVERY);
				})
				.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
					guide.presetTargetPosition(Guide.GuidePos.ACTIVE);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM_DROP);
					arm.presetTargetPosition(Arm.ArmPos.BACK);
				})
				.waitSeconds(0.15)
				.UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
					guide.presetTargetPosition(Guide.GuidePos.STOW);
				})*/

				.lineToLinearHeading(new Pose2d(36, -16, Math.toRadians(90)))
				.UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);

				})
				.build();
	}
	
	private TrajectorySequence rightMediumPark3(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				/*.addTemporalMarker(0.2, () -> {

					lift.presetLiftPosition(Lift.PoleHeights.STACK0);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					arm.presetTargetPosition(Arm.ArmPos.FRONT);
				})
				.lineTo(new Vector2d(57, -11))
				.waitSeconds(0.01)

				.UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.HOVER);

				})
				.waitSeconds(0.05)

				.lineTo(new Vector2d(34, -13))

				.UNSTABLE_addTemporalMarkerOffset(-1.7, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM);
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
					arm.presetTargetPosition(Arm.ArmPos.BACK_DELIVERY);
				})
				.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
					guide.presetTargetPosition(Guide.GuidePos.ACTIVE);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM_DROP);
					arm.presetTargetPosition(Arm.ArmPos.BACK);
				})
				.waitSeconds(0.15)
				.UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
					guide.presetTargetPosition(Guide.GuidePos.STOW);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				})

				 */
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				})
				.splineTo(new Vector2d(48,-12), Math.toRadians(0))
				.splineTo(new Vector2d(60,-24), Math.toRadians(270))
				.build();
	}
	
	private TrajectorySequence leftPark1(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				})
				.lineToLinearHeading(new Pose2d(-60, -12, Math.toRadians(90)))
				.build();
	}
	
	private TrajectorySequence leftPark2(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)

				.UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
					arm.presetTargetPosition(Arm.ArmPos.BACK);
				})
				.lineTo(new Vector2d(-58.5, -12))
				.waitSeconds(0)
				.UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
				})

				.waitSeconds(0.1)

				.setConstraints(getVelocityConstraint(29, Math.toRadians(90),12.76), getAccelerationConstraint(35))
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM);
				})
				.waitSeconds(0.2)
				.UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
					wrist.presetTargetPosition(Wrist.WristPos.FRONT);
				})
				.lineTo(new Vector2d(-34, -14))
				.UNSTABLE_addTemporalMarkerOffset(-1.2, () -> {
					arm.presetTargetPosition(Arm.ArmPos.FRONT_DELIVERY);
				})
				.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
					guide.presetTargetPosition(Guide.GuidePos.ACTIVE);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					lift.presetLiftPosition(Lift.PoleHeights.MEDIUM_DROP);
					arm.presetTargetPosition(Arm.ArmPos.FRONT);
				})
				.waitSeconds(0.25)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
					guide.presetTargetPosition(Guide.GuidePos.STOW);
				})

				.waitSeconds(0)

				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				})
				.lineToLinearHeading(new Pose2d(-37, -12, Math.toRadians(90)))
				.build();
	}
	
	private TrajectorySequence leftPark3(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
				})
				.lineToLinearHeading(new Pose2d(-12, -14, Math.toRadians(90)))
				.build();
	}
	
	private TrajectorySequence rightHighPark1(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.setConstraints(getVelocityConstraint(35, Math.toRadians(90),12.76), getAccelerationConstraint(35))
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
				})
				.setReversed(true)
				.splineTo(new Vector2d(38,-12), Math.toRadians(0))
				.setReversed(false)
				.splineTo(new Vector2d(24,-12), Math.toRadians(180))
				.splineTo(new Vector2d(12,-24),Math.toRadians(270))
				.build();
	}
	
	private TrajectorySequence rightHighPark2(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)





				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
				})
				.lineToLinearHeading(new Pose2d(36, -16, Math.toRadians(270)))
				.build();
	}
	
	private TrajectorySequence rightHighPark3(Pose2d startPose){
		return drive.trajectorySequenceBuilder(startPose)
				.UNSTABLE_addTemporalMarkerOffset(0, () -> {
					intake.presetTargetPosition(Intake.IntakePos.OPEN);
				})
				.waitSeconds(0.1)
				.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
					arm.presetTargetPosition(Arm.ArmPos.HALF);
					intake.presetTargetPosition(Intake.IntakePos.CLOSED);
					lift.presetLiftPosition(Lift.PoleHeights.GROUND);
					wrist.presetTargetPosition(Wrist.WristPos.BACK);
				})
				.setReversed(true)
				.splineTo(new Vector2d(48,-12), Math.toRadians(0))
				.splineTo(new Vector2d(60,-24), Math.toRadians(270))
				.build();
	}
}