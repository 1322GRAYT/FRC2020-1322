package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public abstract class HolonomicDrivetrainSubsystem extends SubsystemBase {

	private double mAdjustmentAngle = 0;
	private boolean mFieldOriented = true;
	private boolean isAuto = false;

	public boolean getIsAuto() {
		return isAuto;
	}

	public void setIsAuto(boolean is) {
		isAuto = is;
	}

	public double getAdjustmentAngle() {
		return mAdjustmentAngle;
	}

	public abstract double getGyroAngle();

	public abstract void HolonomicDrv(double forward, double strafe, double rotation, boolean fieldOriented);

	public boolean isFieldOriented() {
		return mFieldOriented;
	}

	public void setAdjustmentAngle(double adjustmentAngle) {
		mAdjustmentAngle = adjustmentAngle;
	}

	public void setFieldOriented(boolean fieldOriented) {
		mFieldOriented = fieldOriented;
	}

	public abstract void stopDrvMtrs();

	public void zeroGyro() {
		setAdjustmentAngle(getGyroAngle() + getAdjustmentAngle());
	}
}
