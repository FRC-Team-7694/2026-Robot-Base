package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
  interface VisionCameraInput {
    boolean isConnected();

    int getPipelineIndex();

    List<PhotonPipelineResult> getAllUnreadResults();
  }

  private final VisionCameraInput m_camera;
  private double m_lastLatencyMs = Double.NaN;
  private double m_lastTimestampSeconds = Double.NaN;
  private int m_lastTargetCount;
  private double[] m_lastDetectedFiducialIds = new double[0];
  private boolean m_lastHasTargets;
  private double m_lastBestTargetId = -1.0;
  private double m_lastYawDegrees = Double.NaN;
  private double m_lastPitchDegrees = Double.NaN;
  private double m_lastAreaPercent = Double.NaN;
  private double m_lastPoseAmbiguity = Double.NaN;
  private double m_lastDistanceMeters = Double.NaN;

  public VisionSubsystem() {
    this(new PhotonCamera(VisionConstants.kCameraName));
  }

  VisionSubsystem(PhotonCamera camera) {
    this(new VisionCameraInput() {
      @Override
      public boolean isConnected() {
        return camera.isConnected();
      }

      @Override
      public int getPipelineIndex() {
        return camera.getPipelineIndex();
      }

      @Override
      public List<PhotonPipelineResult> getAllUnreadResults() {
        return camera.getAllUnreadResults();
      }
    });
  }

  VisionSubsystem(VisionCameraInput camera) {
    m_camera = camera;
    invalidateAll();
    publishStaticState();
  }

  @Override
  public void periodic() {
    boolean connected = m_camera.isConnected();
    SmartDashboard.putString("Vision/PhotonVision/Camera Name", VisionConstants.kCameraName);
    SmartDashboard.putBoolean("Vision/PhotonVision/Connected", connected);

    List<PhotonPipelineResult> unreadResults = m_camera.getAllUnreadResults();
    if (!connected) {
      invalidateAll();
      SmartDashboard.putBoolean("Vision/PhotonVision/Has New Frame", false);
      SmartDashboard.putNumber("Vision/PhotonVision/Active Pipeline Index", -1);
      publishStaticState();
      return;
    }

    SmartDashboard.putNumber("Vision/PhotonVision/Active Pipeline Index", m_camera.getPipelineIndex());

    if (unreadResults.isEmpty()) {
      SmartDashboard.putBoolean("Vision/PhotonVision/Has New Frame", false);
      publishPreservedFrameState();
      return;
    }

    SmartDashboard.putBoolean("Vision/PhotonVision/Has New Frame", true);
    publishResult(unreadResults.get(unreadResults.size() - 1));
  }

  private void publishResult(PhotonPipelineResult result) {
    m_lastLatencyMs = result.metadata.getLatencyMillis();
    m_lastTimestampSeconds = result.getTimestampSeconds();
    m_lastHasTargets = result.hasTargets();
    m_lastTargetCount = result.getTargets().size();
    m_lastDetectedFiducialIds = result.getTargets().stream().mapToDouble(PhotonTrackedTarget::getFiducialId).toArray();

    SmartDashboard.putNumber("Vision/PhotonVision/Latency Ms", m_lastLatencyMs);
    SmartDashboard.putNumber("Vision/PhotonVision/Timestamp Seconds", m_lastTimestampSeconds);
    SmartDashboard.putBoolean("Vision/PhotonVision/Has Targets", m_lastHasTargets);
    SmartDashboard.putNumber("Vision/PhotonVision/Target Count", m_lastTargetCount);
    SmartDashboard.putNumberArray("Vision/PhotonVision/Detected Fiducial IDs", m_lastDetectedFiducialIds);

    if (!m_lastHasTargets) {
      invalidateTargetState();
      SmartDashboard.putNumber("Vision/PhotonVision/Latency Ms", m_lastLatencyMs);
      SmartDashboard.putNumber("Vision/PhotonVision/Timestamp Seconds", m_lastTimestampSeconds);
      return;
    }

    PhotonTrackedTarget bestTarget = result.getBestTarget();
    m_lastBestTargetId = bestTarget.getFiducialId();
    m_lastYawDegrees = bestTarget.getYaw();
    m_lastPitchDegrees = bestTarget.getPitch();
    m_lastAreaPercent = bestTarget.getArea();
    m_lastPoseAmbiguity = bestTarget.getPoseAmbiguity();
    m_lastDistanceMeters = PhotonUtils.calculateDistanceToTargetMeters(
        VisionConstants.kCameraHeightMeters,
        VisionConstants.kTargetHeightMeters,
        VisionConstants.kCameraPitchRadians,
        Units.degreesToRadians(bestTarget.getPitch()));

    publishTargetState();
  }

  private void publishPreservedFrameState() {
    SmartDashboard.putNumber("Vision/PhotonVision/Latency Ms", m_lastLatencyMs);
    SmartDashboard.putNumber("Vision/PhotonVision/Timestamp Seconds", m_lastTimestampSeconds);
    SmartDashboard.putBoolean("Vision/PhotonVision/Has Targets", m_lastHasTargets);
    SmartDashboard.putNumber("Vision/PhotonVision/Target Count", m_lastTargetCount);
    SmartDashboard.putNumberArray("Vision/PhotonVision/Detected Fiducial IDs", m_lastDetectedFiducialIds);
    if (m_lastHasTargets) {
      publishTargetState();
    } else {
      publishNoTargetState();
    }
  }

  private void publishTargetState() {
    SmartDashboard.putNumber("Vision/PhotonVision/Best Target ID", m_lastBestTargetId);
    SmartDashboard.putNumber("Vision/PhotonVision/Yaw Degrees", m_lastYawDegrees);
    SmartDashboard.putNumber("Vision/PhotonVision/Pitch Degrees", m_lastPitchDegrees);
    SmartDashboard.putNumber("Vision/PhotonVision/Area Percent", m_lastAreaPercent);
    SmartDashboard.putNumber("Vision/PhotonVision/Pose Ambiguity", m_lastPoseAmbiguity);
    SmartDashboard.putNumber("Vision/PhotonVision/POC Distance Meters", m_lastDistanceMeters);
  }

  private void publishNoTargetState() {
    SmartDashboard.putNumber("Vision/PhotonVision/Best Target ID", -1.0);
    SmartDashboard.putNumber("Vision/PhotonVision/Yaw Degrees", Double.NaN);
    SmartDashboard.putNumber("Vision/PhotonVision/Pitch Degrees", Double.NaN);
    SmartDashboard.putNumber("Vision/PhotonVision/Area Percent", Double.NaN);
    SmartDashboard.putNumber("Vision/PhotonVision/Pose Ambiguity", Double.NaN);
    SmartDashboard.putNumber("Vision/PhotonVision/POC Distance Meters", Double.NaN);
  }

  private void invalidateAll() {
    m_lastLatencyMs = Double.NaN;
    m_lastTimestampSeconds = Double.NaN;
    m_lastTargetCount = 0;
    m_lastDetectedFiducialIds = new double[0];
    m_lastHasTargets = false;
    m_lastBestTargetId = -1.0;
    m_lastYawDegrees = Double.NaN;
    m_lastPitchDegrees = Double.NaN;
    m_lastAreaPercent = Double.NaN;
    m_lastPoseAmbiguity = Double.NaN;
    m_lastDistanceMeters = Double.NaN;

    SmartDashboard.putBoolean("Vision/PhotonVision/Has Targets", false);
    SmartDashboard.putNumber("Vision/PhotonVision/Target Count", 0);
    SmartDashboard.putNumberArray("Vision/PhotonVision/Detected Fiducial IDs", new double[0]);
    SmartDashboard.putNumber("Vision/PhotonVision/Best Target ID", -1.0);
    SmartDashboard.putNumber("Vision/PhotonVision/Yaw Degrees", Double.NaN);
    SmartDashboard.putNumber("Vision/PhotonVision/Pitch Degrees", Double.NaN);
    SmartDashboard.putNumber("Vision/PhotonVision/Area Percent", Double.NaN);
    SmartDashboard.putNumber("Vision/PhotonVision/Pose Ambiguity", Double.NaN);
    SmartDashboard.putNumber("Vision/PhotonVision/POC Distance Meters", Double.NaN);
    SmartDashboard.putNumber("Vision/PhotonVision/Latency Ms", Double.NaN);
    SmartDashboard.putNumber("Vision/PhotonVision/Timestamp Seconds", Double.NaN);
  }

  private void invalidateTargetState() {
    m_lastTargetCount = 0;
    m_lastDetectedFiducialIds = new double[0];
    m_lastHasTargets = false;
    m_lastBestTargetId = -1.0;
    m_lastYawDegrees = Double.NaN;
    m_lastPitchDegrees = Double.NaN;
    m_lastAreaPercent = Double.NaN;
    m_lastPoseAmbiguity = Double.NaN;
    m_lastDistanceMeters = Double.NaN;

    SmartDashboard.putBoolean("Vision/PhotonVision/Has Targets", false);
    SmartDashboard.putNumber("Vision/PhotonVision/Target Count", 0);
    SmartDashboard.putNumberArray("Vision/PhotonVision/Detected Fiducial IDs", new double[0]);
    SmartDashboard.putNumber("Vision/PhotonVision/Best Target ID", -1.0);
    SmartDashboard.putNumber("Vision/PhotonVision/Yaw Degrees", Double.NaN);
    SmartDashboard.putNumber("Vision/PhotonVision/Pitch Degrees", Double.NaN);
    SmartDashboard.putNumber("Vision/PhotonVision/Area Percent", Double.NaN);
    SmartDashboard.putNumber("Vision/PhotonVision/Pose Ambiguity", Double.NaN);
    SmartDashboard.putNumber("Vision/PhotonVision/POC Distance Meters", Double.NaN);
  }

  private void publishStaticState() {
    SmartDashboard.putString("Vision/PhotonVision/Camera Name", VisionConstants.kCameraName);
    SmartDashboard.putBoolean("Vision/PhotonVision/Connected", false);
  }
}
