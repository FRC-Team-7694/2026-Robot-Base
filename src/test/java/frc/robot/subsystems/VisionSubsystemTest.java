package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.photonvision.targeting.PhotonPipelineMetadata;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

class VisionSubsystemTest {
  private static final double EPSILON = 1.0e-9;

  @BeforeEach
  void clearDashboard() {
    SmartDashboard.clearPersistent("Vision/PhotonVision/Camera Name");
  }

  @Test
  void calculateDistanceUsesConfiguredProofOfConceptGeometry() {
    double targetPitchDegrees = -15.0;
    double distanceMeters = org.photonvision.PhotonUtils.calculateDistanceToTargetMeters(
        VisionConstants.kCameraHeightMeters,
        VisionConstants.kTargetHeightMeters,
        VisionConstants.kCameraPitchRadians,
        edu.wpi.first.math.util.Units.degreesToRadians(targetPitchDegrees));

    assertEquals(0.935, Math.abs(distanceMeters), 0.000001);
  }

  @Test
  void periodicDrainsOnceAndDisconnectInvalidates() {
    FakeCamera camera = new FakeCamera();
    camera.connected = true;
    camera.results = List.of(
        frame(100.0, 1, 2.5, 3.0, 1.0, 0.2, 17, 18));

    VisionSubsystem subsystem = new VisionSubsystem(camera);
    subsystem.periodic();

    assertEquals(1, camera.unreadCalls);
    assertEquals(2, SmartDashboard.getNumber("Vision/PhotonVision/Target Count", -1), 0.0);
    assertEquals(100.0, SmartDashboard.getNumber("Vision/PhotonVision/Latency Ms", -1), EPSILON);

    camera.connected = false;
    camera.results = List.of(frame(200.0, 2, 4.0, 5.0, 2.0, 0.4, 21, 22));
    subsystem.periodic();

    assertEquals(2, camera.unreadCalls);
    assertEquals(-1.0, SmartDashboard.getNumber("Vision/PhotonVision/Best Target ID", 0), EPSILON);
    assertEquals(0.0, SmartDashboard.getNumber("Vision/PhotonVision/Target Count", -1), EPSILON);
  }

  @Test
  void noFramePreservesPriorFrameValues() {
    FakeCamera camera = new FakeCamera();
    camera.connected = true;
    camera.results = List.of(frame(55.0, 7, 11.0, 12.0, 3.5, 0.1, 5, 6));
    VisionSubsystem subsystem = new VisionSubsystem(camera);

    subsystem.periodic();
    camera.results = List.of();
    subsystem.periodic();

    assertEquals(false, SmartDashboard.getBoolean("Vision/PhotonVision/Has New Frame", true));
    assertEquals(55.0, SmartDashboard.getNumber("Vision/PhotonVision/Latency Ms", -1), EPSILON);
    assertEquals(2, (int) SmartDashboard.getNumber("Vision/PhotonVision/Target Count", -1));
    assertEquals(5.0, SmartDashboard.getNumber("Vision/PhotonVision/Best Target ID", -1), EPSILON);
  }

  @Test
  void multiTargetFrameIsPreservedAcrossNoFrameLoop() {
    FakeCamera camera = new FakeCamera();
    camera.connected = true;
    camera.results = List.of(frame(42.0, 3, 13.0, 14.0, 4.5, 0.3, 9, 10, 11));
    VisionSubsystem subsystem = new VisionSubsystem(camera);

    subsystem.periodic();
    camera.results = List.of();
    subsystem.periodic();

    assertEquals(3.0, SmartDashboard.getNumber("Vision/PhotonVision/Target Count", -1), EPSILON);
    double[] ids = SmartDashboard.getNumberArray("Vision/PhotonVision/Detected Fiducial IDs", new double[0]);
    assertEquals(3, ids.length);
    assertEquals(9.0, ids[0], EPSILON);
    assertEquals(10.0, ids[1], EPSILON);
    assertEquals(11.0, ids[2], EPSILON);
  }

  @Test
  void freshNoTargetFrameInvalidatesOnlyTargetState() {
    FakeCamera camera = new FakeCamera();
    camera.connected = true;
    camera.results = List.of(frame(70.0, 8, 15.0, 16.0, 5.5, 0.5, 12));
    VisionSubsystem subsystem = new VisionSubsystem(camera);

    subsystem.periodic();
    camera.results = List.of(frameNoTargets(71.5));
    subsystem.periodic();

    assertEquals(71.5, SmartDashboard.getNumber("Vision/PhotonVision/Latency Ms", -1), EPSILON);
    assertEquals(0.0, SmartDashboard.getNumber("Vision/PhotonVision/Target Count", -1), EPSILON);
    assertEquals(-1.0, SmartDashboard.getNumber("Vision/PhotonVision/Best Target ID", 0), EPSILON);
    assertEquals(true, Double.isNaN(SmartDashboard.getNumber("Vision/PhotonVision/Yaw Degrees", 0)));
  }

  @Test
  void trueMetadataLatencyIsPublished() {
    FakeCamera camera = new FakeCamera();
    camera.connected = true;
    camera.results = List.of(frame(88.25, 19, 22.0, 23.0, 6.5, 0.6, 30));
    VisionSubsystem subsystem = new VisionSubsystem(camera);

    subsystem.periodic();

    assertEquals(88.25, SmartDashboard.getNumber("Vision/PhotonVision/Latency Ms", -1), EPSILON);
  }

  private static PhotonPipelineResult frame(
      double latencyMs,
      int bestId,
      double yaw,
      double pitch,
      double area,
      double ambiguity,
      double... ids) {
    List<PhotonTrackedTarget> targets = java.util.Arrays.stream(ids)
        .mapToObj(id -> target(id, id == bestId ? yaw : yaw + 1.0, id == bestId ? pitch : pitch + 1.0, area, ambiguity))
        .toList();
    PhotonPipelineResult result = new PhotonPipelineResult(
        new PhotonPipelineMetadata(0, 0, 0, 0), targets, java.util.Optional.empty());
    result.metadata = new PhotonPipelineMetadata(0, 0, 0, 0) {
      @Override
      public double getLatencyMillis() {
        return latencyMs;
      }
    };
    return result;
  }

  private static PhotonPipelineResult frameNoTargets(double latencyMs) {
    PhotonPipelineResult result = new PhotonPipelineResult(
        new PhotonPipelineMetadata(0, 0, 0, 0), List.of(), java.util.Optional.empty());
    result.metadata = new PhotonPipelineMetadata(0, 0, 0, 0) {
      @Override
      public double getLatencyMillis() {
        return latencyMs;
      }
    };
    return result;
  }

  private static PhotonTrackedTarget target(
      double fiducialId,
      double yaw,
      double pitch,
      double area,
      double ambiguity) {
    PhotonTrackedTarget target = new PhotonTrackedTarget();
    target.fiducialId = (int) fiducialId;
    target.yaw = yaw;
    target.pitch = pitch;
    target.area = area;
    target.poseAmbiguity = ambiguity;
    return target;
  }

  private static final class FakeCamera implements VisionSubsystem.VisionCameraInput {
    boolean connected;
    int unreadCalls;
    List<PhotonPipelineResult> results = List.of();

    @Override
    public boolean isConnected() {
      return connected;
    }

    @Override
    public int getPipelineIndex() {
      return 2;
    }

    @Override
    public List<PhotonPipelineResult> getAllUnreadResults() {
      unreadCalls++;
      return results;
    }
  }
}
