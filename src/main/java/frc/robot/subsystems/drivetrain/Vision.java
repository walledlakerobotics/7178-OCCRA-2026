package frc.robot.subsystems.drivetrain;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import java.util.List;
import java.util.Optional;
import kotlin.Metadata;
import kotlin.collections.CollectionsKt;
import kotlin.jvm.internal.Intrinsics;
import kotlin.jvm.optionals.OptionalsKt;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

@Logged
@Metadata(
   mv = {2, 4, 0},
   k = 1,
   xi = 48,
   d1 = {"\u0000R\n\u0002\u0018\u0002\n\u0002\u0018\u0002\n\u0002\u0018\u0002\n\u0002\b\u0003\n\u0002\u0018\u0002\n\u0000\n\u0002\u0018\u0002\n\u0002\u0018\u0002\n\u0002\b\u0004\n\u0002\u0010\u0006\n\u0002\b\t\n\u0002\u0010 \n\u0002\u0018\u0002\n\u0002\b\u0007\n\u0002\u0018\u0002\n\u0002\b\u0003\n\u0002\u0018\u0002\n\u0002\b\u0003\n\u0002\u0010\u0002\n\u0002\b\u0002\n\u0002\u0018\u0002\b\u0007\u0018\u00002\u00060\u0001j\u0002`\u0002B\u0007¢\u0006\u0004\b\u0003\u0010\u0004J\u0006\u0010(\u001a\u00020)J\b\u0010*\u001a\u00020)H\u0016R\u000e\u0010\u0005\u001a\u00020\u0006X\u0082\u0004¢\u0006\u0002\n\u0000R-\u0010\u0007\u001a\u001e\u0012\f\u0012\n \n*\u0004\u0018\u00010\t0\t\u0012\f\u0012\n \n*\u0004\u0018\u00010\t0\t0\b8F¢\u0006\u0006\u001a\u0004\b\u000b\u0010\fR\u0011\u0010\r\u001a\u00020\u000e8F¢\u0006\u0006\u001a\u0004\b\u000f\u0010\u0010R\u0011\u0010\u0011\u001a\u00020\u000e8F¢\u0006\u0006\u001a\u0004\b\u0012\u0010\u0010R\u0011\u0010\u0013\u001a\u00020\u000e8F¢\u0006\u0006\u001a\u0004\b\u0014\u0010\u0010R\u0011\u0010\u0015\u001a\u00020\u000e8F¢\u0006\u0006\u001a\u0004\b\u0016\u0010\u0010R*\u0010\u001a\u001a\b\u0012\u0004\u0012\u00020\u00190\u00182\f\u0010\u0017\u001a\b\u0012\u0004\u0012\u00020\u00190\u0018@BX\u0086\u000e¢\u0006\b\n\u0000\u001a\u0004\b\u001b\u0010\u001cR\u0013\u0010\u001d\u001a\u0004\u0018\u00010\u00198F¢\u0006\u0006\u001a\u0004\b\u001e\u0010\u001fR\u0013\u0010 \u001a\u0004\u0018\u00010!8F¢\u0006\u0006\u001a\u0004\b\"\u0010#R\u0013\u0010$\u001a\u0004\u0018\u00010%8F¢\u0006\u0006\u001a\u0004\b&\u0010'Ê\u0001\u0002\b,¨\u0006+"},
   d2 = {"Lfrc/robot/subsystems/drivetrain/Vision;", "Ljava/lang/AutoCloseable;", "Lkotlin/AutoCloseable;", "<init>", "()V", "camera", "Lorg/photonvision/PhotonCamera;", "cameraMatrix", "Ledu/wpi/first/math/Matrix;", "Ledu/wpi/first/math/numbers/N3;", "kotlin.jvm.PlatformType", "getCameraMatrix", "()Ledu/wpi/first/math/Matrix;", "fx", "", "getFx", "()D", "fy", "getFy", "cx", "getCx", "cy", "getCy", "value", "", "Lorg/photonvision/targeting/PhotonPipelineResult;", "results", "getResults", "()Ljava/util/List;", "latestResult", "getLatestResult", "()Lorg/photonvision/targeting/PhotonPipelineResult;", "bestTarget", "Lorg/photonvision/targeting/PhotonTrackedTarget;", "getBestTarget", "()Lorg/photonvision/targeting/PhotonTrackedTarget;", "bestTargetPose", "Ledu/wpi/first/math/geometry/Translation2d;", "getBestTargetPose", "()Ledu/wpi/first/math/geometry/Translation2d;", "update", "", "close", "308-OCCRA-2026-main", "Ledu/wpi/first/epilogue/Logged;"}
)
public final class Vision implements AutoCloseable {
   @NotNull
   private final PhotonCamera camera = new PhotonCamera("Intake");
   @NotNull
   private List<? extends PhotonPipelineResult> results = CollectionsKt.emptyList();

   public Vision() {
   }

   @NotNull
   public final Matrix<N3, N3> getCameraMatrix() {
      Optional var10000 = this.camera.getCameraMatrix();
      Intrinsics.checkNotNullExpressionValue(var10000, "getCameraMatrix(...)");
      Matrix var1 = (Matrix)OptionalsKt.getOrNull(var10000);
      if (var1 == null) {
         var1 = new Matrix(Nat.N3(), Nat.N3());
      }

      return var1;
   }

   public final double getFx() {
      return this.getCameraMatrix().get(0, 0);
   }

   public final double getFy() {
      return this.getCameraMatrix().get(1, 1);
   }

   public final double getCx() {
      return this.getCameraMatrix().get(0, 2);
   }

   public final double getCy() {
      return this.getCameraMatrix().get(1, 2);
   }

   @NotNull
   public final List<PhotonPipelineResult> getResults() {
      return this.results;
   }

   @Nullable
   public final PhotonPipelineResult getLatestResult() {
      return (PhotonPipelineResult)CollectionsKt.lastOrNull(this.results);
   }

   @Nullable
   public final PhotonTrackedTarget getBestTarget() {
      PhotonPipelineResult var10000 = this.getLatestResult();
      PhotonTrackedTarget var2;
      if (var10000 != null ? var10000.hasTargets() : false) {
         PhotonPipelineResult var1 = this.getLatestResult();
         var2 = var1 != null ? var1.getBestTarget() : null;
      } else {
         var2 = null;
      }

      return var2;
   }

   @Nullable
   public final Translation2d getBestTargetPose() {
      PhotonTrackedTarget var10000 = this.getBestTarget();
      if (var10000 == null) {
         return null;
      } else {
         PhotonTrackedTarget target = var10000;
         return PhotonUtils.estimateCameraToTargetTranslation(PhotonUtils.calculateDistanceToTargetMeters((double)0.0F, (double)4.75F, (double)0.0F, Units.degreesToRadians(target.pitch)), Rotation2d.fromDegrees(target.yaw));
      }
   }

   public final void update() {
      List var10001 = this.camera.getAllUnreadResults();
      Intrinsics.checkNotNullExpressionValue(var10001, "getAllUnreadResults(...)");
      this.results = var10001;
   }

   public void close() {
      this.camera.close();
   }
}
