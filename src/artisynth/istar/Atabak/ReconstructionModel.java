package artisynth.istar.Atabak;

import java.util.ArrayList;
import java.util.List;

import javax.swing.JCheckBox;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;

import artisynth.demos.test.IntersectionTester;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.workspace.RootModel;
import maspack.geometry.PolygonalMesh;
import maspack.interpolation.NumericList;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

public class ReconstructionModel extends RootModel {
   public Vector3d TranslateToCentroidMandible = new Vector3d ();
   public ArrayList<PolygonalMesh> implantsSeperated =
      new ArrayList<PolygonalMesh> ();
   public Vector3d mandibleCentroidOriginal;
   public List<Point3d> frameMarkerPos = new ArrayList<Point3d> ();
   public List<FrameMarker> fm = new ArrayList<FrameMarker> ();
   public PolygonalMesh scapulaTrimPlane;
   public JCheckBox donorIsScapulaCheckBox;
   public NumericList plateNumericList = new NumericList (3);
   public boolean DEBUG = true;
   public int numberOfSegments = 1;
   public NumericList simpList;
   public VectorNd DonorLengthDir = new VectorNd ();
   public List<RigidTransform3d> transforms = new ArrayList<RigidTransform3d> ();
   public List<AffineTransform3d> translateBack = new ArrayList<AffineTransform3d> ();
   public VectorNd fromCenterToSurface = new VectorNd ();
   public ArrayList<PolygonalMesh> fibulaScrews = new ArrayList<PolygonalMesh> ();
   public ArrayList<FixedMeshBody> fibulaScrewsBodies =
      new ArrayList<FixedMeshBody> ();
   public ArrayList<Integer> fibulaScrewsIndices = new ArrayList<Integer> ();
   public Point3d topmax = new Point3d ();
   public Point3d botmax = new Point3d ();
   public Point3d topPoint = new Point3d ();
   public List<PolygonalMesh> cuttingPlanes = new ArrayList<PolygonalMesh> ();
   public List<FixedMeshBody> cuttingPlanesMeshBodies = new ArrayList<FixedMeshBody> ();
   public JTextArea log = new JTextArea(5, 20);
   public JScrollPane scrollPane = new JScrollPane(log); 
   public IntersectionTester Intersection;
   public ArrayList<FixedMeshBody> activePlaneList = new ArrayList<> ();
   public ReconstructionModel () {
      log.setEditable(false);
   }
}