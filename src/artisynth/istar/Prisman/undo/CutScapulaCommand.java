package artisynth.istar.Prisman.undo;

import java.util.List;
import artisynth.core.mechmodels.*;
import maspack.geometry.*;
import maspack.matrix.*;
import artisynth.core.gui.editorManager.Command;

public class CutScapulaCommand implements Command {
   private String myName;
   private MechModel myModel;
   private int[] num;
   private List<FixedMeshBody> scapulasegmented;
   private List<PolygonalMesh> cp;
   private RigidTransform3d t;
   private List<PolygonalMesh> sc;
   private List<FixedMeshBody> scBody;
   private int max;
   private FixedMeshBody db;
   private List<PolygonalMesh> scp1;
   private List<FixedMeshBody> scp1b;
   private PolygonalMesh d;

   public CutScapulaCommand (String name, MechModel model,
   List<FixedMeshBody> segment, int[] CutNumber,
   List<PolygonalMesh> cuttingPlanes, RigidTransform3d tr,
   List<PolygonalMesh> scapulaClone, List<FixedMeshBody> scapulaCloneBody,
   int numberOfSegments, PolygonalMesh Donor, FixedMeshBody DonorBody,
   List<PolygonalMesh> plane1clones, List<FixedMeshBody> plane1clonesbody) {
      myName = name;
      myModel = model;
      scapulasegmented = segment;
      num = CutNumber;
      cp = cuttingPlanes;
      t = tr;
      sc = scapulaClone;
      scBody = scapulaCloneBody;
      max = numberOfSegments;
      db = DonorBody;
      scp1 = plane1clones;
      scp1b = plane1clonesbody;
      d = Donor;
   }

   public void execute () {
      myModel.addMeshBody (scapulasegmented.get (num[0]));
      cp.get (2 * num[0]).transform (t);
      cp.get (2 * num[0] + 1).transform (t);
      scp1.get (num[0]).transform (t);
      myModel.addMeshBody (scp1b.get (num[0]));
      PolygonalMesh reversePlane =
         new PolygonalMesh (cp.get (2 * num[0] + 1).clone ());
      reversePlane.flip ();

      myModel.removeMeshBody (scBody.get (num[0]));
      sc.add (num[0] + 1, new PolygonalMesh (sc.get (num[0])));
      sc.get (num[0] + 1).transform (t);
      sc.set (
         num[0] + 1,
         MeshFactory.getSubtraction (sc.get (num[0] + 1), reversePlane));
      scBody.add (
         num[0] + 1, new FixedMeshBody ("scapulaClone", sc.get (num[0] + 1)));
      myModel.addMeshBody (scBody.get (num[0] + 1));

      num[0]++;

      if (num[0] == max) {
         scBody.get (num[0]).getRenderProps ().setVisible (false);
         db.getRenderProps ().setVisible (true);
      }

      // Testing if transformed planes are correct
      // SurfaceMeshIntersector intersector = new SurfaceMeshIntersector ();
      // PolygonalMesh segment = new PolygonalMesh(d);
      // segment = intersector.findDifference01 (segment.clone(), cp.get
      // (2*(num[0]-1)).clone());
      // segment = intersector.findDifference01 (segment.clone(), cp.get
      // (2*(num[0]-1)+1).clone());
      // segment = intersector.findDifference01 (segment.clone(), scp1.get
      // (2*(num[0]-1)).clone());
      // segment.inverseTransform(t);
      // FixedMeshBody segmentBody = new FixedMeshBody
      // ("TestSegment"+String.valueOf (num[0]-1), segment);
      // myModel.addMeshBody(segmentBody);
   }

   public void undo () {
      num[0]--;
      if (num[0] == max - 1) {
         scBody.get (num[0]).getRenderProps ().setVisible (true);
         db.getRenderProps ().setVisible (false);
      }

      myModel.removeMeshBody (scapulasegmented.get (num[0]));
      cp.get (2 * num[0]).inverseTransform (t);
      cp.get (2 * num[0] + 1).inverseTransform (t);

      myModel.removeMeshBody (scBody.get (num[0] + 1));
      sc.remove (num[0] + 1);
      myModel.addMeshBody (scBody.get (num[0]));

      myModel.removeMeshBody (scp1b.get (num[0]));
      scp1.remove (num[0]);
      scp1b.remove (num[0]);
   }

   public String getName () {
      return myName;
   }
}
