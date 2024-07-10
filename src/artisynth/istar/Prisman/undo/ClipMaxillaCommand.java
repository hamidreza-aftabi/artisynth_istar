package artisynth.istar.Prisman.undo;

import java.awt.Color;
import java.util.ArrayList;

import artisynth.core.gui.editorManager.Command;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.demos.test.IntersectionTester;
import artisynth.istar.Assist.Assist;
import artisynth.istar.Prisman.ReconstructionModel;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AffineTransform3dBase;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;

public class ClipMaxillaCommand implements Command {
   final int BOXSIZE = 500;
   String myName = "ClipCommand";
   Assist Assist = new Assist ();
   MechModel mechModel = null;
   ReconstructionModel root = null;
   PolygonalMesh BackupMandible = new PolygonalMesh ();

   public ClipMaxillaCommand (ReconstructionModel root, MechModel mechModel) {
      this.mechModel = mechModel;
      this.root = root;
      // this.BackupMandible = Assist.GetMesh (mechModel, "Mandible").copy ();
   }

   public void execute () {
      // Check if the operation has already completed
      if (Assist.GetRigidBody (mechModel, "Resection") != null) {
         System.out.println ("The maxilla has already been cut");
      }
      else {
         // get all the planes in the scene
         ArrayList<FixedMeshBody> planeBodies = new ArrayList<FixedMeshBody> ();
         for (MeshComponent meshcomp : mechModel.meshBodies ()) {
            if (meshcomp.getName () != null
            && meshcomp.getName ().contains ("plane")) {
               planeBodies.add ((FixedMeshBody)meshcomp);
            }
         }
         // convert planes to boxes which we will intersect
         int x = 0;
         for (FixedMeshBody Body : planeBodies) {
            PolygonalMesh testSquareMesh =
               (PolygonalMesh)MeshFactory.createBox (BOXSIZE, BOXSIZE, BOXSIZE);
            FixedMeshBody boxBody = new FixedMeshBody (testSquareMesh);
            boxBody.setName ("Box" + x);
            boxBody.transformGeometry (Body.getMeshToWorld ());
            RigidTransform3d TBW = new RigidTransform3d (boxBody.getPose ());
            RigidTransform3d TDZ = new RigidTransform3d (0, 0, BOXSIZE / 2);
            TBW.mul (TDZ);
            boxBody.setPose (TBW);
            mechModel.addMeshBody (boxBody);
//            Body.getRenderProps ().setVisible (false);
            ++x;
         }
         // get all boxes
         ArrayList<FixedMeshBody> BoxBodies = new ArrayList<FixedMeshBody> ();
         for (MeshComponent meshcomp : mechModel.meshBodies ()) {
            if (meshcomp.getName () != null
            && meshcomp.getName ().contains ("Box")) {
               BoxBodies.add ((FixedMeshBody)meshcomp);
            }
         }
         // create initial body for intersection of all boxes
         FixedMeshBody intersect = new FixedMeshBody ();
         intersect.setName ("Intersect");
         PolygonalMesh intersectMesh =
            new PolygonalMesh (
               MeshFactory
                  .createBox (BOXSIZE * 100, BOXSIZE * 100, BOXSIZE * 100));
         for (FixedMeshBody Body : BoxBodies) {
            intersectMesh =
               MeshFactory
                  .getIntersection (
                     intersectMesh, (PolygonalMesh)Body.getMesh ());
            Body.getRenderProps ().setVisible (false);
         }
         intersect.setMesh (intersectMesh);
         mechModel.addMeshBody (intersect);
         // intersect.getRenderProps ()
         // create resection area
         FixedMeshBody resection =
            new FixedMeshBody (
               MeshFactory
                  .getIntersection (
                     Assist.GetMesh (mechModel, "Maxilla"),
                     Assist.GetMesh (mechModel, "Intersect")));
         resection.setName ("Resection");
         mechModel.addMeshBody (resection);
         resection.getRenderProps ().setVisible (false);
         // Create Clipped Maxilla
         RigidBody clippedMaxilla = new RigidBody ();
         clippedMaxilla
            .setSurfaceMesh (
               MeshFactory
                  .getSubtraction (
                     Assist.GetMesh (mechModel, "Maxilla"),
                     Assist.GetMesh (mechModel, "Intersect")));
         clippedMaxilla.setName ("ClippedMaxilla");
         mechModel.addRigidBody (clippedMaxilla);
         Assist
            .GetRigidBody (mechModel, "Maxilla").getRenderProps ()
            .setVisible (false);
      }
      Assist
         .GetMeshBody (mechModel, "Intersect").getRenderProps ()
         .setVisible (false);
      if (root.donorIsScapulaCheckBox.isSelected ()) {
         RenderProps.setWireFrame (Assist.GetMeshBody (mechModel, "Donor"), true);
         // setup intersection mesh using an intersection tester
         root.Intersection =
            new IntersectionTester (
               Assist.GetMesh (mechModel, "Intersect"),
               Assist.GetMesh (mechModel, "Donor"), 0);
         root.Intersection
            .setCSGOperation (SurfaceMeshIntersector.CSG.INTERSECTION);
         root.Intersection.setRenderCSGMesh (true);
         root.addController (root.Intersection);
      }
   }

   public void undo () {
      mechModel
         .removeRigidBody (Assist.GetRigidBody (mechModel, "ClippedMaxilla"));
      mechModel.removeMeshBody (Assist.GetMeshBody (mechModel, "Resection"));
      mechModel.removeMeshBody (Assist.GetMeshBody (mechModel, "Intersect"));
      Assist
         .GetRigidBody (mechModel, "Maxilla").getRenderProps ()
         .setVisible (true);
      ArrayList<MeshComponent> boxes = new ArrayList<> ();
      for (MeshComponent mesh : mechModel.meshBodies ()) {
         if (mesh.getName ().contains ("plane")) {
            mesh.getRenderProps ().setVisible (true);
         }
         else if (mesh.getName ().contains ("Box")) {
            boxes.add (mesh);
         }
      }
      for (MeshComponent mesh : boxes) {
         mechModel.removeMeshBody (mesh);
      }
   }

   public String getName () {
      return myName;
   }
}