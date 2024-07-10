package artisynth.istar.MandibleRecon;

import java.awt.event.ActionEvent;

import javax.swing.AbstractAction;

import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.MeshFactory;
import artisynth.istar.Assist.Assist;
import artisynth.istar.Prisman.ReconstructionModel;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;

public class prepManualCutButtonClicked extends AbstractAction {
      RigidBody DonorBody;
      RigidBody mandibleCuttingPlane1;
      RigidBody mandibleCuttingPlane2;
      RigidBody cutDonorPlane1Body;
      PolygonalMesh cutDonorPlane1;
      Assist Assist = new Assist();
      Point3d centerCutDonorPlane1 = new Point3d ();
      Vector3d normalCutDonorPlane1 = new Vector3d ();
      RigidBody scapulaMeshOutlineRigidBody;
      RigidBody donorClipped;
      MechModel mechModel = null;
      ReconstructionModel root = null;
      SurfaceMeshIntersector.CSG csg = SurfaceMeshIntersector.CSG.DIFFERENCE10;
      SurfaceMeshIntersector.CSG csg2 = SurfaceMeshIntersector.CSG.DIFFERENCE01;
      Vector3d originalClippedDonorCentroid;
      RigidTransform3d scapulaTrimPlaneBodyPose;
      static int PLANE_LENGTH = 100;

      public prepManualCutButtonClicked (ReconstructionModel root, MechModel model) {
         this.root = root;
         this.mechModel = model;
         putValue (NAME, "setUpManualReconButton");
      }

      @Override
      public void actionPerformed (ActionEvent evt) {

         cutDonorPlane1 = MeshFactory.createPlane (PLANE_LENGTH, PLANE_LENGTH);
         maspack.matrix.RigidTransform3d transformCutDonorPlane =
            new maspack.matrix.RigidTransform3d ();
         Vector3d mandibleCentroid = new Vector3d ();
         Assist.GetMesh (mechModel, "Mandible").computeCentroid (mandibleCentroid);
         transformCutDonorPlane.setTranslation (mandibleCentroid);
         cutDonorPlane1.transform (transformCutDonorPlane);
         cutDonorPlane1.translateToCentroid ();
         cutDonorPlane1Body = new RigidBody ();
         cutDonorPlane1Body.setSurfaceMesh (cutDonorPlane1);
         cutDonorPlane1Body.setName ("cutDonorPlane1Body");
         DonorBody = new RigidBody ();
         DonorBody.addMesh (Assist.GetMesh (mechModel, "Donor"));
         DonorBody.setName ("DonorRigidBody");
         mechModel.add (DonorBody);
         Assist.GetMeshBody (mechModel, "Donor").getRenderProps ().setVisible (false);
         mechModel.addRigidBody (cutDonorPlane1Body);
      }
}