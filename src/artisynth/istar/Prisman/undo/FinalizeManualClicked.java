package artisynth.istar.Prisman.undo;

import java.awt.event.ActionEvent;
import java.util.ArrayList;

import javax.swing.AbstractAction;

import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.RigidBody;
import artisynth.istar.Prisman.HelperMathFunctions;
import artisynth.istar.Prisman.HelperMeshFunctions;
import artisynth.istar.Prisman.ReconstructionModel;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.PolygonalMesh;

public class FinalizeManualClicked extends AbstractAction {
   ReconstructionModel root = null;
   MechModel mechModel = null;
   artisynth.istar.Assist.Assist Assist = new artisynth.istar.Assist.Assist ();
   HelperMeshFunctions meshHelper = new HelperMeshFunctions();
   HelperMathFunctions mathHelper = new HelperMathFunctions();
   double IMPLANT_HEIGHT_MUSTHAVE = 7.0;
   
   public FinalizeManualClicked (ReconstructionModel root, MechModel model) {
      this.root = root;
      this.mechModel = model;
   }
   
   @Override
   public void actionPerformed (ActionEvent e) {
      PolygonalMesh leftMand = (PolygonalMesh) Assist.GetRigidBody (mechModel, "Mandible Left").getSurfaceMesh ();
      PolygonalMesh rightMand = (PolygonalMesh) Assist.GetRigidBody (mechModel, "Mandible Right").getSurfaceMesh ();
      SurfaceMeshIntersector intersector = new SurfaceMeshIntersector ();
      PolygonalMesh recon = new PolygonalMesh ();
      PolygonalMesh segmentsOnly = new PolygonalMesh ();
      ArrayList<PolygonalMesh> donorSegMeshes = new ArrayList<PolygonalMesh> ();
      
      recon = intersector.findUnion (recon, leftMand);
      
      for (MeshComponent meshes : mechModel.meshBodies ()) {
         if (meshes.getName ().contains ("Clipped Donor ")) {
            PolygonalMesh segment = (PolygonalMesh) meshes.getMesh ();
            recon = intersector.findUnion (recon, segment);
            segmentsOnly = intersector.findUnion (segmentsOnly, segment); 
            donorSegMeshes.add (segment);
         }
      }
      
      recon = intersector.findUnion (recon, rightMand);
      
      RigidBody reconMand = new RigidBody ("Reconstructed Mandible");
      reconMand.setSurfaceMesh (recon);
      mechModel.addRigidBody (reconMand);
      
      System.out.println (root.cuttingPlanes.size ());
      
      if (!meshHelper.getImplantList(mechModel).isEmpty ()) {
         ArrayList<PolygonalMesh> implantList = meshHelper.getImplantList (mechModel);
         for (int i = 0; i < implantList.size (); i++) {
            PolygonalMesh implant = implantList.get (i);
            if (intersector.findUnion (recon, implant).numDisconnectedVertices () > 0) {
               System.out.println ("Implant number " + i + " is not implantable");
               continue;
            }
         
            double volume = intersector.findIntersection (recon, implant).computeVolume ();
            double height = volume / (4 * Math.PI);
            if ((height / (4 * Math.PI)) < IMPLANT_HEIGHT_MUSTHAVE) {
               System.out.println ("Implant number " + i + " is not implantable");
               continue;
            }
         }
      }
      
      PolygonalMesh resectedPiece = Assist.GetMesh (mechModel, "Resection").copy ();
      //calculate volume overlap
      double volOverlap = meshHelper.getVolumeOverlap (resectedPiece, segmentsOnly);
      
      //calculate HD 95
      double hd95 = mathHelper.get3DHausdorff95 (segmentsOnly, resectedPiece);
      
      //calculate bony contact
//      PolygonalMesh fullMand = Assist.GetMesh (mechModel, "BackupMandible").copy ();
//      double bonyContact = meshHelper.getBonyContact 
//         (leftMand, rightMand, resectedPiece, fullMand, donorSegMeshes, root.cuttingPlanes, mechModel);
//      
//      System.out.println ("Average Bony Contact: " + bonyContact + "%");
      System.out.println ("Volume overlap: " + volOverlap + "%");
      System.out.println ("Hausdorff 95th Percentile: " + hd95 + "mm");
      
   }

}