package artisynth.istar.Prisman.undo;

import java.awt.event.ActionEvent;

import javax.swing.AbstractAction;

import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.istar.Assist.Assist;
import artisynth.istar.Prisman.HelperMeshFunctions;
import artisynth.istar.Prisman.ReconstructionModel;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;

public class FinalCutButtonClicked extends AbstractAction {
   ReconstructionModel root = null;
   MechModel mechModel = null;
   Assist Assist = new Assist ();
   static int PLANE_LENGTH = 100;
   HelperMeshFunctions meshHelper = new HelperMeshFunctions ();
   
   public FinalCutButtonClicked (ReconstructionModel root, MechModel model) {
      this.root = root;
      this.mechModel = model;
   }
   
   @Override
   public void actionPerformed (ActionEvent e) {
      FixedMeshBody plane1MeshBody = Assist.GetMeshBody (mechModel, "Plane2");
      PolygonalMesh plane1Mesh = (PolygonalMesh)plane1MeshBody.getMesh ();
      
      Point3d c1 = new Point3d ();
      plane1Mesh.computeCentroid (c1);
      Vector3d planenormal1 = new Vector3d (plane1Mesh.getNormal (0));
      RigidTransform3d pose = plane1MeshBody.getPose ();
      Point3d centroid1 = new Point3d (c1);
      centroid1.transform (pose);
      planenormal1.transform (pose);
      
      PolygonalMesh cutDonorPlane1 = MeshFactory.createPlane (PLANE_LENGTH, PLANE_LENGTH);
      meshHelper.createPlane 
         (cutDonorPlane1.getNormal (0), new Vector3d (planenormal1), centroid1, pose, cutDonorPlane1);

      PolygonalMesh donorSeg = new PolygonalMesh ();
      
      int j = 0;
      for (int i = 0; i < mechModel.meshBodies ().size (); i++) {
         char[] c = new char[mechModel.meshBodies().get (i).getName ().length ()];
         c = mechModel.meshBodies().get (i).getName ().toCharArray ();
         if (c[mechModel.meshBodies().get (i).getName ().length () - 1] > j) {
            j = c[mechModel.meshBodies().get (i).getName ().length () - 1];
            System.out.println (c[mechModel.meshBodies().get (i).getName ().length () - 1]);
            System.out.println (j);
         }
//         
//         if (mechModel.meshBodies().get (i).getName ().contains ("Negative Segment " + j)) {
//            j++;
//         }
      }
      
//      System.out.println 
//         (mechModel.meshBodies ().
//            contains (Assist.GetMeshBody (mechModel, "Negative Segment " + (j - 2))));
      
      donorSeg = (PolygonalMesh) Assist.GetMeshBody (mechModel, "Negative Segment " + (j - 1)).getMesh ().copy ();
      
      Assist.GetMeshBody (mechModel, "Negative Segment " + (j - 1)).getRenderProps ().setVisible (false);
      
      FixedMeshBody donorPlane1 = new FixedMeshBody (cutDonorPlane1);
      donorPlane1.setName ("Donor Plane " + (j - 1));
      mechModel.addMeshBody (donorPlane1);
      
      FixedMeshBody addNewSeg = new FixedMeshBody (donorSeg);
      addNewSeg.setName ("Donor Segment " + (j - 1));
      mechModel.addMeshBody (addNewSeg);
      
   }
   
   

}

