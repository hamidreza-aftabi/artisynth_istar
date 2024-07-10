package artisynth.istar.Prisman;

import java.awt.event.ActionEvent;
import java.util.ArrayList;
import java.util.List;

import javax.swing.AbstractAction;

import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.RigidBody;
import artisynth.istar.Assist.Assist;
import artisynth.istar.Prisman.HelperMandibleFunctions;
import artisynth.istar.Prisman.HelperMeshFunctions;
import artisynth.istar.Prisman.ReconstructionModel;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;

public class ManualSegmentClicked extends AbstractAction {
   ReconstructionModel root = null;
   MechModel mechModel = null;
   int count;
   Assist Assist = new Assist ();
   static int PLANE_LENGTH = 100;
   HelperMeshFunctions meshHelper = new HelperMeshFunctions ();
   
   public ManualSegmentClicked (ReconstructionModel root, MechModel model) {
      this.root = root;
      this.mechModel = model;
      count = 0;
   }
   
   @Override
   public void actionPerformed (ActionEvent e) {      
      PolygonalMesh donorSeg = new PolygonalMesh ();
      
      if (count == 0) {
         FixedMeshBody plane1MeshBody = Assist.GetMeshBody (mechModel, "Plane1");
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
         
         FixedMeshBody donorPlane1 = new FixedMeshBody (cutDonorPlane1);
         donorPlane1.setName ("Donor Plane 0");
         mechModel.addMeshBody (donorPlane1);
         count++;
         
      } else {
         PolygonalMesh addPlane = MeshFactory.createPlane (90, 90, 10, 10);
         FixedMeshBody addPlaneBody = new FixedMeshBody (addPlane);
         addPlaneBody.setName ("Donor Plane " + count);
         mechModel.addMeshBody (addPlaneBody);
         count++;
      }
      
//      
//      
//      if (count == 0) {
//         RigidBody clippedDonor = Assist.GetRigidBody (mechModel, "ClippedDonor");
//         donorSeg = clippedDonor.getSurfaceMesh ().copy ();
//                  
//         FixedMeshBody plane1MeshBody = Assist.GetMeshBody (mechModel, "Plane1");
//         PolygonalMesh plane1Mesh = (PolygonalMesh)plane1MeshBody.getMesh ();
//         
//         Point3d c1 = new Point3d ();
//         plane1Mesh.computeCentroid (c1);
//         Vector3d planenormal1 = new Vector3d (plane1Mesh.getNormal (0));
//         RigidTransform3d pose = plane1MeshBody.getPose ();
//         Point3d centroid1 = new Point3d (c1);
//         centroid1.transform (pose);
//         planenormal1.transform (pose);
//         
//         PolygonalMesh cutDonorPlane1 = MeshFactory.createPlane (PLANE_LENGTH, PLANE_LENGTH);
//         meshHelper.createPlane 
//            (cutDonorPlane1.getNormal (0), new Vector3d (planenormal1), centroid1, pose, cutDonorPlane1);
//         
//         FixedMeshBody donorPlane1 = new FixedMeshBody (cutDonorPlane1);
//         donorPlane1.setName ("Donor Plane 0");
//         mechModel.addMeshBody (donorPlane1);
//         
//         Assist.GetRigidBody (mechModel, "ClippedDonor").getRenderProps ().setVisible (false);
//                  
//         count++;
//         
//      } else {
//         PolygonalMesh addPlane = MeshFactory.createPlane (90, 90, 10, 10);
//         FixedMeshBody addPlaneBody = new FixedMeshBody (addPlane);
//         addPlaneBody.setName ("Donor Plane " + count);
//         mechModel.addMeshBody (addPlaneBody);
//         
//         System.out.println(mechModel.meshBodies ().contains (Assist.GetMeshBody (mechModel, "Negative Segment " + (count - 1))));
//
//         donorSeg = (PolygonalMesh) Assist.GetMeshBody (mechModel, "Negative Segment " + (count - 1)).getMesh ().copy ();
//         
//         Assist.GetMeshBody (mechModel, "Negative Segment " + (count - 1)).getRenderProps ().setVisible (false);
//
//         count++;
//      }
//      
//      FixedMeshBody addNewSeg = new FixedMeshBody (donorSeg);
//      addNewSeg.setName ("Donor Segment " + (count - 1));
//      mechModel.addMeshBody (addNewSeg);
      
   }

}