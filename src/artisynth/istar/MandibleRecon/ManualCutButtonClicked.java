package artisynth.istar.MandibleRecon;

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

public class ManualCutButtonClicked extends AbstractAction {
      int count;
      RigidBody DonorBody;
      RigidBody mandibleCuttingPlane1;
      RigidBody mandibleCuttingPlane2;
      RigidBody cutDonorPlane1Body;
      PolygonalMesh cutDonorPlane1;
      Assist Assist = new Assist ();
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
      boolean flag1 = false;
      boolean flag2 = false;
      SurfaceMeshIntersector intersector = new SurfaceMeshIntersector ();
      HelperMandibleFunctions mandHelp = new HelperMandibleFunctions ();
      HelperMeshFunctions meshHelper = new HelperMeshFunctions ();
      FixedMeshBody prevPlane = new FixedMeshBody ();
      
      public ManualCutButtonClicked (ReconstructionModel root, MechModel model) {
         count = 0;
         this.root = root;
         this.mechModel = model;
      }

      @Override
      public void actionPerformed (ActionEvent evt) {
         //create segment by differencing
         //leave segment in model
         //need to link planes to clipped donor mesh so they move with it. 
         PolygonalMesh donorCopy = new PolygonalMesh();
         
         if (count == 0) {
            donorCopy = Assist.GetMesh (mechModel, "ClippedDonor");
         } else {
            donorCopy = Assist.GetMesh (mechModel, "Clipped Donor" + (count - 2));
         }
         
         FixedMeshBody copyMesh = Assist.GetMeshBody (mechModel, "ClippedDonor");
         
         FixedMeshBody planeMesh = new FixedMeshBody();
         
         PolygonalMesh segment = new PolygonalMesh ();
         if (count == 0) {
             planeMesh = Assist.GetMeshBody (mechModel, "Donor Plane " + count);
         } else {
             planeMesh = Assist.GetMeshBody (mechModel, "Donor Plane " + (count - 1));
         }
         System.out.println (count);
         PolygonalMesh plane = (PolygonalMesh) planeMesh.getMesh ();
         
         Point3d c = new Point3d ();
         plane.computeCentroid (c);
         Vector3d planenormal = new Vector3d (plane.getNormal (0));
         RigidTransform3d pose = planeMesh.getPose ();
         Point3d centroid = new Point3d (c);
         centroid.transform (pose);
         planenormal.transform (pose);
         
         PolygonalMesh donorPlane = MeshFactory.createPlane (PLANE_LENGTH, PLANE_LENGTH);
         meshHelper.createPlane 
            (donorPlane.getNormal (0), new Vector3d (planenormal), centroid, pose, donorPlane);
         FixedMeshBody donorPlaneMesh = new FixedMeshBody (donorPlane);
         
         SurfaceMeshIntersector intersector = new SurfaceMeshIntersector ();
         
//         PolygonalMesh negativePiece = 
//            intersector.findDifference01 (donorCopy, donorPlane);
         
         if (count == 0) {
            //donorPlane.flip();
            PolygonalMesh positivePiece =
               intersector.findDifference01 (donorCopy, donorPlane);
            
            FixedMeshBody ClippedDonor = new FixedMeshBody (positivePiece);
            ClippedDonor.setName ("Clipped Donor " + count);
            mechModel.addMeshBody (ClippedDonor);
         }
         
         if (count >= 1) {
            donorPlane.flip();
            PolygonalMesh positivePiece =
               intersector.findDifference01 (donorCopy, donorPlane);
            
            PolygonalMesh prev = (PolygonalMesh) prevPlane.getMesh ();
            prev.flip ();
            positivePiece =
               intersector.findDifference01 (positivePiece, prev);
            
            FixedMeshBody ClippedDonor = new FixedMeshBody (positivePiece);
            ClippedDonor.setName ("Clipped Donor " + count);
            mechModel.addMeshBody (ClippedDonor);
         }
         
         
         
         donorCopy.addMesh (donorPlane, true);
         FixedMeshBody ncd = new FixedMeshBody (donorCopy);
         ncd.setName ("New Clipped Donor" + count);
         mechModel.addMeshBody (ncd);
         
         if (count == 0) {
            //Assist.GetRigidBody (mechModel, "ClippedDonor").getRenderProps ().setVisible (false);
         } else {
            Assist.GetMeshBody (mechModel, "New Clipped Donor " + (count - 1)).getRenderProps ().setVisible (false);
         }
         
         count++;

         
//         //get "Donor Segment " + count 
//         //create new plane equal to "Donor Plane " + count
//         //find difference between them
//         PolygonalMesh donorSeg = Assist.GetMesh (mechModel, "Donor Segment " + count);
//         FixedMeshBody donorSegMesh = Assist.GetMeshBody (mechModel, "Donor Segment " + count);
//         
//         FixedMeshBody planeMesh = Assist.GetMeshBody (mechModel, "Donor Plane " + count);
//         PolygonalMesh plane = (PolygonalMesh) planeMesh.getMesh ();
//         
//         Point3d c = new Point3d ();
//         plane.computeCentroid (c);
//         Vector3d planenormal = new Vector3d (plane.getNormal (0));
//         RigidTransform3d pose = planeMesh.getPose ();
//         Point3d centroid = new Point3d (c);
//         centroid.transform (pose);
//         planenormal.transform (pose);
//         
//         PolygonalMesh donorPlane = MeshFactory.createPlane (PLANE_LENGTH, PLANE_LENGTH);
//         meshHelper.createPlane 
//            (donorPlane.getNormal (0), new Vector3d (planenormal), centroid, pose, donorPlane);
//         FixedMeshBody donorPlaneMesh = new FixedMeshBody (donorPlane);
//         
//         root.cuttingPlanes.add (donorPlane);
//         root.cuttingPlanesMeshBodies.add (donorPlaneMesh);
//         
//         SurfaceMeshIntersector intersector = new SurfaceMeshIntersector ();
//         
//         PolygonalMesh negativePiece = 
//            intersector.findDifference01 (donorSeg, donorPlane);
//         
//         if (count == 1) {
//            donorPlane.flip();
//            PolygonalMesh positivePiece =
//               intersector.findDifference01 (donorSeg, donorPlane);
//            
//            FixedMeshBody ClippedDonor = new FixedMeshBody (positivePiece);
//            ClippedDonor.setName ("Clipped Donor " + count);
//            mechModel.addMeshBody (ClippedDonor);
//         }
//         
//         if (count > 1) {
//            donorPlane.flip();
//            PolygonalMesh positivePiece =
//               intersector.findDifference01 (donorSeg, donorPlane);
//            
//            PolygonalMesh prev = (PolygonalMesh) prevPlane.getMesh ();
//            prev.flip ();
//            positivePiece =
//               intersector.findDifference01 (positivePiece, prev);
//            
//            FixedMeshBody ClippedDonor = new FixedMeshBody (positivePiece);
//            ClippedDonor.setName ("Clipped Donor " + count);
//            mechModel.addMeshBody (ClippedDonor);
//         }
//         
//         FixedMeshBody NegativeDonor = new FixedMeshBody (negativePiece);
//         NegativeDonor.setName ("Negative Segment " + count);
//         mechModel.addMeshBody (NegativeDonor);
//         
//
//         Assist.GetMeshBody (mechModel, "Donor Segment " + (count)).getRenderProps ().setVisible (false);

         
         
         
         
         
         
         
         
         
         
         
         
         
         
         
         
         
//         
//         if (count == 0) {            
//            FixedMeshBody plane1MeshBody = Assist.GetMeshBody (mechModel, "Plane1");
//            PolygonalMesh plane1Mesh = (PolygonalMesh)plane1MeshBody.getMesh ();
//            
//            Point3d c1 = new Point3d ();
//            plane1Mesh.computeCentroid (c1);
//            Vector3d planenormal1 = new Vector3d (plane1Mesh.getNormal (0));
//            RigidTransform3d pose = plane1MeshBody.getPose ();
//            Point3d centroid1 = new Point3d (c1);
//            centroid1.transform (pose);
//            planenormal1.transform (pose);
//            
//            cutDonorPlane1 = MeshFactory.createPlane (PLANE_LENGTH, PLANE_LENGTH);
//            meshHelper.createPlane 
//               (cutDonorPlane1.getNormal (0), new Vector3d (planenormal1), centroid1, pose, cutDonorPlane1);
//            
//            cutDonorPlane1Body = new RigidBody ("cutDonorPlane1Body");
//            cutDonorPlane1Body.setSurfaceMesh (cutDonorPlane1);
//            DonorBody = new RigidBody("DonorRigidBody");
//            DonorBody.setSurfaceMesh (Assist.GetMesh (mechModel, "Donor"));
//            mechModel.addRigidBody (DonorBody);
//            Assist.GetMeshBody (mechModel, "Donor").getRenderProps ().setVisible (false);
//            mechModel.addRigidBody (cutDonorPlane1Body);
//            
//            
//            
//            
////            cutDonorPlane1 = MeshFactory.createPlane (PLANE_LENGTH, PLANE_LENGTH);
////            maspack.matrix.RigidTransform3d transformCutDonorPlane =
////               new maspack.matrix.RigidTransform3d ();
////            Vector3d mandibleCentroid = new Vector3d ();
////            Assist.GetMesh (mechModel, "Mandible").computeCentroid (mandibleCentroid);
////            transformCutDonorPlane.setTranslation (mandibleCentroid);
////            cutDonorPlane1.transform (transformCutDonorPlane);
////            cutDonorPlane1.translateToCentroid ();
////            cutDonorPlane1Body = new RigidBody ();
////            cutDonorPlane1Body.setSurfaceMesh (cutDonorPlane1);
////            cutDonorPlane1Body.setName ("cutDonorPlane1Body");
////            DonorBody = new RigidBody ();
////            DonorBody.addMesh (Assist.GetMesh (mechModel, "Donor"));
////            DonorBody.setName ("DonorRigidBody");
////            mechModel.addRigidBody (DonorBody);
////            Assist.GetMeshBody (mechModel, "Donor").getRenderProps ().setVisible (false);
////            mechModel.addRigidBody (cutDonorPlane1Body);
//         }else {
////          cutDonorPlane1 = MeshFactory.createPlane (PLANE_LENGTH, PLANE_LENGTH);
////          maspack.matrix.RigidTransform3d transformCutDonorPlane =
////             new maspack.matrix.RigidTransform3d ();
////          Vector3d mandibleCentroid = new Vector3d ();
////          Assist.GetMesh (mechModel, "Mandible").computeCentroid (mandibleCentroid);
////          transformCutDonorPlane.setTranslation (mandibleCentroid);
////          cutDonorPlane1.transform (transformCutDonorPlane);
////          cutDonorPlane1.translateToCentroid ();
////          cutDonorPlane1Body = new RigidBody ();
////          cutDonorPlane1Body.setSurfaceMesh (cutDonorPlane1);
////          //cutDonorPlane1Body.setName ("cutDonorPlane1Body");
////          DonorBody = new RigidBody ();
////          DonorBody.addMesh (Assist.GetMesh (mechModel, "Donor"));
////         // DonorBody.setName ("DonorRigidBody");
////          mechModel.addRigidBody (DonorBody);
////          Assist.GetMeshBody (mechModel, "Donor").getRenderProps ().setVisible (false);
////          mechModel.addRigidBody (cutDonorPlane1Body);
////            
//            
//            ManualSplit (
//               mechModel, cutDonorPlane1Body, DonorBody, count);
//         }
         count = count + 1;
         prevPlane = new FixedMeshBody (donorPlane);
      }
      
      
      private void ManualSplit (
         MechModel mechModel, RigidBody Plane, RigidBody Donor, Integer count) {
         SurfaceMeshIntersector intersector =
            new SurfaceMeshIntersector ();
         PolygonalMesh segmentPlane = Plane.getSurfaceMesh ().clone ();
         
//         String homedir = ArtisynthPath.getHomeDir ();
//         File pathHome = new File (homedir);
//         String homeParent = pathHome.getParentFile ().getAbsolutePath ();
//         String path =
//            homeParent
//            + "/artisynth_projects/src/artisynth/models/Prisman/stl/horizontal.stl";
         Assist Assist = new Assist();
//         PolygonalMesh metalinsertlong = Assist.loadGeometry(path);
         segmentPlane.setName ("FibulaPlane" + count);
         RigidBody segmentPlaneBody = new RigidBody ();
         segmentPlaneBody.setName ("CuttingPlane" + count);
         segmentPlaneBody.setSurfaceMesh (segmentPlane);
         RigidTransform3d transform = new RigidTransform3d ();
         transform.setRotation (Plane.getRotation ());
         transform.R.mulRotY180 ();
         transform.setTranslation (Plane.getPosition ());
         segmentPlane.transform (transform);
         
         // segmentPlaneBody.getRenderProps ().setVisible (false);
//         mechModel.add (segmentPlaneBody);
         PolygonalMesh resectionMesh =
            intersector
               .findDifference01 (
                  Donor.getSurfaceMesh (), Plane.getSurfaceMesh ());
         // RigidBody resectionMeshBody = new RigidBody ("ResectionBody" + count);
         // resectionMeshBody.setSurfaceMesh (resectionMesh);
         // mechModel.add (resectionMeshBody);
         PolygonalMesh segment =
            intersector
               .findDifference01 (
                  Donor.getSurfaceMesh (), segmentPlaneBody.getSurfaceMesh ());
         // Apparently this works for adding a mesh in relative space
         RigidTransform3d temptransform = segmentPlane.getMeshToWorld ();
         Donor.addMesh (segmentPlane);
         
//         Donor.addMesh(metalinsertlong);
         segmentPlane.inverseTransform (temptransform);
         Donor.setSurfaceMesh (resectionMesh);
         temptransform = resectionMesh.getMeshToWorld ();
         resectionMesh.inverseTransform (temptransform);
         FixedMeshBody ClippedDonor = new FixedMeshBody ();
         ClippedDonor.setName ("Segment" + count);
         ClippedDonor.setMesh (segment);
         mechModel.add (ClippedDonor);

      }
   }