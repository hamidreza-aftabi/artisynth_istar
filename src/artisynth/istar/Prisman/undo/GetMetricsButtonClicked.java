package artisynth.istar.Prisman.undo;

import java.awt.event.ActionEvent;
import java.util.ArrayList;
import java.util.List;

import javax.swing.AbstractAction;

import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.istar.Assist.Assist;
import artisynth.istar.Prisman.HelperMathFunctions;
import artisynth.istar.Prisman.HelperMeshFunctions;
import artisynth.istar.Prisman.ReconstructionModel;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;

public class GetMetricsButtonClicked extends AbstractAction {
   ReconstructionModel root = null;
   MechModel mechModel = null;
   Assist Assist = new Assist ();
   HelperMeshFunctions meshHelper = new HelperMeshFunctions ();  
   HelperMathFunctions mathHelper = new HelperMathFunctions();
   
   public GetMetricsButtonClicked (ReconstructionModel root, MechModel model) {
      this.root = root;
      this.mechModel = model;
   }
   
   @Override
   public void actionPerformed (ActionEvent e) {
      PolygonalMesh nativeMand = Assist.GetMesh (mechModel, "Mandible");
      PolygonalMesh reconMand = Assist.GetMesh (mechModel, "Postop Mandible");
      
      FixedMeshBody plane1MeshBody = Assist.GetMeshBody (mechModel, "Plane1");
      PolygonalMesh plane1Mesh = (PolygonalMesh)plane1MeshBody.getMesh ();

      Point3d c1 = new Point3d ();
      plane1Mesh.computeCentroid (c1);
      Vector3d planenormal1 = new Vector3d (plane1Mesh.getNormal (0));
      System.out.println (planenormal1.x + ", " + planenormal1.y + ", " + planenormal1.z); //
      RigidTransform3d pose = plane1MeshBody.getPose ();
      Point3d centroid1 = new Point3d (c1);
      centroid1.transform (pose);
      planenormal1.transform (pose);
      
      PolygonalMesh plane1 = MeshFactory.createPlane (90, 90, 10, 10);
      
      meshHelper.createPlane 
         (plane1.getNormal (0), new Vector3d (planenormal1), centroid1, pose, plane1);

      FixedMeshBody plane2MeshBody = Assist.GetMeshBody (mechModel, "Plane2");
      PolygonalMesh plane2Mesh = (PolygonalMesh)plane2MeshBody.getMesh ();

      Point3d c2 = new Point3d ();
      plane2Mesh.computeCentroid (c2);
      Vector3d planenormal2 = new Vector3d (plane2Mesh.getNormal (0));
      pose = plane2MeshBody.getPose ();
      Point3d centroid2 = new Point3d (c2);
      centroid2.transform (pose);
      planenormal2.transform (pose);
      
      PolygonalMesh plane2 = MeshFactory.createPlane (90, 90, 10, 10);
      
      meshHelper.createPlane 
         (plane2.getNormal (0), new Vector3d (planenormal2), centroid2, pose, plane2);
      
      FixedMeshBody p1Mesh = new FixedMeshBody (plane1);
      FixedMeshBody p2Mesh = new FixedMeshBody (plane2);
      ArrayList<FixedMeshBody> PlaneList = new ArrayList<FixedMeshBody> ();
      PlaneList.add (p1Mesh);
      PlaneList.add (p2Mesh);
      
      RigidBody ResectionBody = new RigidBody ();
      
      try {
         ResectionBody
            .setSurfaceMesh (Assist.GetMesh (mechModel, "Mandible").copy ());
         ResectionBody.setName ("Resection Preop");
      }
      catch (Exception k) {
         System.out.println ("Could not clone mandible");
      }
      
      SurfaceMeshIntersector intersector = new SurfaceMeshIntersector();
      
      for (FixedMeshBody plane : PlaneList) {
         PolygonalMesh resectionMesh =
            intersector
               .findDifference01 (
                  ResectionBody.getSurfaceMesh (),
                  (PolygonalMesh)plane.getMesh ());
         root.rerender ();
         ResectionBody.setSurfaceMesh (resectionMesh);
      }
      
      if (ResectionBody
         .getSurfaceMesh ().partitionIntoConnectedMeshes () != null) {
         PolygonalMesh[] MeshList =
            ResectionBody.getSurfaceMesh ().partitionIntoConnectedMeshes ();
         MeshList[0].removeDisconnectedVertices ();
         MeshList[0].removeDisconnectedFaces ();
         ResectionBody.setSurfaceMesh (MeshList[0]);
      }
      
      mechModel.addRigidBody (ResectionBody);
      
      RigidBody PostopBody = new RigidBody ();
      
      try {
         PostopBody
            .setSurfaceMesh (Assist.GetMesh (mechModel, "Postop Mandible").copy ());
         PostopBody.setName ("Resection Postop");
      }
      catch (Exception k) {
         System.out.println ("Could not clone mandible");
      }
      
      for (FixedMeshBody plane : PlaneList) {
         PolygonalMesh resectionMesh =
            intersector
               .findDifference01 (
                  PostopBody.getSurfaceMesh (),
                  (PolygonalMesh)plane.getMesh ());
         root.rerender ();
         PostopBody.setSurfaceMesh (resectionMesh);
      }
      
      if (PostopBody
         .getSurfaceMesh ().partitionIntoConnectedMeshes () != null) {
         PolygonalMesh[] MeshList =
            PostopBody.getSurfaceMesh ().partitionIntoConnectedMeshes ();
         MeshList[0].removeDisconnectedVertices ();
         MeshList[0].removeDisconnectedFaces ();
         PostopBody.setSurfaceMesh (MeshList[0]);
      }
      
      mechModel.addRigidBody (PostopBody);
//      
//      ArrayList<PolygonalMesh> donorSegments = new ArrayList<PolygonalMesh> ();
//      donorSegments.add (PostopBody.getSurfaceMesh ());
//      
//      ArrayList<PolygonalMesh> MeshParts = new ArrayList<PolygonalMesh> ();
//      for (FixedMeshBody plane : PlaneList) {
//         ((PolygonalMesh)plane.getMesh ()).flip ();
//         PolygonalMesh Testmesh =
//            intersector
//               .findDifference01 (
//                  Assist.GetMesh (mechModel, "Postop Mandible"),
//                  (PolygonalMesh)plane.getMesh ());
//         MeshParts.add (Testmesh);
//         ((PolygonalMesh)plane.getMesh ()).flip ();
//         plane.getRenderProps ().setVisible (false);
//      }
//      
//      RigidBody PostopPieceLeft = new RigidBody ("Postop Left");
//      PostopPieceLeft.setSurfaceMesh (MeshParts.get (0));
//      mechModel.addRigidBody (PostopPieceLeft);
//      
//      RigidBody PostopPieceRight = new RigidBody ("Postop Right");
//      PostopPieceRight.setSurfaceMesh (MeshParts.get (1));
//      mechModel.addRigidBody (PostopPieceRight);
//      
//      ArrayList<PolygonalMesh> planes = new ArrayList<PolygonalMesh> ();
//      for (FixedMeshBody temp : PlaneList) {
//         planes.add ((PolygonalMesh)temp.getMesh ());
//      }
      
//      double volume = meshHelper.getVolumeOverlap (nativeMand, reconMand);
//      
//      System.out.println ("Volume between postop and native: " + volume + "%");
      
      double hd95 = mathHelper.get3DHausdorff95 
         (ResectionBody.getSurfaceMesh (), PostopBody.getSurfaceMesh ());
      
      System.out.println ("HD95 between postop and native: " + hd95 + "mm");
//      
//      double bonyContact = meshHelper.getBonyContact 
//         (PostopPieceLeft.getSurfaceMesh (), PostopPieceRight.getSurfaceMesh (), 
//               reconMand, donorSegments, planes, mechModel);
//      
//      System.out.println ("Average bony contact for postop: " + bonyContact + "%");
      
      FrameMarker[] frameMarkers = 
         Assist.GetRigidBody (mechModel, "Postop Mandible").getFrameMarkers ();
      
      for (int i = 1; i < frameMarkers.length - 1; i++) {
         Point3d p = frameMarkers[i].getPosition ();
         Point3d prev = frameMarkers[i - 1].getPosition ();
         Point3d next = frameMarkers[i + 1].getPosition ();
         
         Vector3d vec1 = prev.sub (p);
         Vector3d vec2 = next.sub (p);
         
         double costheta = (vec1.dot (vec2)) / (vec1.norm() * vec2.norm ());
         double theta = Math.acos (costheta);
         
         System.out.println ("Angle at Point " + i + ": " + Math.toDegrees (theta) + " degrees");
      }
      
//      getBonyContact (PolygonalMesh mandibleLeft, PolygonalMesh mandibleRight,
//      PolygonalMesh mandibleFull, List<PolygonalMesh> donorSegmentMeshes, 
//      List<PolygonalMesh> mandiblePlanes, MechModel mechModel)
      
      
      
//      //for analyzing Farahna's cases (need length, so place just two frameMarkers 
//      //on either side of resection; first on non-delineated and then on delineated)
//      FrameMarker[] frameMarkers = 
//         Assist.GetRigidBody (mechModel, "Mandible").getFrameMarkers ();
//      double length = 0.0;
//      
//      for (int i = 0; i < frameMarkers.length - 1; i++) {
//         Point3d p1 = frameMarkers[i].getPosition ();
//         Point3d p2 = frameMarkers[i+1].getPosition ();
//         
//         Vector3d vec = p1.sub (p2);
//         length = length + vec.norm ();
//      }
//      
//      System.out.println ("Length of resection: " + length + "mm");
//      //end delineated length analysis (comment out if need)
      
      
      
   }

}
