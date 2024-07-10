package artisynth.istar.Prisman.undo;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import org.apache.tools.ant.helper.ProjectHelper2.RootHandler;

import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.istar.Assist.Assist;
import artisynth.istar.Prisman.HelperMandibleFunctions;
import artisynth.istar.Prisman.HelperMathFunctions;
import artisynth.istar.Prisman.HelperMeshFunctions;
import artisynth.istar.Prisman.ImprovedFormattedTextField;
import artisynth.istar.Prisman.ReconstructionModel;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.interpolation.NumericListKnot;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.Point3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import artisynth.core.gui.editorManager.Command;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.istar.Assist.Assist;
import artisynth.istar.Prisman.HelperMandibleFunctions;
import artisynth.istar.Prisman.HelperMathFunctions;
import artisynth.istar.Prisman.HelperMeshFunctions;
import artisynth.istar.Prisman.ImprovedFormattedTextField;
import artisynth.istar.Prisman.ReconstructionModel;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.BVIntersector;
import maspack.geometry.MeshFactory;
import maspack.geometry.OBB;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.PolylineMesh;
import maspack.geometry.Vertex3d;
import maspack.interpolation.NumericList;
import maspack.interpolation.NumericListKnot;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

public class TestingTransform2 implements Command {
   Vector3d scapulaNormalAvg = new Vector3d ();
   FixedMeshBody nonResectionMeshBody;
   List<PolygonalMesh> donorSegmentMeshes = new ArrayList<PolygonalMesh> ();
   List<FixedMeshBody> donorSegmentBodies = new ArrayList<FixedMeshBody> ();
   List<PolygonalMesh> mandiblePlanes = new ArrayList<PolygonalMesh> ();
   List<Point3d> segmentCentroids = new ArrayList<Point3d> ();
   PolygonalMesh DonorCuttingPlanes[] = new PolygonalMesh[2];
   List<Point3d> lineTops = new ArrayList<Point3d> ();
   List<Point3d> lineBottoms = new ArrayList<Point3d> ();
   List<Vector3d> lengthDirs = new ArrayList<Vector3d> ();
   List<Vector3d> toSides = new ArrayList<Vector3d> ();
   List<PolygonalMesh> donorSegmentMeshesShort =
      new ArrayList<PolygonalMesh> ();
   List<FixedMeshBody> donorSegmentBodiesShort =
      new ArrayList<FixedMeshBody> ();
   List<Point3d> spoints = new ArrayList<Point3d> ();
   Point3d disPoint = new Point3d ();
   FixedMeshBody resectionFibulaMeshBody;
   FixedMeshBody nonResectionFibulaMeshBody;
   ReconstructionModel root = null;
   Assist Assist = new Assist ();
   HelperMeshFunctions meshHelper = new HelperMeshFunctions ();
   HelperMathFunctions mathHelper = new HelperMathFunctions ();
   HelperMandibleFunctions mandHelper = new HelperMandibleFunctions ();
   MechModel mechModel = null;
   PolygonalMesh[] nonResectionFibulaMeshes;
   PolygonalMesh[] resectionFibulaMeshes;
   ImprovedFormattedTextField DonorDistanceProx;
   ImprovedFormattedTextField DonorDistanceDis;
   ImprovedFormattedTextField multiplier;
   double SAW_BLD_THICK;

   public TestingTransform2 (ReconstructionModel root, MechModel mechModel,
   ImprovedFormattedTextField DonorDistanceProx,
   ImprovedFormattedTextField DonorDistanceDis,
   ImprovedFormattedTextField multiplier, double SAW_BLD_THICK) {
      this.root = root;
      this.mechModel = mechModel;
      this.DonorDistanceDis = DonorDistanceDis;
      this.DonorDistanceProx = DonorDistanceProx;
      this.multiplier = multiplier;
      this.SAW_BLD_THICK = SAW_BLD_THICK;
   }

   //automated scapula solutions- for optimizing fibula solution
   
   @Override
   public void execute() {
      PolygonalMesh nonResectionFibulaMesh = new PolygonalMesh ();
      RigidBody clippedDonorMeshBody = new RigidBody ("ClippedDonor");
      PolygonalMesh clippedDonorMesh = new PolygonalMesh ();
      List<Object> returnObjects =
         this.mandHelper
            .prepareDonor (
               disPoint, Float.parseFloat (DonorDistanceProx.getText ()),
               Float.parseFloat (DonorDistanceDis.getText ()),
               this.DonorCuttingPlanes, clippedDonorMesh,
               Assist.GetMesh (mechModel, "Donor"), root.scapulaTrimPlane,
               root.donorIsScapulaCheckBox.isSelected (),
               Assist.GetMeshBody (mechModel, "Donor"), false);
      clippedDonorMeshBody = new RigidBody ("ClippedDonor");
      disPoint = (Point3d)returnObjects.get (0);
      DonorCuttingPlanes = (PolygonalMesh[])returnObjects.get (1);
      clippedDonorMesh = (PolygonalMesh)returnObjects.get (2);
      clippedDonorMeshBody.setSurfaceMesh (clippedDonorMesh);
      
      if (Assist.GetRigidBody (mechModel, "ClippedDonor") == null) {
         mechModel.addRigidBody (clippedDonorMeshBody);
         Assist
            .GetMeshBody (mechModel, "Donor").getRenderProps ()
            .setVisible (false);
      }
      else {
         clippedDonorMeshBody = Assist.GetRigidBody (mechModel, "ClippedDonor");
         clippedDonorMesh = clippedDonorMeshBody.getSurfaceMesh ();         
         FrameMarker[] DonorLineFrameMarkers =
         clippedDonorMeshBody.getFrameMarkers ();
         
         System.out.println ("Begin Transforming");
         
         for (int i = 0; i < donorSegmentMeshes.size (); i++) {
            mechModel.removeMeshBody (donorSegmentBodies.get (i));
         }
         root.cuttingPlanes.clear ();
         root.cuttingPlanesMeshBodies.clear ();
         donorSegmentMeshes.clear ();
         donorSegmentBodies.clear ();
         spoints.clear ();
         root.translateBack.clear ();
         root.transforms.clear ();
         
         OBB boundingbox = clippedDonorMeshBody.getSurfaceMesh ().computeOBB ();
         RigidTransform3d poseDonor = clippedDonorMeshBody.getPose ();
         Vector3d[] axes = new Vector3d[3];
         for (int i = 0; i < axes.length; i++) {
            axes[i] = new Vector3d ();
         }
         boundingbox.getSortedAxes (axes);
         Vector3d lengthdir = new Vector3d (axes[0].copy ());
         lengthdir.transform (poseDonor);
         root.DonorLengthDir = new VectorNd (lengthdir);
         Point3d DonorCentroid = new Point3d ();
         clippedDonorMeshBody.getSurfaceMesh ().computeCentroid (DonorCentroid);
         DonorCentroid.transform (poseDonor);

         Vector3d widthBB = new Vector3d ();
         boundingbox.getWidths (widthBB);
         PolygonalMesh boundingClip =
            MeshFactory
               .createBox (widthBB.get (0), widthBB.get (1), widthBB.get (2));
         boundingClip.transform (boundingbox.getTransform ());
         FixedMeshBody bCb = new FixedMeshBody ("boundingClip", boundingClip);

         root.DonorLengthDir.normalize ();
         
         // Checking direction of DonorLengthDir to make sure that it is
         // pointing in the direction of:
         // proximal to distal for fibula, distal to proximal for scapula
         Vector3d upDir =
            new Vector3d (DonorLineFrameMarkers[0].getPosition ());
         upDir.sub (DonorCentroid);
         double dotproduct = upDir.dot (root.DonorLengthDir);
         dotproduct =
            dotproduct / (upDir.norm () * root.DonorLengthDir.norm ());
         if (dotproduct > 0) {
            root.DonorLengthDir.negate ();
         }
         
         // Getting prox and distal endpoints
         Vector3d widths = new Vector3d ();
         boundingbox.getHalfWidths (widths);
         Point3d bcenter = new Point3d ();
         boundingbox.getCenter (bcenter);
         Point3d bbcenter = new Point3d (bcenter);
         bbcenter.transform (poseDonor);

         double fibulaLength = 0.0;
         for (int i = 0; i < 3; i++) {
            if (widths.get (i) > fibulaLength) {
               fibulaLength = widths.get (i);
            }
         }
         
         // topPoint and botPoint are top and bottom points on clipped donor
         double x = root.DonorLengthDir.get (0);
         double y = root.DonorLengthDir.get (1);
         double z = root.DonorLengthDir.get (2);
         x = x * fibulaLength + bbcenter.get (0);
         y = y * fibulaLength + bbcenter.get (1);
         z = z * fibulaLength + bbcenter.get (2);
         Point3d botPoint = new Point3d (x, y, z);
         x = root.DonorLengthDir.get (0) * (-1);
         y = root.DonorLengthDir.get (1) * (-1);
         z = root.DonorLengthDir.get (2) * (-1);
         x = x * fibulaLength + bbcenter.get (0);
         y = y * fibulaLength + bbcenter.get (1);
         z = z * fibulaLength + bbcenter.get (2);
         root.topPoint.set (x, y, z);
         
         // Calculating and applying transform
         Vector3d normProx =
            new Vector3d (DonorCuttingPlanes[0].getNormal (0).copy ());
         normProx.normalize ();
         
         PolygonalMesh[] DonorPiecesClippingPlanes = new PolygonalMesh[2];
         if (disPoint.distance (root.topPoint) > disPoint.distance (botPoint)) {
            DonorPiecesClippingPlanes[0] =
               MeshFactory.createPlane (90, 90, 10, 10);
            FixedMeshBody Donorclipping1 =
               new FixedMeshBody (
                  "Donor Pieces 0", DonorPiecesClippingPlanes[0]);
            RigidTransform3d poseclipping0 = Donorclipping1.getPose ();
            Vector3d normal1 = DonorPiecesClippingPlanes[0].getNormal (0);
            meshHelper
               .createPlane (
                  normal1, new Vector3d (normProx), new Point3d (0, 0, 0),
                  poseclipping0, DonorPiecesClippingPlanes[0]);

            DonorPiecesClippingPlanes[1] =
               MeshFactory.createPlane (90, 90, 10, 10);
            FixedMeshBody Donorclipping2 =
               new FixedMeshBody (
                  "Donor Pieces 1", DonorPiecesClippingPlanes[1]);
            poseclipping0 = Donorclipping2.getPose ();
            normal1 = DonorPiecesClippingPlanes[1].getNormal (0);
            meshHelper
               .createPlane (
                  normal1, new Vector3d (normProx.copy ().negate ()),
                  new Point3d (0, 0, 0), poseclipping0,
                  DonorPiecesClippingPlanes[1]);
         }
         else {
            DonorPiecesClippingPlanes[0] =
               MeshFactory.createPlane (90, 90, 10, 10);
            FixedMeshBody Donorclipping1 =
               new FixedMeshBody (
                  "Donor Pieces 0", DonorPiecesClippingPlanes[0]);
            RigidTransform3d poseclipping0 = Donorclipping1.getPose ();
            Vector3d normal1 =
               new Vector3d (DonorPiecesClippingPlanes[0].getNormal (0));
            meshHelper
               .createPlane (
                  normal1, new Vector3d (normProx.copy ().negate ()),
                  new Point3d (0, 0, 0), poseclipping0,
                  DonorPiecesClippingPlanes[0]);

            DonorPiecesClippingPlanes[1] =
               MeshFactory.createPlane (90, 90, 10, 10);
            FixedMeshBody Donorclipping2 =
               new FixedMeshBody (
                  "Donor Pieces 1", DonorPiecesClippingPlanes[1]);
            poseclipping0 = Donorclipping2.getPose ();
            normal1 = new Vector3d (DonorPiecesClippingPlanes[1].getNormal (0));
            meshHelper
               .createPlane (
                  normal1, new Vector3d (normProx), new Point3d (0, 0, 0),
                  poseclipping0, DonorPiecesClippingPlanes[1]);

         }
      
         
         // Projecting first placed DonorLine Marker onto DonorLengthDir
         VectorNd frameMarkerProjOntoCenterline = new VectorNd ();
         VectorNd AP =
            new VectorNd (DonorLineFrameMarkers[0].getPosition ())
               .sub (new VectorNd (root.topPoint.copy ()));
         VectorNd AB =
            new VectorNd (new VectorNd (botPoint.copy ()))
               .sub (new VectorNd (root.topPoint.copy ()));
         frameMarkerProjOntoCenterline =
            new VectorNd (new VectorNd (root.topPoint.copy ()))
               .add (
                  new VectorNd (AB)
                     .scale (
                        new VectorNd (AP).dot (AB)
                        / new VectorNd (AB).dot (AB)));

         root.fromCenterToSurface =
            new VectorNd (DonorLineFrameMarkers[0].getPosition ());
         root.fromCenterToSurface.sub (frameMarkerProjOntoCenterline);
         
         // Lowering srcPoint which will be the center of the top plane so that
         // when the first cutting plane is really slanted, it doesn't run pass
         // the proximal endpoint
         VectorNd srcPoint = new VectorNd ();
         srcPoint = new VectorNd (root.topPoint.copy ());
         srcPoint.add (root.fromCenterToSurface);
         double scaleLower = 10; //ARBITRARY VALUE- change as necessary

         VectorNd lower =
            new VectorNd (root.DonorLengthDir.copy ()).scale (scaleLower);
         srcPoint.add (lower);
         System.out.println ("srcPoint " + srcPoint);
         
         // Gathering a list of the endpoints that define the length and
         // position of each donor segment that is
         // contained in the simplified plate
         Point3d[] simpPoints = new Point3d[root.simpList.getNumKnots ()];
         Iterator<NumericListKnot> simpItr = root.simpList.iterator ();
         int l = 0;
         while (simpItr.hasNext ()) {
            simpPoints[l] = new Point3d (simpItr.next ().v);
            l++;
         }

         // Iterate through each segment and calculate initial transform matrix.
         // No spacing out the planes or reorienting the planes (in case of
         // scapula)
         Vector3d startPoint = new Vector3d ();
         Vector3d endPoint = new Vector3d ();
         Vector3d startExtensionVector = new Vector3d ();
         Vector3d endExtensionVector = new Vector3d ();
         double extensionLength = 0;
         double segmentLength = 0;
         List<Point3d> startPoints = new ArrayList<Point3d> ();
         List<Point3d> endPoints = new ArrayList<Point3d> ();
         List<Point3d[]> PFTOld = new ArrayList<Point3d[]> ();
         
         List<Vector3d> segmentLengths = new ArrayList<Vector3d> ();
         for (int i = 0; i < simpPoints.length - 1; i++) {
            Point3d simp1 = simpPoints[i];
            Point3d simp2 = simpPoints[i + 1];
            Vector3d length = simp1.sub (simp2);
            segmentLengths.add (length);
         }
         
         root.numberOfSegments = segmentLengths.size ();
         
         for (int i = 0; i < root.numberOfSegments; i++) {
            startPoint = new Vector3d (srcPoint);

            startPoints.add (i, new Point3d (startPoint));
            segmentLength = simpPoints[i].distance (simpPoints[i + 1]);

            endExtensionVector = new Vector3d (root.DonorLengthDir);
            endExtensionVector.scale (segmentLength);

            endPoint = new Vector3d (startPoint);
            endPoint.add (endExtensionVector);

            endPoints.add (i, new Point3d (endPoint));
            
            root.cuttingPlanes.add (
               2 * i,
                  new PolygonalMesh (DonorPiecesClippingPlanes[0].clone ()));
            root.cuttingPlanes.add (
               2 * i + 1,
                  new PolygonalMesh (DonorPiecesClippingPlanes[1].clone ()));
            meshHelper
               .setPlaneOrigin (root.cuttingPlanes.get (2 * i), startPoint);
            meshHelper
               .setPlaneOrigin (root.cuttingPlanes.get (2 * i + 1), endPoint);
            
            Point3d[] mandiblePointsForTransform = new Point3d[3];
            Point3d[] DonorPointsForTransform = new Point3d[3];
            // Calculating Transform
            System.out.println (startPoint);
            System.out.println (endPoint);
            
            mandiblePointsForTransform =
            meshHelper
               .pointsForTransform (
                  Assist.GetMesh (mechModel, "Mandible"),
                  new Vector3d (simpPoints[i]),
                  new Vector3d (simpPoints[i + 1]));
            System.out.println (clippedDonorMesh.numFaces ());
            DonorPointsForTransform =
            meshHelper
               .pointsForTransform (
                  clippedDonorMesh, new Vector3d (startPoint),
                  new Vector3d (endPoint));
            
            PFTOld.add (i, DonorPointsForTransform);    

            root.transforms
               .add (
                  i, meshHelper
                     .SVDRegistration (
                        mandiblePointsForTransform, DonorPointsForTransform));
            root.cuttingPlanes.get (2 * i).transform (root.transforms.get (i));
            root.cuttingPlanes
               .get (2 * i + 1).transform (root.transforms.get (i));
            
            //adding planes as FixedMeshBodies so they can be transformed back to donor later
            root.cuttingPlanesMeshBodies.add (
               2 * i,
               new FixedMeshBody (
                  "Mesh" + String.valueOf (i),
                  root.cuttingPlanes.get (2 * i)));
            root.cuttingPlanesMeshBodies.add (
               2 * i + 1,
               new FixedMeshBody (
                  "Mesh2" + String.valueOf (i),
                  root.cuttingPlanes.get (2 * i + 1)));
            
            srcPoint = new VectorNd (endPoint);
            //changing start point to next framemarker
//            Vector3d refPoint = new Vector3d (startPoint);
//            
//            for (int j = 1; j < DonorLineFrameMarkers.length; j++) {
//               Vector3d compPoint = DonorLineFrameMarkers[j].getLocation ();
//               Vector3d compDist = compPoint.sub (refPoint);
//               Vector3d lengthVec = new Vector3d (root.DonorLengthDir);
//               
//               Vector3d projVec = lengthVec.scale((compDist.dot (lengthVec) / (lengthVec.norm () * lengthVec.norm ())));
//               
//               if (projVec.norm() >= segmentLength) {
//                  srcPoint = new VectorNd (compPoint);
//                  break;
//               }
//            }
            
            //LINE 1373 and 631
            
            
         }
         

         for (int i = 0; i < root.numberOfSegments - 1; i++) {
            RigidTransform3d pose1 =
               root.cuttingPlanesMeshBodies.get (2 * i + 1).getPose ();
            RigidTransform3d pose2 =
               root.cuttingPlanesMeshBodies.get (2 * i + 2).getPose ();

            // Calculating mean normal and centroids for planes that are at the
            // segment junctions
            Vector3d normal1 =
               new Vector3d (
                  root.cuttingPlanes.get (2 * i + 1).getNormal (0).copy ());
            Vector3d normal2 =
               new Vector3d (
                  root.cuttingPlanes.get (2 * i + 2).getNormal (0).copy ());
            normal1.transform (pose1);
            normal2.transform (pose2);
            Vector3d avenormal = new Vector3d ();
            normal2.negate ();
            avenormal.add (normal1, normal2);
            avenormal.scale (0.5);
            avenormal.normalize ();
            Point3d cent1 = new Point3d ();
            Point3d cent2 = new Point3d ();
            root.cuttingPlanes.get (2 * i + 1).computeCentroid (cent1);
            root.cuttingPlanes.get (2 * i + 2).computeCentroid (cent2);
            Point3d cen1 = new Point3d (cent1);
            Point3d cen2 = new Point3d (cent2);
            cen1.transform (pose1);
            cen2.transform (pose2);
            Vector3d avecen = new Vector3d ();
            avecen.add (cen1, cen2);
            avecen.scale (0.5);

            // Setting cuttingPlanes in the junctions so that they are flushed
            // together
            normal1.normalize ();
            RotationMatrix3d rotate1 =
               meshHelper.rotatePlane (normal1, avenormal);
            AffineTransform3d rotateplane1 = new AffineTransform3d ();
            rotateplane1.setRotation (rotate1);
            root.cuttingPlanes.get (2 * i + 1).transform (rotateplane1);

            avenormal.negate ();
            normal2.negate ();

            normal2.normalize ();
            rotate1 = meshHelper.rotatePlane (normal2, avenormal);
            rotateplane1 = new AffineTransform3d ();
            rotateplane1.setRotation (rotate1);
            root.cuttingPlanes.get (2 * i + 2).transform (rotateplane1);

            // Translating cuttingPlane to the segment boundary which is the
            // average centroid
            meshHelper
               .setPlaneOrigin (root.cuttingPlanes.get (2 * i + 1), avecen);
            meshHelper
               .setPlaneOrigin (root.cuttingPlanes.get (2 * i + 2), avecen);

         }

         // Getting Information to Hard Code Outermost Cutting Planes
         // Centroids (cuttingPlanes(0) and cuttingPlanes(last))
         RigidTransform3d pose1 =
            root.cuttingPlanesMeshBodies.get (0).getPose ();
         RigidTransform3d pose2 =
            root.cuttingPlanesMeshBodies
               .get (2 * root.numberOfSegments - 1).getPose ();
         RigidTransform3d poseplane1 =
            Assist.GetMeshBody (mechModel, "Plane1").getPose ();
         RigidTransform3d poseplane2 =
            Assist.GetMeshBody (mechModel, "Plane2").getPose ();

         Point3d centroidplane1 = new Point3d ();
         Point3d centroidplane2 = new Point3d ();
         Assist.GetMesh (mechModel, "Plane1").computeCentroid (centroidplane1);
         Assist.GetMesh (mechModel, "Plane2").computeCentroid (centroidplane2);
         centroidplane1.transform (poseplane1);
         centroidplane2.transform (poseplane2);

         Vector3d plane1normal =
            new Vector3d (Assist.GetMesh (mechModel, "Plane1").getNormal (0));
         Vector3d plane2normal =
            new Vector3d (Assist.GetMesh (mechModel, "Plane2").getNormal (0));
         plane1normal.transform (poseplane1);
         plane2normal.transform (poseplane2);

         Point3d centroidmesh0c = new Point3d ();
         Point3d centroidlastmeshc = new Point3d ();
         root.cuttingPlanes.get (0).computeCentroid (centroidmesh0c);
         Point3d centroidmesh0 = new Point3d (centroidmesh0c);
         centroidmesh0.transform (pose1);
         root.cuttingPlanes
            .get (2 * root.numberOfSegments - 1)
            .computeCentroid (centroidlastmeshc);
         Point3d centroidlastmesh = new Point3d (centroidlastmeshc);
         centroidlastmesh.transform (pose2);

         Vector3d normalleft =
            new Vector3d (
               root.cuttingPlanes
                  .get (2 * root.numberOfSegments - 1).getNormal (0));
         normalleft.transform (pose2);
         Vector3d normalright =
            new Vector3d (root.cuttingPlanes.get (0).getNormal (0));
         normalright.transform (pose1);

         // Finding which resection plane is closest to Mesh 0 and transform
         // Mesh 0
         // accordingly (so cuttingPlane0 is flushed with resection plane
         // closestto0 and same thing with cuttingPlanelast)
         String closestto0 = "Plane2";
         String closesttolast = "Plane1";
         double plane2mesh0dis = centroidmesh0.distance (centroidplane2);
         double plane1mesh0dis = centroidmesh0.distance (centroidplane1);
         if (plane1mesh0dis < plane2mesh0dis) {
            closestto0 = "Plane1";
            closesttolast = "Plane2";
         }

         System.out.println ("Closest to Mesh 0: ");
         System.out.println (closestto0);

         // Hardcoding Mesh 0
         if (closestto0 == "Plane2") {
            normalright.normalize ();
            plane2normal.normalize ();
            RotationMatrix3d rotate =
               meshHelper.rotatePlane (normalright, plane2normal);
            AffineTransform3d rotateplane = new AffineTransform3d ();
            rotateplane.setRotation (rotate);
            root.cuttingPlanes.get (0).transform (rotateplane);
            meshHelper
               .setPlaneOrigin (root.cuttingPlanes.get (0), centroidplane2);

         }
         else {
            normalright.normalize ();
            plane1normal.normalize ();
            RotationMatrix3d rotate =
               meshHelper.rotatePlane (normalright, plane1normal);
            AffineTransform3d rotateplane = new AffineTransform3d ();
            rotateplane.setRotation (rotate);
            root.cuttingPlanes.get (0).transform (rotateplane);
            meshHelper
               .setPlaneOrigin (root.cuttingPlanes.get (0), centroidplane1);

         }

         // Hardcoding Last Mesh
         if (closesttolast == "Plane2") {
            normalleft.normalize ();
            plane2normal.normalize ();
            RotationMatrix3d rotate =
               meshHelper.rotatePlane (normalleft, plane2normal);
            AffineTransform3d rotateplane = new AffineTransform3d ();
            rotateplane.setRotation (rotate);
            root.cuttingPlanes
               .get (2 * root.numberOfSegments - 1).transform (rotateplane);
            meshHelper
               .setPlaneOrigin (
                  root.cuttingPlanes.get (2 * root.numberOfSegments - 1),
                  centroidplane2);

         }
         else {
            normalleft.normalize ();
            plane1normal.normalize ();
            RotationMatrix3d rotate =
               meshHelper.rotatePlane (normalleft, plane1normal);
            AffineTransform3d rotateplane = new AffineTransform3d ();
            rotateplane.setRotation (rotate);
            root.cuttingPlanes
               .get (2 * root.numberOfSegments - 1).transform (rotateplane);
            meshHelper
               .setPlaneOrigin (
                  root.cuttingPlanes.get (2 * root.numberOfSegments - 1),
                  centroidplane1);

         }

         for (int i = 0; i <= root.numberOfSegments; i++) {
            spoints.add (i, new Point3d (simpPoints[i]));
            RigidTransform3d poses =
               Assist.GetMeshBody (mechModel, "Clipped Plate").getPose ();
            spoints.get (i).transform (poses);

         }
         // Extending the segment to account for blade thickness

         // Creating a copy of the cuttingPlanes that are on the mandible
         // This copied set is used for finalizeRecon function later so that all
         // the segments have proper intersections for them to be joined
         // together
         for (int i = 0; i < root.numberOfSegments; i++) {
            mandiblePlanes
               .add ((2 * i), root.cuttingPlanes.get (2 * i).clone ());
            mandiblePlanes
               .add ((2 * i + 1), root.cuttingPlanes.get (2 * i + 1).clone ());
         }

         // Viewing the mandible Cutting planes for debug
         // for (int i =0; i< numberOfSegments; i++) {
         // FixedMeshBody tempMeshBody1 = new FixedMeshBody("testingMG" +
         // Integer.toString (2 *i), mandiblePlanes.get(2*i));
         // FixedMeshBody tempMeshBody2 = new FixedMeshBody("testingMGs" +
         // Integer.toString (2 *i+1), mandiblePlanes.get(2*i+1));
         // mechModel.addMeshBody (tempMeshBody1);
         // mechModel.addMeshBody (tempMeshBody2);
         // }
         //
         //
         // Transforming all the planes back to Donor

         for (int i = 0; i < root.numberOfSegments; i++) {
            root.cuttingPlanes
               .get (2 * i).inverseTransform (root.transforms.get (i));
            root.cuttingPlanes
               .get (2 * i + 1).inverseTransform (root.transforms.get (i));

            // Code for realigning cutting planes when donor is scapula.
            // This is done only for scapula due to the curviness of the scapula
            // compared to the fibula
            if (root.donorIsScapulaCheckBox.isSelected ()) {
               PolygonalMesh segment = new PolygonalMesh (clippedDonorMesh);
               segment =
                  MeshFactory
                     .getSubtraction (segment, root.cuttingPlanes.get (2 * i));
               segment =
                  MeshFactory
                     .getSubtraction (
                        segment, root.cuttingPlanes.get (2 * i + 1));
               FixedMeshBody segmentBody =
                  new FixedMeshBody ("Segment" + String.valueOf (i), segment);

               // Creating Bounding Box to Bound the Segment
               OBB segmentBB = segment.computeOBB ();
               Vector3d widthSegments = new Vector3d ();
               segmentBB.getWidths (widthSegments);

               // Collecting frameMarkers that are on the segment
               List<Point3d> markersOnSegment = new ArrayList<Point3d> ();
               for (int m = 0; m < DonorLineFrameMarkers.length; m++) {
                  Point3d pos =
                     new Point3d (DonorLineFrameMarkers[m].getPosition ());
                  if (segmentBB.containsPoint (pos)) {
                     markersOnSegment.add (new Point3d (pos));
                  }
               }

               // Calculating Segment Centroids
               Point3d sCentroid = new Point3d ();
               segment.computeCentroid (sCentroid);
               RigidTransform3d posesegment = segmentBody.getPose ();
               sCentroid.transform (posesegment);
               segmentCentroids.add (i, new Point3d (sCentroid));

               // Getting lengthDirs from intersection contour
               Plane plane1 = new Plane ();
               Plane plane2 = new Plane ();
               Vector3d n1 =
                  new Vector3d (root.cuttingPlanes.get (2 * i).getNormal (0));
               Vector3d n2 =
                  new Vector3d (
                     root.cuttingPlanes.get (2 * i + 1).getNormal (0));
               Vector3d cent1 = new Vector3d ();
               Vector3d cent2 = new Vector3d ();
               root.cuttingPlanes.get (2 * i).computeCentroid (cent1);
               root.cuttingPlanes.get (2 * i + 1).computeCentroid (cent2);
               Vector3d c1 = new Point3d (cent1);
               Vector3d c2 = new Point3d (cent2);

               RotationMatrix3d r1 =
                  meshHelper
                     .rotatePlane (new Vector3d (0, 0, 1), new Vector3d (n1));
               RotationMatrix3d r2 =
                  meshHelper
                     .rotatePlane (new Vector3d (0, 0, 1), new Vector3d (n2));
               RigidTransform3d t1 = new RigidTransform3d ();
               RigidTransform3d t2 = new RigidTransform3d ();
               t1.setRotation (r1);
               t2.setRotation (r2);
               t1.setTranslation (c1);
               t2.setTranslation (c2);
               plane1.transform (t1);
               plane2.transform (t2);

               BVIntersector contourintersector1 = new BVIntersector ();
               ArrayList<LinkedList<Point3d>> contour1 =
                  new ArrayList<LinkedList<Point3d>> ();
               contour1 =
                  contourintersector1
                     .intersectMeshPlane (
                        clippedDonorMesh.clone (), plane1, 0.01);
               ArrayList<PolygonalMesh> contourPlane1 =
                  new ArrayList<PolygonalMesh> ();
               for (int h = 0; h < contour1.size (); h++) {
                  contourPlane1.add (h, new PolygonalMesh ());
                  for (int g = 0; g < contour1.get (h).size (); g++) {
                     Vertex3d newVert1 =
                        new Vertex3d (contour1.get (h).get (g));
                     contourPlane1.get (h).addVertex (newVert1);
                  }
               }
               PolygonalMesh chosenContour1 = new PolygonalMesh ();
               FixedMeshBody chosenContourBody1 = new FixedMeshBody ();
               double maxarea = 0;
               for (int h = 0; h < contour1.size (); h++) {
                  Point3d contourcentroid1 = new Point3d ();
                  contourPlane1.get (h).computeCentroid (contourcentroid1);
                  FixedMeshBody contourIntersection1 =
                     new FixedMeshBody ("contour1", contourPlane1.get (h));
                  RigidTransform3d X = contourIntersection1.getPose ();
                  OBB contourOBB1 = contourPlane1.get (h).computeOBB ();
                  Vector3d cw = new Vector3d ();
                  contourOBB1.getWidths (cw);
                  double area = meshHelper.ComputeContourArea (cw);
                  if (area > maxarea) {
                     maxarea = area;
                     chosenContour1 = new PolygonalMesh (contourPlane1.get (h));
                     chosenContourBody1 =
                        new FixedMeshBody ("Contour1", chosenContour1);
                  }
               }

               Point3d center1 = new Point3d ();
               chosenContour1.computeCentroid (center1);
               RigidTransform3d X = chosenContourBody1.getPose ();
               center1.transform (X);

               BVIntersector contourintersector2 = new BVIntersector ();
               ArrayList<LinkedList<Point3d>> contour2 =
                  new ArrayList<LinkedList<Point3d>> ();
               contour2 =
                  contourintersector2
                     .intersectMeshPlane (clippedDonorMesh, plane2, 0.01);
               ArrayList<PolygonalMesh> contourPlane2 =
                  new ArrayList<PolygonalMesh> ();
               for (int h = 0; h < contour2.size (); h++) {
                  contourPlane2.add (h, new PolygonalMesh ());
                  for (int g = 0; g < contour2.get (h).size (); g++) {
                     Vertex3d newVert2 =
                        new Vertex3d (contour2.get (h).get (g));
                     contourPlane2.get (h).addVertex (newVert2);
                  }
               }
               PolygonalMesh chosenContour2 = new PolygonalMesh ();
               FixedMeshBody chosenContourBody2 = new FixedMeshBody ();
               maxarea = 0;
               for (int h = 0; h < contour2.size (); h++) {
                  Point3d contourcentroid2 = new Point3d ();
                  contourPlane2.get (h).computeCentroid (contourcentroid2);
                  FixedMeshBody contourIntersection2 =
                     new FixedMeshBody ("contour2", contourPlane2.get (h));
                  RigidTransform3d X2 = contourIntersection2.getPose ();
                  OBB contourOBB2 = contourPlane2.get (h).computeOBB ();
                  Vector3d cw = new Vector3d ();
                  contourOBB2.getWidths (cw);
                  double area = meshHelper.ComputeContourArea (cw);
                  if (area > maxarea) {
                     maxarea = area;
                     chosenContour2 = new PolygonalMesh (contourPlane2.get (h));
                     chosenContourBody2 =
                        new FixedMeshBody ("Contour2", chosenContour2);
                  }
               }
                              
               Point3d center2 = new Point3d ();
               chosenContour2.computeCentroid (center2);
               RigidTransform3d X2 = chosenContourBody2.getPose ();
               center2.transform (X2);

               Vector3d lD = new Vector3d (center2);
               lD.sub (center1);
               lD.normalize ();
               if (lD.dot (root.DonorLengthDir) < 0) {
                  lD.negate ();
               }
               lengthDirs.add (i, lD);

               // Calculating lineTops and lineBottoms
               lineTops.add (i, new Point3d (center1));
               lineBottoms.add (i, new Point3d (center2));

               // Calculating vectorToSide for each segment based on
               // framemarkers that are on the segment
               Vector3d total = new Point3d (0, 0, 0);
               // linh - commenting this out to test new way to calculate
               // tosides (normal of plane that fits thru all markers on segment
               for (int m = 0; m < markersOnSegment.size (); m++) {
                  VectorNd projectedPoint =
                     mathHelper
                        .projectPointToLine (
                           new Point3d (markersOnSegment.get (m)),
                           new Vector3d (lineTops.get (i)),
                           new Vector3d (lineBottoms.get (i)));
                  Vector3d vecSide = new Vector3d (markersOnSegment.get (m));
                  vecSide.sub (new Vector3d (projectedPoint));
                  total.add (new Vector3d (vecSide));
               }
               Vector3d segmentCenter = new Vector3d ();
               Vector3d[] markersOnSegmentArray =
                  new Vector3d[markersOnSegment.size ()];
               for (int m = 0; m < markersOnSegment.size (); m++) {
                  markersOnSegmentArray[m] =
                     new Vector3d (markersOnSegment.get (m));
               }
               System.out.println ("total " + total);
               mathHelper
                  .planeFit (markersOnSegmentArray, segmentCenter, total);
               System.out.println ("lineTops" + i + " " + lineTops.get (i));
               System.out
                  .println ("lineBottoms" + i + " " + lineBottoms.get (i));
               total.scale ((1.0 / markersOnSegment.size ()));
               // toSides.add (i, new Vector3d (total));
               toSides.add (i, scapulaNormalAvg);

               System.out
                  .println (
                     "toSidesOriginal" + i + " "
                     + total.scale ((1.0 / markersOnSegment.size ())));
               System.out
                  .println (
                     "toSidesIfScapulaNormalAvg" + i + " " + toSides.get (i));

               lineTops.get (i).add (new Vector3d (toSides.get (i)));
               lineBottoms.get (i).add (new Vector3d (toSides.get (i)));
               System.out.println ("lineTopsNew" + i + " " + lineTops.get (i));
               System.out
                  .println ("lineBottomsNew" + i + " " + lineBottoms.get (i));
            }
         }

         // Viewing donor cutting planes for debug after inverse transform
         for (int i = 0; i < root.numberOfSegments; i++) {
            FixedMeshBody tempMeshBody1 =
               new FixedMeshBody (
                  "testingDG" + Integer.toString (2 * i),
                  root.cuttingPlanes.get (2 * i).clone ());
            FixedMeshBody tempMeshBody2 =
               new FixedMeshBody (
                  "testingDGs" + Integer.toString (2 * i + 1),
                  root.cuttingPlanes.get (2 * i + 1).clone ());
             mechModel.addMeshBody (tempMeshBody1);
             mechModel.addMeshBody (tempMeshBody2);
         }
         
         
      }
      
     
      

   }

   @Override
   public void undo () {
   
   }

   @Override
   public String getName () {
      // TODO Auto-generated method stub
      return "Testing Transform";
   }
}
