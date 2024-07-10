package artisynth.istar.Prisman.undo;

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
import maspack.geometry.MeshBase;
import maspack.geometry.MeshFactory;
import maspack.geometry.OBB;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.interpolation.NumericListKnot;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

public class MaxillaTransform implements Command {
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
   int clicked = 0;

   public MaxillaTransform (ReconstructionModel root, MechModel mechModel,
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

   @Override
   public void execute () {
      if (clicked < 2) {
         clicked = clicked + 1;
      }
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
         if (root.donorIsScapulaCheckBox.isSelected ()) {

            // code to cut prepped donor; assuming 1 segment recon
            int BOX_WIDTH = 180;
            int BOX_LENGTH = 180;
            int BOX_HEIGHT = 180;
            PolygonalMesh plane1Box =
               MeshFactory
                  .createBox (BOX_WIDTH * 2, BOX_LENGTH * 2, BOX_HEIGHT);
            PolygonalMesh plane2Box =
               MeshFactory
                  .createBox (BOX_WIDTH * 2, BOX_LENGTH * 2, BOX_HEIGHT);
            PolygonalMesh plane3Box =
               MeshFactory
                  .createBox (BOX_WIDTH * 2, BOX_LENGTH * 2, BOX_HEIGHT);
            FixedMeshBody plane1BoxMeshBody =
               new FixedMeshBody ("Box1p", plane1Box);
            FixedMeshBody plane2BoxMeshBody =
               new FixedMeshBody ("Box2p", plane2Box);
            FixedMeshBody plane3BoxMeshBody =
               new FixedMeshBody ("Box3p", plane3Box);

            RigidTransform3d ps =
               Assist
                  .GetMeshBody (mechModel, "ScapulaTrimPlane").getPose ()
                  .copy ();
            Vector3d tn = new Vector3d ();
            ps.R.getColumn (2, tn);
            tn.normalize ();
            tn.scale (BOX_HEIGHT / 2);
            ps.addTranslation (tn);
            plane1BoxMeshBody.setPose (ps);

            // ps = cutDonorPlane1Body.getPose ().copy ();
            ps = root.activePlaneList.get (0).getPose ().copy ();
            ps.R.getColumn (2, tn);
            tn.normalize ();
            tn.scale (BOX_WIDTH / 2);
            ps.addTranslation (tn);
            plane2BoxMeshBody.setPose (ps);

            ps = root.activePlaneList.get (1).getPose ().copy ();
            ps.R.getColumn (2, tn);
            tn.normalize ();
            tn.scale (BOX_WIDTH / 2);
            ps.addTranslation (tn);
            plane3BoxMeshBody.setPose (ps);

            PolygonalMesh intersectedBox = new PolygonalMesh ();
            intersectedBox = MeshFactory.getIntersection (plane1Box, plane2Box);
            intersectedBox =
               MeshFactory.getIntersection (intersectedBox, plane3Box);
            intersectedBox = MeshFactory.createConvexHull (intersectedBox);
            FixedMeshBody intersectedBoxMeshBody =
               new FixedMeshBody ("BoxForScap", intersectedBox);

            clippedDonorMesh =
               MeshFactory
                  .getSubtraction (
                     Assist.GetMesh (mechModel, "Donor"), intersectedBox);
            FixedMeshBody clippedDonorFixedMeshBody =
               new FixedMeshBody ("UnresectedDonor", clippedDonorMesh);
            donorSegmentMeshes
               .add (
                  MeshFactory
                     .getIntersection (
                        Assist.GetMesh (mechModel, "Donor"), intersectedBox));
            donorSegmentBodies
               .add (
                  new FixedMeshBody (
                     "DonorSegment0", donorSegmentMeshes.get (0)));
            mechModel.addMeshBody (clippedDonorFixedMeshBody);
            mechModel.addMeshBody (donorSegmentBodies.get (0));

            // mechModel.addRigidBody (clippedDonorMeshBody);
            mechModel
               .removeMeshBody (
                  Assist.GetMeshBody (mechModel, "ScapulaTrimPlane"));
            PolygonalMesh mandiblereconstructed =
               MeshFactory
                  .getUnion (
                     Assist
                        .GetRigidBody (mechModel, "Maxilla").getSurfaceMesh (),
                     Assist.GetMesh (mechModel, "DonorSegment0"));
            FixedMeshBody reconstruction =
               new FixedMeshBody (
                  "ReconstructedMaxilla", mandiblereconstructed);
            mechModel.addMeshBody (reconstruction);
            Assist
               .GetMeshBody (mechModel, "Donor").getRenderProps ()
               .setVisible (false);
         }
         else {
            mechModel.addRigidBody (clippedDonorMeshBody);
            Assist
               .GetMeshBody (mechModel, "Donor").getRenderProps ()
               .setVisible (false);
         }
      }
      else {
         System.out.println ("Begin Transforming");
         // Clearing List

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

         // Finding length direction of clipped donor by using bounding clipped
         // donor in a box and calculating the sortedAxes
         FrameMarker[] DonorLineFrameMarkers =
            Assist.GetRigidBody (mechModel, "ClippedDonor").getFrameMarkers ();
         OBB boundingbox = clippedDonorMesh.computeOBB ();
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
         clippedDonorMesh.computeCentroid (DonorCentroid);
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
         root.topPoint = new Point3d (x, y, z);

         // Calculating and applying transform
         Vector3d normProx =
            new Vector3d (DonorCuttingPlanes[0].getNormal (0).copy ());
         normProx.normalize ();

         /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

         // Setting up the pair of cutting planes that will be transformed for
         // each Donor segment
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
         VectorNd storageVariable = new VectorNd (root.fromCenterToSurface);

         // Lowering srcPoint which will be the center of the top plane so that
         // when the first cutting plane is really slanted, it doesn't run pass
         // the proximal endpoint
         VectorNd srcPoint = new VectorNd ();
         srcPoint = new VectorNd (root.topPoint.copy ());
         srcPoint.add (root.fromCenterToSurface);
         double scaleLower = 10;

         VectorNd lower =
            new VectorNd (root.DonorLengthDir.copy ()).scale (scaleLower);
         srcPoint.add (lower);

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

         // Initializing a list of variables needed for transform when scapula
         // is the donor bone
         List<Point3d> startPoints = new ArrayList<Point3d> ();
         List<Point3d> endPoints = new ArrayList<Point3d> ();
         List<Point3d[]> PFTOld = new ArrayList<Point3d[]> ();

         // Iterate through each segment and calculate initial transform matrix.
         // No spacing out the planes or reorienting the planes (in case of
         // scapula)
         Vector3d startPoint = new Vector3d ();
         Vector3d endPoint = new Vector3d ();
         Vector3d startExtensionVector = new Vector3d ();
         Vector3d endExtensionVector = new Vector3d ();
         double extensionLength = 0;
         double segmentLength = 0;

         for (int i = 0; i < root.numberOfSegments; i++) {
            // Setting start and end points for the cutting planes
            startPoint = new Vector3d (srcPoint);

            System.out.println (startPoint);

            startPoints.add (i, new Point3d (startPoint));
            segmentLength = simpPoints[i].distance (simpPoints[i + 1]);

            endExtensionVector = new Vector3d (root.DonorLengthDir);
            endExtensionVector.scale (segmentLength);

            endPoint = new Vector3d (startPoint);
            endPoint.add (endExtensionVector);

            endPoints.add (i, new Point3d (endPoint));

            root.cuttingPlanes
               .add (
                  2 * i,
                  new PolygonalMesh (DonorPiecesClippingPlanes[0].clone ()));
            root.cuttingPlanes
               .add (
                  2 * i + 1,
                  new PolygonalMesh (DonorPiecesClippingPlanes[1].clone ()));
            meshHelper
               .setPlaneOrigin (root.cuttingPlanes.get (2 * i), startPoint);
            meshHelper
               .setPlaneOrigin (root.cuttingPlanes.get (2 * i + 1), endPoint);

            /// Use new method of calculating normals when determining dental
            /// Implants
            // For extending the fibula segment

            Point3d[] mandiblePointsForTransform = new Point3d[3];
            Point3d[] DonorPointsForTransform = new Point3d[3];

            // Calculating Transform
            mandiblePointsForTransform =
               meshHelper
                  .pointsForTransform (
                     Assist.GetMesh (mechModel, "Maxilla"),
                     new Vector3d (simpPoints[i]),
                     new Vector3d (simpPoints[i + 1]));
            DonorPointsForTransform =
               meshHelper
                  .pointsForTransform (
                     clippedDonorMesh, new Vector3d (startPoint),
                     new Vector3d (endPoint));
            PFTOld.add (i, DonorPointsForTransform);

            // //View transform points for debugging
            // for(int i1 = 0; i1 < mandiblePointsForTransform.length; i1++) {
            // Point3d p = mandiblePointsForTransform[i1];
            // PolygonalMesh poly = new PolygonalMesh();
            // poly =
            // MeshFactory.createSphere (
            // 1, 20, 20, true);
            // FixedMeshBody polyBody = new
            // FixedMeshBody("maxillaSeg"+i+"Point"+i1, poly);
            // RigidTransform3d trans = new RigidTransform3d();
            // trans.setTranslation (p);
            // polyBody.setPose (trans);
            // mechModel.addMeshBody (polyBody);
            //
            // }
            //
            // for(int i1 = 0; i1 < DonorPointsForTransform.length; i1++) {
            // Point3d p = DonorPointsForTransform[i1];
            // PolygonalMesh poly = new PolygonalMesh();
            // poly =
            // MeshFactory.createSphere (
            // 1, 20, 20, true);
            // FixedMeshBody polyBody = new FixedMeshBody("fibSeg"+i+"Point"+i1,
            // poly);
            // RigidTransform3d trans = new RigidTransform3d();
            // trans.setTranslation (p);
            // polyBody.setPose (trans);
            // mechModel.addMeshBody (polyBody);
            //
            // }

            // Transforming the cuttingPlanes from the donor back to the
            // mandible
            root.transforms
               .add (
                  i,
                  meshHelper
                     .SVDRegistration (
                        mandiblePointsForTransform, DonorPointsForTransform));
            root.cuttingPlanes.get (2 * i).transform (root.transforms.get (i));
            root.cuttingPlanes
               .get (2 * i + 1).transform (root.transforms.get (i));

            srcPoint = new VectorNd (endPoint);
            root.cuttingPlanesMeshBodies
               .add (
                  2 * i,
                  new FixedMeshBody (
                     "Mesh" + String.valueOf (i),
                     root.cuttingPlanes.get (2 * i)));
            root.cuttingPlanesMeshBodies
               .add (
                  2 * i + 1,
                  new FixedMeshBody (
                     "Mesh2" + String.valueOf (i),
                     root.cuttingPlanes.get (2 * i + 1)));
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

         FixedMeshBody plane1MeshBody = root.activePlaneList.get (0);
         FixedMeshBody plane2MeshBody = root.activePlaneList.get (1);
         PolygonalMesh plane1Mesh = (PolygonalMesh)plane1MeshBody.getMesh ();
         PolygonalMesh plane2Mesh = (PolygonalMesh)plane2MeshBody.getMesh ();

         RigidTransform3d poseplane1 = plane1MeshBody.getPose ();
         RigidTransform3d poseplane2 = plane2MeshBody.getPose ();

         Point3d centroidplane1 = new Point3d ();
         Point3d centroidplane2 = new Point3d ();
         plane1Mesh.computeCentroid (centroidplane1);
         plane2Mesh.computeCentroid (centroidplane2);
         centroidplane1.transform (poseplane1);
         centroidplane2.transform (poseplane2);

         Vector3d plane1normal = new Vector3d (plane1Mesh.getNormal (0));
         Vector3d plane2normal = new Vector3d (plane2Mesh.getNormal (0));
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

         // PolygonalMesh mesh0Test = cuttingPlanes.get (0).clone ();
         // FixedMeshBody mesh0TestBody = new FixedMeshBody ("mesh0TestBody",
         // mesh0Test);
         // mechModel.addMeshBody (mesh0TestBody);

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

         // PolygonalMesh meshnTest = cuttingPlanes.get (2 * numberOfSegments -
         // 1).clone ();
         // FixedMeshBody meshnTestBody = new FixedMeshBody ("meshnTestBody",
         // meshnTest);
         // mechModel.addMeshBody (meshnTestBody);
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

         // //Viewing the mandible Cutting planes for debug
         // for (int i =0; i< numberOfSegments; i++) {
         // FixedMeshBody tempMeshBody1 = new FixedMeshBody("testingMG" +
         // Integer.toString (2 *i), mandiblePlanes.get(2*i));
         // FixedMeshBody tempMeshBody2 = new FixedMeshBody("testingMGs" +
         // Integer.toString (2 *i+1), mandiblePlanes.get(2*i+1));
         // mechModel.addMeshBody (tempMeshBody1);
         // mechModel.addMeshBody (tempMeshBody2);
         // }

         // Transforming all the planes back to Donor
         for (int i = 0; i < root.numberOfSegments; i++) {
            root.cuttingPlanes
               .get (2 * i).inverseTransform (root.transforms.get (i));
            root.cuttingPlanes
               .get (2 * i + 1).inverseTransform (root.transforms.get (i));
         }

         // //Viewing donor cutting planes for debug after inverse transform
         // for (int i =0; i< numberOfSegments; i++) {
         // FixedMeshBody tempMeshBody1 = new FixedMeshBody("testingDG" +
         // Integer.toString (2 *i), cuttingPlanes.get(2*i).clone ());
         // FixedMeshBody tempMeshBody2 = new FixedMeshBody("testingDGs" +
         // Integer.toString (2 *i+1), cuttingPlanes.get(2*i+1).clone ());
         // mechModel.addMeshBody (tempMeshBody1);
         // mechModel.addMeshBody (tempMeshBody2);
         // }

         // Finding shortest width of the clippedDonor's bounding box
         double shortestEdgeOfClippedDonor = 1000.0;
         for (int i = 0; i < 3; i++) {
            if (widths.get (i) < shortestEdgeOfClippedDonor) {
               shortestEdgeOfClippedDonor = widths.get (i);
            }
         }

         // Resetting Source Point
         srcPoint = new VectorNd (root.topPoint.copy ());
         srcPoint.add (root.fromCenterToSurface);
         double scaleValue = scaleLower;

         lower = new VectorNd (root.DonorLengthDir.copy ()).scale (scaleValue);
         srcPoint.add (lower);

         // Spacing out the planes by a certain distance, realigning planes when
         // donor is scapula and recalculate transform matrix. Transform back to
         // mandible will be done in 2 steps.
         // 1. Transform back from correct position to initial position on donor
         // (cuttingPlanes still on donor but not spaced out or realigned
         // (PFTDonor calculated initially) --> Transform A
         // 2. Transform back from initial position on donor to mandible using
         // the initially calculated SVD transfromation matrix. --> Transform B
         // If step 1 is not done and SVD matrix is recalculated, segments are
         // not guaranteed to be on the correct position on the mandible

         // List of variables initialization
         List<AffineTransform3d> transformBackDonor =
            new ArrayList<AffineTransform3d> ();

         List<Vector3d> translate = new ArrayList<Vector3d> ();

         for (int i = 0; i < root.numberOfSegments; i++) {
            // Setting new start and end points for the cutting planes
            if (i == 0) {
               startPoint = new Point3d (srcPoint);
               /// Blade Width
               Vector3d bufferVector = new Vector3d ();
               bufferVector.sub (simpPoints[i], simpPoints[i + 1]);
               bufferVector.inverseTransform (root.transforms.get (i));
               bufferVector.normalize ();
               bufferVector.scale (SAW_BLD_THICK);
               startPoint.add (bufferVector);
               //

               root.topmax = new Point3d (srcPoint);

               translate.add (i, new Vector3d (0, 0, 0));
            }
            // I'm not proud of this I don't have time to figure out Jennifer's
            // algorithm so I have to hardcode the extension vector

            // else if (i== numberOfSegments - 1){
            // extensionLength = 20;
            // startExtentionVector =
            // new VectorNd (DonorLengthDir).scale (extensionLength);
            // startPoint = new Vector3d (srcPoint);
            //
            // translate.add (i, new Vector3d (startExtentionVector));
            // }

            // Jennifer's old algo for calculating the extension vector
            // else {
            // // Calculating extension vector based on the angle of the
            // // previous plane so that planes does not intersect on the center
            // // of the donor bone
            // Vector3d prevplanenormal =
            // new Vector3d (cuttingPlanes.get (2 * i - 1).getNormal (0));
            // prevplanenormal.normalize ();
            // Vector3d horizontalnormal =
            // new Vector3d (DonorLengthDir.copy ().negate ());
            // double dot = prevplanenormal.dot (horizontalnormal);
            // double norm = prevplanenormal.norm () * horizontalnormal.norm ();
            // dot = dot / norm;
            // double theta = Math.acos (dot);
            // double beta = Math.PI / 2 - theta;
            // double boversinbeta = shortestEdgeOfClippedDonor * 2 / Math.sin
            // (beta);
            // scaleValue = Float.parseFloat (multiplier.getText ());
            // extensionLength = Math.sin (theta) * boversinbeta * scaleValue;
            // extensionLength = Math.abs (extensionLength);
            //
            // startExtentionVector =
            // new VectorNd (DonorLengthDir).scale (extensionLength);
            // startPoint = new Vector3d (srcPoint);
            // startPoint.add (new Vector3d (startExtentionVector));
            // translate.add (i, new Vector3d (startExtentionVector));
            // }

            else {
               // Only need to calculate the extension vector if the plane angle
               // causes the bottom of the donor segment
               // To be longer than the first segment
               // Calculating extension vector based on the angle of the
               // previous plane so that planes does not intersect on the center
               // of the donor bone

               double firstExtension = 7;
               Vector3d prevPlaneNormal =
                  new Vector3d (
                     root.cuttingPlanes.get (2 * i - 1).getNormal (0));
               prevPlaneNormal.normalize ();
               double dotToCheckIfPlaneIsAngledIn =
                  prevPlaneNormal.dot (root.fromCenterToSurface);
               if (dotToCheckIfPlaneIsAngledIn < 0) {
                  Vector3d horizontalNormal =
                     new Vector3d (root.DonorLengthDir.copy ().negate ());
                  horizontalNormal.normalize ();
                  double dot = prevPlaneNormal.dot (horizontalNormal);
                  if (dot < 0) {
                     horizontalNormal.negate ();
                     dot = prevPlaneNormal.dot (horizontalNormal);
                     double theta = Math.acos (dot);
                     double beta = Math.PI / 2 - theta;
                     firstExtension +=
                        shortestEdgeOfClippedDonor / Math.sin (beta);
                  }
               }

               double secondExtension = 7;
               // Repeat for other plane
               Vector3d nextPlaneNormal =
                  new Vector3d (root.cuttingPlanes.get (2 * i).getNormal (0));
               nextPlaneNormal.normalize ();
               dotToCheckIfPlaneIsAngledIn =
                  nextPlaneNormal.dot (root.fromCenterToSurface);
               if (dotToCheckIfPlaneIsAngledIn > 0) {
                  Vector3d horizontalNormal =
                     new Vector3d (root.DonorLengthDir.copy ().negate ());
                  horizontalNormal.negate ();
                  double dot = nextPlaneNormal.dot (horizontalNormal);
                  if (dot < 0) {
                     double theta = Math.acos (dot);
                     double beta = Math.PI / 2 - theta;
                     secondExtension =
                        shortestEdgeOfClippedDonor / Math.sin (beta);
                  }
               }

               // Add extension based on the angle cutting planes have with each
               // other
               double dot = nextPlaneNormal.dot (prevPlaneNormal);
               double theta = Math.acos (dot);
               scaleValue = Float.parseFloat (multiplier.getText ());
               double thirdExtension =
                  (15 + scaleValue) * (1 - theta / Math.PI);

               extensionLength =
                  firstExtension + secondExtension + thirdExtension;

               startExtensionVector =
                  new Vector3d (root.DonorLengthDir).scale (extensionLength);
               startPoint = new Vector3d (srcPoint);
               /// Buffer Vector for blade width
               Vector3d bufferVector = new Vector3d ();
               bufferVector.sub (simpPoints[i], simpPoints[i + 1]);
               bufferVector.inverseTransform (root.transforms.get (i));
               bufferVector.normalize ();
               bufferVector.scale (SAW_BLD_THICK);
               startPoint.add (bufferVector);
               startPoint.add (new Vector3d (startExtensionVector));
               translate.add (i, new Vector3d (startExtensionVector));
            }

            segmentLength = spoints.get (i).distance (spoints.get (i + 1));

            // Calculating endPoint of the segment
            endExtensionVector = new Vector3d (root.DonorLengthDir);
            endExtensionVector.scale (segmentLength);

            endPoint = new Vector3d (startPoint);
            endPoint.add (new Vector3d (endExtensionVector));
            // Extending the segment to account for blade thickness
            Vector3d bufferVector = new Vector3d ();
            bufferVector.sub (simpPoints[i], simpPoints[i + 1]);
            bufferVector.inverseTransform (root.transforms.get (i));
            bufferVector.normalize ();
            bufferVector.scale (SAW_BLD_THICK);
            endPoint.sub (bufferVector);
            // Have to do it twice
            endPoint.sub (bufferVector);

            if (i == (root.numberOfSegments - 1)) {
               root.botmax = new Point3d (endPoint);
            }

            meshHelper
               .setPlaneOrigin (root.cuttingPlanes.get (2 * i), startPoint);
            meshHelper
               .setPlaneOrigin (root.cuttingPlanes.get (2 * i + 1), endPoint);

            // Creating Translating back (Transform A) for fibula

            AffineTransform3d tBack = new AffineTransform3d ();
            Vector3d translationV = new Vector3d (0, 0, 0);
            for (int g = 0; g <= i; g++) {
               translationV.add (translate.get (g));
            }
            tBack.setTranslation (new Vector3d (translationV));
            root.translateBack.add (i, new AffineTransform3d (tBack));

            System.out.println (startPoint);
            // Moving endpoint back, accounting for blade thickness
            // This allows for appropriate overlap
            // if (i != 0) {
            //
            // }
            endPoint.add (bufferVector);
            srcPoint = new VectorNd (endPoint);
         }

         // Cutting donor into appropriate segments
         for (int i = 0; i < root.numberOfSegments; i++) {
            SurfaceMeshIntersector intersector = new SurfaceMeshIntersector ();
            donorSegmentMeshes
               .add (
                  i,
                  intersector
                     .findDifference01 (
                        clippedDonorMesh.clone (),
                        root.cuttingPlanes.get (2 * i).clone ()));

            donorSegmentMeshes
               .set (
                  i,
                  intersector
                     .findDifference01 (
                        donorSegmentMeshes.get (i),
                        root.cuttingPlanes.get (2 * i + 1).clone ()));
            donorSegmentBodies
               .add (
                  i,
                  new FixedMeshBody (
                     "DonorSegment" + String.valueOf (i),
                     donorSegmentMeshes.get (i)));
         }

         // Transforming Donor segment to mandible
         for (int i = 0; i < root.numberOfSegments; i++) {
            // Step 1

            donorSegmentMeshes
               .get (i).inverseTransform (root.translateBack.get (i));

            // Step 2
            donorSegmentMeshes.get (i).transform (root.transforms.get (i));
         }

         // Using MandiblePlanes To cut segments
         for (int i = 0; i < root.numberOfSegments; i++) {
            SurfaceMeshIntersector intersector = new SurfaceMeshIntersector ();
            donorSegmentMeshesShort
               .add (
                  i,
                  intersector
                     .findDifference01 (
                        donorSegmentMeshes.get (i).clone (),
                        mandiblePlanes.get (2 * i).clone ()));

            donorSegmentMeshesShort
               .set (
                  i,
                  intersector
                     .findDifference01 (
                        donorSegmentMeshesShort.get (i),
                        mandiblePlanes.get (2 * i + 1).clone ()));
            donorSegmentBodiesShort
               .add (
                  i,
                  new FixedMeshBody (
                     "DonorSegmentShort" + String.valueOf (i),
                     mandiblePlanes.get (i)));
         }
         for (int i = 0; i < root.numberOfSegments; i++) {
            if (!donorSegmentBodies.isEmpty ()) {
               mechModel.addMeshBody (donorSegmentBodies.get (i));
            }
            // myModel.addMeshBody (temp.get (2 * i));
            // myModel.addMeshBody (temp.get (2 * i + 1));
         }

         System.out.println ("End Transforming");
         System.out.println ("Finalize");
         // nonResectionFibulaMesh =
         // mandHelper
         // .visualizeFibulaCuts (
         // mechModel, resectionFibulaMeshBody,
         // nonResectionFibulaMeshBody, nonResectionFibulaMeshes,
         // resectionFibulaMeshes, root.cuttingPlanes, clippedDonorMesh,
         // root.cuttingPlanesMeshBodies, nonResectionFibulaMesh,
         // clippedDonorMeshBody);

         PolygonalMesh mandiblereconstructed =
            Assist.GetMesh (mechModel, "ClippedMaxilla").copy ();
         for (int i = 0; i < root.numberOfSegments; i++) {
            // Using the actual size (blade width not accounted for) for making
            // the recon mod
            PolygonalMesh segment = donorSegmentMeshesShort.get (i).clone ();
            segment
               .transform (donorSegmentMeshesShort.get (i).getMeshToWorld ());

            // Moving segments into each other before getting the union. This is
            // where the set of copied plane produced in transform function is
            // used.
            Vector3d normal =
               new Vector3d (mandiblePlanes.get (2 * i + 1).getNormal (0));
            normal.negate ();
            if (i == 0) {
               normal = new Vector3d (mandiblePlanes.get (0).getNormal (0));
            }
            normal.normalize ();
            Vector3d translation = new Vector3d (normal);
            translation.scale (0.1);
            AffineTransform3d t = new AffineTransform3d ();
            t.setTranslation (translation);
            mandiblereconstructed.transform (t);
            translation.negate ();
            t = new AffineTransform3d ();
            t.setTranslation (translation);
            segment.transform (t);

            // Getting union
            mandiblereconstructed =
               MeshFactory.getUnion (mandiblereconstructed, segment);

         }
//         
//         double volOverlap = meshHelper.getVolumeOverlap 
//            (mandiblereconstructed, Assist.GetMesh (mechModel, "BackupMandible"));
//         
//         double hd95 = mathHelper.get3DHandedHausdorff95
//            (mandiblereconstructed, Assist.GetMesh (mechModel, "BackupMandible"));
//         
//         System.out.println ("Volume overlap: " + volOverlap + "%");
//         System.out.println ("Hausdorff 95: " + hd95 + "mm");

         // if (Assist
         // .GetRigidBody (mechModel, "ClippedMaxilla").getSurfaceMesh ()
         // .partitionIntoConnectedMeshes ().length > 1) {
         // Vector3d normal =
         // new Vector3d (
         // mandiblePlanes
         // .get ((2 * root.numberOfSegments) - 1).getNormal (0));
         // normal.normalize ();
         // Vector3d translation = new Vector3d (normal);
         // translation.scale (0.1);
         // AffineTransform3d t = new AffineTransform3d ();
         // t.setTranslation (translation);
         // PolygonalMesh last =
         // new PolygonalMesh (
         // Assist.GetMesh (mechModel, "ClippedMaxilla").copy ());
         // last.transform (t);
         //
         // mandiblereconstructed =
         // MeshFactory.getUnion (mandiblereconstructed, last.copy ());
         // }
         // Moving non-resected2 into last segment before getting the union
         // Vector3d normal =
         // new Vector3d (
         // mandiblePlanes
         // .get ((2 * root.numberOfSegments) - 1).getNormal (0));
         // normal.normalize ();
         // Vector3d translation = new Vector3d (normal);
         // translation.scale (0.1);
         // AffineTransform3d t = new AffineTransform3d ();
         // t.setTranslation (translation);
         // PolygonalMesh last =
         // new PolygonalMesh (Assist.GetMesh (mechModel, "ClippedMaxilla").copy
         // ());
         // last.transform (t);
         //
         // mandiblereconstructed =
         // MeshFactory.getUnion (mandiblereconstructed, last.copy ());

         /// This shouldn't need to be here, but for some reason during the
         /// iteration of adding the segments
         // mandiblereconstructed =
         // MeshFactory
         // .getUnion (mandiblereconstructed, nonResectionMesh1.copy ());
         FixedMeshBody reconstruction =
            new FixedMeshBody ("ReconstructedMaxilla", mandiblereconstructed);
         System.out.println ("add to model");
         mechModel.addMeshBody (reconstruction);
         // finalizeCommand cmd =
         // new finalizeCommand (
         // "Finalize Plan", mechModel, root.numberOfSegments,
         // donorSegmentBodies,
         // nonResectionMeshBody, reconstruction);
         Assist
            .attachVisiblityToPanel (
               reconstruction, root.getControlPanels ().get ("Visibility"));
      }

   }

   @Override
   public void undo () {
      // TODO Auto-generated method stub
      if (clicked > 0) {
         clicked = clicked - 1;
      }
      if (clicked == 1) {
         mechModel
            .removeRigidBody (Assist.GetRigidBody (mechModel, "ClippedDonor"));
      }
      else {
         mechModel
            .removeMeshBody (
               Assist.GetMeshBody (mechModel, "ReconstructedMaxilla"));
         ArrayList<MeshComponent> tempMeshList = new ArrayList<> ();
         for (MeshComponent mesh : mechModel.meshBodies ()) {
            if (mesh.getName () != null
            && mesh.getName ().contains ("DonorSegment")) {
               tempMeshList.add (mesh);
            }
         }
         for (MeshComponent mesh : tempMeshList) {
            mechModel.removeMeshBody (mesh);
         }
         if (Assist.GetRigidBody (mechModel, "ClippedDonor") != null) {
            Assist
               .GetRigidBody (mechModel, "ClippedDonor").getRenderProps ()
               .setVisible (true);
         }
      }
   }

   @Override
   public String getName () {
      // TODO Auto-generated method stub
      return "Transform";
   }
}