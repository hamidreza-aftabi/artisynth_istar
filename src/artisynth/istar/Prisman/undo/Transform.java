package artisynth.istar.Prisman.undo;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import javax.swing.JScrollPane;
import javax.swing.JTextArea;

import artisynth.core.gui.ControlPanel;
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
import maspack.geometry.Vertex3d;
import maspack.interpolation.NumericListKnot;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.AxisAngle;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

public class Transform implements Command {
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

   public Transform (ReconstructionModel root, MechModel mechModel,
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
   public void execute () {
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
            ps = Assist.GetMeshBody (mechModel, "Plane1").getPose ().copy ();
            ps.R.getColumn (2, tn);
            tn.normalize ();
            tn.scale (BOX_WIDTH / 2);
            ps.addTranslation (tn);
            plane2BoxMeshBody.setPose (ps);

            ps = Assist.GetMeshBody (mechModel, "Plane2").getPose ().copy ();
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
                        .GetRigidBody (mechModel, "Mandible").getSurfaceMesh (),
                     Assist.GetMesh (mechModel, "DonorSegment0"));
            FixedMeshBody reconstruction =
               new FixedMeshBody (
                  "ReconstructedMandible", mandiblereconstructed);
            mechModel.addMeshBody (reconstruction);
            Assist
               .GetMeshBody (mechModel, "Donor").getRenderProps ()
               .setVisible (false);
            
            //calculating volume overlap for scapula
            PolygonalMesh resection = Assist.GetMesh (mechModel, "Resection");
            PolygonalMesh scapSegment = Assist.GetMesh (mechModel, "DonorSegment0");
            double scapOverlap = meshHelper.getVolumeOverlap (scapSegment, resection);
            System.out.println ("Volume overlap: " + scapOverlap + "%");
            
            //calculating Hausdorff 95 distance for scapula
            double hd95Scap = mathHelper.get3DHausdorff95 (scapSegment, resection);
            System.out.println ("Hausdorff 95 distance: " + hd95Scap + "mm");
            
            //calculating bony contact for scapula
            //first get planes
            FixedMeshBody plane1MeshBody = Assist.GetMeshBody (mechModel, "Plane1");
            PolygonalMesh plane1Mesh = (PolygonalMesh)plane1MeshBody.getMesh ();
            
            Point3d c1 = new Point3d ();
            plane1Mesh.computeCentroid (c1);
            Vector3d planenormal1 = new Vector3d (plane1Mesh.getNormal (0));
            RigidTransform3d pose = plane1MeshBody.getPose ();
            Point3d centroid1 = new Point3d (c1);
            centroid1.transform (pose);
            planenormal1.transform (pose);
            
            PolygonalMesh newMesh1 = MeshFactory.createPlane (90, 90, 10, 10);
            
            meshHelper.createPlane 
               (newMesh1.getNormal (0), new Vector3d (planenormal1), centroid1, pose, newMesh1);
            
            FixedMeshBody plane2MeshBody = Assist.GetMeshBody (mechModel, "Plane2");
            PolygonalMesh plane2Mesh = (PolygonalMesh)plane2MeshBody.getMesh ();
            
            Point3d c2 = new Point3d ();
            plane2Mesh.computeCentroid (c2);
            Vector3d planenormal2 = new Vector3d (plane2Mesh.getNormal (0));
            pose = plane2MeshBody.getPose ();
            Point3d centroid2 = new Point3d (c2);
            centroid2.transform (pose);
            planenormal2.transform (pose);

            PolygonalMesh newMesh2 = MeshFactory.createPlane (90, 90, 10, 10);
            meshHelper.createPlane 
               (newMesh2.getNormal (0), new Vector3d (planenormal2), centroid2, pose, newMesh2);
            
            ArrayList<PolygonalMesh> planes = new ArrayList<PolygonalMesh> ();
            planes.add (newMesh1);
            planes.add (newMesh2);
            
//            double bonyContactScap = meshHelper.getBonyContactScapula 
//               (mandiblereconstructed, scapSegment, planes, mechModel);
//            System.out.println ("Average bony contact: " + bonyContactScap + "%");
                        
         }
         else {
            mechModel.addRigidBody (clippedDonorMeshBody);
            Assist
               .GetMeshBody (mechModel, "Donor").getRenderProps ()
               .setVisible (false);
            
//            PolygonalMesh x = MeshFactory.createPlane (90, 90, 10, 10);
//            FixedMeshBody MandCutPlane = new FixedMeshBody (x);
//            MandCutPlane.setName ("Mandible Halving Plane - Ceph Calc");
//            mechModel.addMeshBody (MandCutPlane);
//            
//            System.out.println (mechModel.meshBodies ().contains (MandCutPlane));
         }
      }
      else {
         clippedDonorMeshBody = Assist.GetRigidBody (mechModel, "ClippedDonor");
         clippedDonorMesh = clippedDonorMeshBody.getSurfaceMesh ();
         // calculate average normal of scapula to be used as PointsForTransform
         if (root.donorIsScapulaCheckBox.isSelected ()) {
            FrameMarker[] scapulaNormalFrameMarkers =
               Assist
                  .GetRigidBody (mechModel, "ClippedDonor").getFrameMarkers ();
            int len = scapulaNormalFrameMarkers.length;
            Vector3d scapulaNormal0 =
               scapulaNormalFrameMarkers[len - 4].getLocation ();
            Vector3d scapulaNormal1 =
               scapulaNormalFrameMarkers[len - 3].getLocation ();
            Vector3d scapulaNormal2 =
               scapulaNormalFrameMarkers[len - 2].getLocation ();
            Vector3d scapulaNormal3 =
               scapulaNormalFrameMarkers[len - 1].getLocation ();
            Vector3d[] scapulaNormalList = new Vector3d[4];
            scapulaNormalList[0] = scapulaNormal0;
            scapulaNormalList[1] = scapulaNormal1;
            scapulaNormalList[2] = scapulaNormal2;
            scapulaNormalList[3] = scapulaNormal3;
            // scapulaNormal1.sub (scapulaNormal2);
            // scapulaNormal2.sub (scapulaNormal3).negate ();

            Vector3d scapulaPlaneOrigin = new Vector3d ();
            HelperMathFunctions helperMath = new HelperMathFunctions ();
            helperMath
               .planeFit (
                  scapulaNormalList, scapulaPlaneOrigin, scapulaNormalAvg);

            RenderableComponentList<FrameMarker> allMarkers =
               mechModel.frameMarkers ();
            int size = allMarkers.size ();
            for (int i = size - 1; i > size - 5; i--) {
               allMarkers.remove (i);
            }

            // scapulaNormalAvg =
            // scapulaNormal1.cross (scapulaNormal2);
            scapulaNormalAvg.normalize ();
            System.out.println ("scapulaNormalAvg " + scapulaNormalAvg);
         }

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
            clippedDonorMeshBody.getFrameMarkers ();
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

         ArrayList<PolygonalMesh> implantList = meshHelper.getImplantList (mechModel);
         if (!implantList.isEmpty ()) {
            root.fromCenterToSurface.setZero ();
         }

         // Lowering srcPoint which will be the center of the top plane so that
         // when the first cutting plane is really slanted, it doesn't run pass
         // the proximal endpoint
         VectorNd srcPoint = new VectorNd ();
         srcPoint = new VectorNd (root.topPoint.copy ());
         srcPoint.add (root.fromCenterToSurface);
         double scaleLower = 10;
         if (root.donorIsScapulaCheckBox.isSelected ()) {
            scaleLower = 3;
         }

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
            System.out.println (startPoint);
            System.out.println (endPoint);
            
            if (!implantList.isEmpty ()) {
               mandiblePointsForTransform = 
               meshHelper
                  .mandiblePointsForTransformImplants ( 
                     root.dentalImplantList,
                     new Vector3d (simpPoints[i]),
                     new Vector3d (simpPoints[i + 1]));
               DonorPointsForTransform = 
               meshHelper
                  .fibulaPointsForTransformImplants (
                     clippedDonorMesh,
                     new Vector3d (DonorLineFrameMarkers[0].getPosition ()),
                     new Vector3d (startPoint), new Vector3d (endPoint));
            }
            else {
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
               
            // Vector3d center =
               // new Vector3d (
               // (startPoint.get (0) + endPoint.get (0)) / 2,
               // (startPoint.get (1) + endPoint.get (1)) / 2,
               // (startPoint.get (2) + endPoint.get (2)) / 2);
               // Vector3d scapulaNormalAvgScale = scapulaNormalAvg.clone
               // ().scale (0.01);
               // Point3d point3 = new Point3d (center.add
               // (scapulaNormalAvgScale));

               // DonorPointsForTransform[0] = new Point3d (startPoint.x,
               // startPoint.y, startPoint.z);
               // DonorPointsForTransform[1] = new Point3d (endPoint.x,
               // endPoint.y, endPoint.z);
               // DonorPointsForTransform[2] = point3;
               System.out.println ("DonorPointsForTransform " + i);
               System.out
                  .println (
                     DonorPointsForTransform[0] + " " + DonorPointsForTransform[1]
                     + " " + DonorPointsForTransform[2]);
               // System.out.println("point3 manual is " + point3);
               PFTOld.add (i, DonorPointsForTransform);    
            }
            
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
//            Vector3d refPoint = new Vector3d (startPoint);
//            
//            for (int j = 1; j < DonorLineFrameMarkers.length; j++) {
//               Vector3d compPoint = DonorLineFrameMarkers[j].getLocation ();
//               
//               if (compPoint.z < endPoint.z) {
//                  srcPoint = new VectorNd (compPoint);
////                  srcPoint.add (root.fromCenterToSurface);
//                  break;
//               }
//               
////               Vector3d compDist = compPoint.sub (refPoint);
////               Vector3d lengthVec = new Vector3d (root.DonorLengthDir);
////               lengthVec.scale (fibulaLength);
////               
////               Vector3d projVec = lengthVec.scale((compDist.dot (lengthVec) / (lengthVec.norm () * lengthVec.norm ())));
////               
////               if (projVec.norm() >= segmentLength) {
////                  srcPoint = new VectorNd (compPoint);
////                  srcPoint.add (root.fromCenterToSurface);
////                  break;
////               }
//            }
            
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
          for (int i =0; i< root.numberOfSegments; i++) {
          FixedMeshBody tempMeshBody1 = new FixedMeshBody("testingMG" +
          Integer.toString (2 *i), mandiblePlanes.get(2*i));
          FixedMeshBody tempMeshBody2 = new FixedMeshBody("testingMGs" +
          Integer.toString (2 *i+1), mandiblePlanes.get(2*i+1));
//          mechModel.addMeshBody (tempMeshBody1);
//          mechModel.addMeshBody (tempMeshBody2);
          }
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
         if (root.donorIsScapulaCheckBox.isSelected ()) {
            scaleValue = 3;
         }
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

               // Point3d topmax = new Point3d (srcPoint);
               root.topmax.set (srcPoint);

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
               root.botmax.set (endPoint);
            }

            // Adding code for when donor is a scapula. startPoint is projected
            // to the segment's surface. endPoint is calculated accordingly
            // using segment's lengthDir and startPoint
            if (root.donorIsScapulaCheckBox.isSelected ()) {
               AP =
                  new VectorNd (startPoint)
                     .sub (new VectorNd (lineTops.get (i)));
               AB =
                  new VectorNd (new VectorNd (lineBottoms.get (i)))
                     .sub (new VectorNd (lineTops.get (i)));
               VectorNd projectedPoint =
                  new VectorNd (new VectorNd (lineTops.get (i)))
                     .add (
                        new VectorNd (AB)
                           .scale (
                              new VectorNd (AP).dot (AB)
                              / new VectorNd (AB).dot (AB)));
               startPoint = new Point3d (projectedPoint);

               endExtensionVector = new Vector3d (lengthDirs.get (i));
               endExtensionVector.scale (segmentLength);

               endPoint = new Vector3d (startPoint);
               endPoint.add (new Vector3d (endExtensionVector));
               if (i == (root.numberOfSegments - 1)) {
                  root.botmax.set (endPoint);
               }

               Point3d newCentroid = new Point3d (startPoint);
               newCentroid.add (endPoint);
               newCentroid.scale (0.5);

               Vector3d newLengthDir = new Vector3d (lengthDirs.get (i));
               newLengthDir.normalize ().scale (5);

               Point3d newPoint1 = new Point3d (newCentroid);
               newPoint1.sub (newLengthDir);
               Point3d newPoint2 = new Point3d (newCentroid);
               newPoint2.add (newLengthDir);

               // Calculating Transform A for scapula donor
               Vector3d newVectorToSide = new Vector3d (toSides.get (i));
               Point3d[] PFTNew = new Point3d[3];
               PFTNew =
                  meshHelper
                     .pointsForTransform (
                        newPoint1, newPoint2, newVectorToSide);

               System.out
                  .println (
                     "PFTNew is " + PFTNew[0] + " " + PFTNew[1] + " "
                     + PFTNew[2]);
               System.out
                  .println (
                     "PFTOld.get (i) is " + PFTOld.get (i)[0] + " "
                     + PFTOld.get (i)[1] + " " + PFTOld.get (i)[2]);

               RigidTransform3d tBackDonor = new RigidTransform3d ();
               tBackDonor = meshHelper.SVDRegistration (PFTOld.get (i), PFTNew);

               transformBackDonor.add (i, new AffineTransform3d (tBackDonor));
               root.cuttingPlanes
                  .get (2 * i).inverseTransform (transformBackDonor.get (i));
               root.cuttingPlanes
                  .get (2 * i + 1)
                  .inverseTransform (transformBackDonor.get (i));
            }
            else {
               meshHelper
                  .setPlaneOrigin (root.cuttingPlanes.get (2 * i), startPoint);
               meshHelper
                  .setPlaneOrigin (
                     root.cuttingPlanes.get (2 * i + 1), endPoint);
            }
            // Creating Translating back (Transform A) for fibula
            if (!root.donorIsScapulaCheckBox.isSelected ()) {
               AffineTransform3d tBack = new AffineTransform3d ();
               Vector3d translationV = new Vector3d (0, 0, 0);
               for (int g = 0; g <= i; g++) {
                  translationV.add (translate.get (g));
               }
               tBack.setTranslation (new Vector3d (translationV));
               root.translateBack.add (i, new AffineTransform3d (tBack));
            }
            System.out.println (startPoint);
            // Moving endpoint back, accounting for blade thickness
            // This allows for appropriate overlap
            // if (i != 0) {
            //
            // }
            endPoint.add (bufferVector);
            srcPoint = new VectorNd (endPoint);
            
//            for (int j = 1; j < DonorLineFrameMarkers.length; j++) {
//               Vector3d compPoint = DonorLineFrameMarkers[j].getLocation ();
//               
//               if (compPoint.z < endPoint.z) {
//                  compPoint.add (bufferVector);
//                  srcPoint = new VectorNd (compPoint);
////                  srcPoint.add (root.fromCenterToSurface);
//                  break;
//               }
////               
////               Vector3d compDist = compPoint.sub (refPoint);
////               Vector3d lengthVec = new Vector3d (root.DonorLengthDir);
////               lengthVec.scale (fibulaLength);
////               
////               Vector3d projVec = lengthVec.scale((compDist.dot (lengthVec) / (lengthVec.norm () * lengthVec.norm ())));
////               
////               if (projVec.norm() >= segmentLength) {
////                  compPoint.add (bufferVector);
////                  srcPoint = new VectorNd (compPoint);
////                  srcPoint.add (root.fromCenterToSurface);
////                  break;
////               }
//            }
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
         
         ArrayList<PolygonalMesh> implantListClone = new ArrayList<PolygonalMesh> ();
         ArrayList<FixedMeshBody> implantsCloneMeshBodies = 
            new ArrayList<FixedMeshBody> ();
         
         //For dental implants, transform implants back to fibula
         if (!implantList.isEmpty ()) {
            for (int i = 0; i < implantList.size (); i++) {
               for (int j = 0; j < root.numberOfSegments; j++) {
                  //Convert from normal and point to Ax + By + Cz + D
                  //notation
                  PolygonalMesh plane1 =
                     new PolygonalMesh (mandiblePlanes.get (2 * j).clone ());
                  PolygonalMesh plane2 =
                     new PolygonalMesh (
                        mandiblePlanes.get (2 * j + 1).clone ());
                  
                  Vector3d firstPlaneNormal = 
                     new Vector3d (plane1.getNormal (0));
                  Vector3d firstPlaneCenter = new Vector3d ();
                  plane1.computeCentroid (firstPlaneCenter);
                  Vector3d secondPlaneNormal =
                     new Vector3d (plane2.getNormal (0));
                  Vector3d secondPlaneCenter = new Vector3d ();
                  plane2.computeCentroid (secondPlaneCenter);
                  double d1 = firstPlaneNormal.dot (firstPlaneCenter);
                  double d2 = secondPlaneNormal.dot (secondPlaneCenter);
                  d1 = -d1;
                  d2 = -d2;

                  VectorNd equationOfPlane1 = new VectorNd (firstPlaneNormal);
                  equationOfPlane1.append (d1);
                  VectorNd equationOfPlane2 = new VectorNd (secondPlaneNormal);
                  equationOfPlane2.append (d2);

                  Vector3d implantCentroid = new Vector3d ();
                  implantList.get (i).computeCentroid (implantCentroid);
                  VectorNd implantCentroid4 = new VectorNd (implantCentroid);
                  implantCentroid4.append (1);

                  // only transform implant if between planes 1 and 2
                  if (implantCentroid4.dot (equationOfPlane1) > 0
                  && implantCentroid4.dot (equationOfPlane2) > 0) {
                     PolygonalMesh tempPoly =
                        implantList.get (i).clone ();
                     tempPoly.inverseTransform (root.transforms.get (j));
                     tempPoly.transform (root.translateBack.get (j));
                     implantListClone.add (tempPoly);
                     FixedMeshBody implantBody = 
                        new FixedMeshBody (
                           "Transformed Implant" + Integer.toString (i) + "-"
                           + Integer.toString (j), tempPoly);
                     implantsCloneMeshBodies.add (implantBody);
                     mechModel.addMeshBody (implantBody);
                     break;
                  }
                     
                  
               }              
            }
         }
         
         // Transforming Donor segment to mandible
         for (int i = 0; i < root.numberOfSegments; i++) {
            // Step 1
            if (root.donorIsScapulaCheckBox.isSelected ()) {
               donorSegmentMeshes
                  .get (i).transform (transformBackDonor.get (i));
               RigidBody segMeshTest = new RigidBody ("segMeshTest" + i);
               segMeshTest.setSurfaceMesh (donorSegmentMeshes.get (i).clone ());
               mechModel.add (segMeshTest);
            }
            else {
               donorSegmentMeshes
                  .get (i).inverseTransform (root.translateBack.get (i));
            }
            // Step 2
            donorSegmentMeshes.get (i).transform (root.transforms.get (i));
         }
         
         //Moving fibula segment after rotation ot maximize surface contact 
         //between adjacent segments
         if (true) {
            BVIntersector planeMeshIntersector = new BVIntersector ();
            
            for (int i = 0; i < root.numberOfSegments; i++) {
               if (i == 0) {
                  
               } 
               else if (i == root.numberOfSegments - 1 && root.numberOfSegments > 2) {
                  Vector3d segmentCentroid = new Vector3d (simpPoints[i]);
                  RigidTransform3d newPose = new RigidTransform3d();
                  newPose.setTranslation (segmentCentroid);
                  donorSegmentMeshes.get (i).inverseTransform (newPose);
                  donorSegmentBodies.get (i).setPose (newPose);
                  
                  Plane plane1 = new Plane ();
                  Vector3d n1 = new Vector3d (mandiblePlanes.get (2*i).getNormal (0));
                  Vector3d cent1 = new Vector3d ();
                  mandiblePlanes.get (2 * i).computeCentroid (cent1);
                  Vector3d c1 = new Point3d (cent1);
                  RotationMatrix3d r1 = meshHelper.rotatePlane (new Vector3d (0,0,1), new Vector3d (n1));
                  RigidTransform3d t1 = new RigidTransform3d ();
                  t1.setRotation (r1);
                  t1.setTranslation (c1);
                  plane1.transform (t1);
                  
                  Plane plane2 = new Plane ();
                  Vector3d n2 = new Vector3d (mandiblePlanes.get (2*i + 1).getNormal (0).normalize ());
                  Vector3d cent2 = new Vector3d ();
                  mandiblePlanes.get (2 * i + 1).computeCentroid (cent2);
                  Vector3d c2 = new Point3d (cent2);
                  RotationMatrix3d r2 = meshHelper.rotatePlane (new Vector3d (0,0,1), new Vector3d (n2));
                  RigidTransform3d t2 = new RigidTransform3d ();
                  t2.setRotation (r2);
                  t2.setTranslation (c2);
                  plane2.transform (t2);
                  
                  Vector3d axis = new Vector3d (simpPoints[i].sub ((simpPoints[i + 1])));
                  
                  RigidTransform3d rotationAroundAxis = new RigidTransform3d ();
                  double angle = 0;
                  
                  AxisAngle axisAng = new AxisAngle ();
                  
                  ArrayList<LinkedList<Point3d>> contours0 = new ArrayList<LinkedList<Point3d>>();
                  ArrayList<LinkedList<Point3d>> contours1 = new ArrayList<LinkedList<Point3d>>();
                  ArrayList<LinkedList<Point3d>> contours2 = new ArrayList<LinkedList<Point3d>>();
                  if (root.numberOfSegments == 2) {
                     contours1 = planeMeshIntersector.intersectMeshPlane 
                        (Assist.GetMesh (mechModel, "Resection"), plane1, 0.01);
                     
                  }else {
                     contours1 = planeMeshIntersector.intersectMeshPlane 
                        (Assist.GetMesh (mechModel, "Resection"), plane2, 0.01);
                     contours0 = planeMeshIntersector.intersectMeshPlane (
                        donorSegmentMeshes.get (i-1), plane1, 0.01);
                  }
                  
                  int range = 180;
                  double[] differences = new double[range*2];
                  LinkedList<Point3d> contour0 = mathHelper.getLargestContour (contours0);
                  LinkedList<Point3d> contour1 = mathHelper.getLargestContour (contours1);
                  double contourHauss = 0;
                  double contourHaussOther = 0;
                  
                  for (int j = -range; j < range; j++) {
                     angle = j * Math.PI / 180;
                     axisAng.set (axis, angle);
                     rotationAroundAxis.setRotation (axisAng);
                     donorSegmentMeshes.get (i).transform (rotationAroundAxis);
                     
                     contours2 = planeMeshIntersector.intersectMeshPlane (donorSegmentMeshes.get (i), plane1, 0.01);
                     if (contours2.size() == 0) {
                        contourHauss = Double.POSITIVE_INFINITY;
                     }
                     else {
                        LinkedList<Point3d> contour2 = 
                           mathHelper.getLargestContour (contours2);
                        
                        contourHauss = mathHelper.getHaussdorf95 (contour1, contour2);
                        if (root.numberOfSegments != 2) {
                           contourHaussOther = 
                              mathHelper.getHaussdorf95 (contour0, contour2);
                        }
                        
                        donorSegmentMeshes.get (i).inverseTransform (rotationAroundAxis);
                        
                        
                     }
                     differences [j + range] = 
                        0 * contourHauss + 1 * contourHaussOther;
                     
                  }
                  
                  double bestAng = (mathHelper.getIndexOfMin (differences) - range) * Math.PI / 180;
                  axisAng.set (axis, bestAng);
                  rotationAroundAxis.setRotation (axisAng);
                  System.out.println ("Best Angle: " + bestAng * 180 / Math.PI);
                  
                  donorSegmentMeshes.get (i).transform (rotationAroundAxis);
                  
                  donorSegmentMeshes.get (i).transform (donorSegmentBodies.get (i). getPose ());
                  donorSegmentBodies.get (i).setPose (new RigidTransform3d ());

               }
               else if (i == 1) {
                  Vector3d segmentCentroid = new Vector3d (simpPoints[i]);
                  RigidTransform3d newPose = new RigidTransform3d ();
                  newPose.setTranslation (segmentCentroid);
                  donorSegmentMeshes.get (i).inverseTransform (newPose);
                  donorSegmentBodies.get (i).setPose (newPose);
                  
                  Plane plane1 = new Plane ();
                  Vector3d n1 = new Vector3d (mandiblePlanes.get (2*i).getNormal (0));
                  Vector3d cent1 = new Vector3d ();
                  mandiblePlanes.get (2 * i).computeCentroid (cent1);
                  Vector3d c1 = new Point3d (cent1);
                  RotationMatrix3d r1 = meshHelper.rotatePlane (new Vector3d (0,0,1), new Vector3d (n1));
                  RigidTransform3d t1 = new RigidTransform3d ();
                  t1.setRotation (r1);
                  t1.setTranslation (c1);
                  plane1.transform (t1);
                  
                  Plane plane2 = new Plane ();
                  Vector3d n2 = new Vector3d (mandiblePlanes.get (2*i + 1).getNormal (0).normalize ());
                  Vector3d cent2 = new Vector3d ();
                  mandiblePlanes.get (2 * i + 1).computeCentroid (cent2);
                  Vector3d c2 = new Point3d (cent2);
                  RotationMatrix3d r2 = meshHelper.rotatePlane (new Vector3d (0,0,1), new Vector3d (n2));
                  RigidTransform3d t2 = new RigidTransform3d ();
                  t2.setRotation (r2);
                  t2.setTranslation (c2);
                  plane2.transform (t2);
                  
                  Vector3d axis = new Vector3d (simpPoints[i].sub (simpPoints [i + 1]));
                  
                  RigidTransform3d rotationAroundAxis = new RigidTransform3d ();
                  double angle = 0;
                                    
                  AxisAngle axisAng = new AxisAngle ();
                  
                  ArrayList<LinkedList<Point3d>> contours1 = new ArrayList<LinkedList<Point3d>>();
                  ArrayList<LinkedList<Point3d>> contours2 = new ArrayList<LinkedList<Point3d>>();
                  
                  contours1 = planeMeshIntersector.intersectMeshPlane 
                     (Assist.GetMesh (mechModel, "Resection"), plane1, 0.01);
                  
                  int range = 180;
                  double[] differences = new double[range*2];
                  LinkedList<Point3d> contour1 = mathHelper.getLargestContour (contours1);
                  double contourHauss = 0;
                  
                  for (int j = -range; j < range; j++) {
                     angle = j * Math.PI / 180;
                     axisAng.set (axis, angle);
                     rotationAroundAxis.setRotation (axisAng);
                    
                     donorSegmentMeshes.get (i).transform (rotationAroundAxis);
                     
                     contours2 = planeMeshIntersector.intersectMeshPlane 
                        (donorSegmentMeshes.get (i), plane1, 0.01);
                     if (contours2.size() == 0) {
                        contourHauss = Double.POSITIVE_INFINITY;
                     }
                     else {
                        LinkedList<Point3d> contour2 = 
                           mathHelper.getLargestContour (contours2);
                        
                        contourHauss = mathHelper.getHaussdorf95 (contour1, contour2);
                        
                        donorSegmentMeshes.get (i).inverseTransform (rotationAroundAxis);
                        
                     }
                     differences [j + range] = contourHauss;
                  }
                  
                  double bestAng = (mathHelper.getIndexOfMin (differences) - range) * Math.PI / 180;
                  axisAng.set (axis, bestAng);
                  rotationAroundAxis.setRotation (axisAng);
                  System.out.println ("Best Angle: " + bestAng * 180 / Math.PI);
                  
                  donorSegmentMeshes.get (i).transform (rotationAroundAxis);
                  
                  donorSegmentMeshes.get (i).transform (donorSegmentBodies.get (i). getPose ());
                  donorSegmentBodies.get (i).setPose (new RigidTransform3d ());
                  
               }
            }
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
         
//         for (int m = 0; m < root.cuttingPlanes.size (); m++) {
//            FixedMeshBody PLEASEWORK = new FixedMeshBody (root.cuttingPlanes.get (m));
//            PLEASEWORK.setName ("Please work " + m);
//            mechModel.addMeshBody (PLEASEWORK);
//         }

         System.out.println ("End Transforming");
         System.out.println ("Finalize");
         nonResectionFibulaMesh =
            mandHelper
               .visualizeFibulaCuts (
                  mechModel, resectionFibulaMeshBody,
                  nonResectionFibulaMeshBody, nonResectionFibulaMeshes,
                  resectionFibulaMeshes, root.cuttingPlanes, clippedDonorMesh,
                  root.cuttingPlanesMeshBodies, nonResectionFibulaMesh,
                  clippedDonorMeshBody);

         PolygonalMesh mandiblereconstructed =
            Assist.GetMesh (mechModel, "Mandible").copy ();
         
         PolygonalMesh segmentsUnion = new PolygonalMesh ();
         PolygonalMesh forBonyContact = Assist.GetMesh (mechModel, "Mandible").copy ();
         
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
            
            translation.negate ();
            t = new AffineTransform3d ();
            t.setTranslation (translation);
            segmentsUnion.transform (t);
            
            FixedMeshBody trimmed = new FixedMeshBody(segment);
            trimmed.setName ("TrimmedSegment"+i);
            mechModel.addMeshBody (trimmed);
            
            // Getting union
            mandiblereconstructed =
               MeshFactory.getUnion (mandiblereconstructed, segment);
            segmentsUnion = 
               MeshFactory.getUnion (segmentsUnion, segment);
         }
                         
         //calculate volume overlap for fibula
         PolygonalMesh resectedPiece = Assist.GetMesh (mechModel, "Resection").copy ();
         double volOverlap = meshHelper.getVolumeOverlap (resectedPiece, segmentsUnion);
         
         //calculate Hausdorff 95 distance for fibula
         double hd95 = mathHelper.get3DHausdorff95 (resectedPiece, segmentsUnion);

         //calculate bony contact
         //
         
         RigidBody mandLeftBod = Assist.GetRigidBody (mechModel, "Mandible Left");
         PolygonalMesh mandLeft = mandLeftBod.getSurfaceMesh ();
         
         RigidBody mandRightBod = Assist.GetRigidBody (mechModel, "Mandible Right");
         PolygonalMesh mandRight = mandRightBod.getSurfaceMesh ();
         
//         double avgContact = meshHelper.getBonyContact 
//            (mandLeft, mandRight, resectedPiece, forBonyContact, donorSegmentMeshes, mandiblePlanes, mechModel);
         
//         //
//         double avgContact = meshHelper.getBonyContact 
//            (forBonyContact, donorSegmentMeshes, mandiblePlanes, mechModel);

//         System.out.println ("Average bony contact: " + avgContact + "%");
         System.out.println("Volume overlap: " + volOverlap + "%");
         System.out.println("Hausdorff 95 distance: " + hd95 + "mm");         

         // Moving non-resected2 into last segment before getting the union
         Vector3d normal =
            new Vector3d (
               mandiblePlanes
                  .get ((2 * root.numberOfSegments) - 1).getNormal (0));
         normal.normalize ();
         Vector3d translation = new Vector3d (normal);
         translation.scale (0.1);
         AffineTransform3d t = new AffineTransform3d ();
         t.setTranslation (translation);
         PolygonalMesh last =
            new PolygonalMesh (Assist.GetMesh (mechModel, "Mandible").copy ());
         last.transform (t);

         mandiblereconstructed =
            MeshFactory.getUnion (mandiblereconstructed, last.copy ());       

         /// This shouldn't need to be here, but for some reason during the
         /// iteration of adding the segments
         // mandiblereconstructed =
         // MeshFactory
         // .getUnion (mandiblereconstructed, nonResectionMesh1.copy ());

         FixedMeshBody reconstruction =
            new FixedMeshBody ("ReconstructedMandible", mandiblereconstructed);
         System.out.println ("add to model");
         mechModel.addMeshBody (reconstruction);
         Assist.GetRigidBody (mechModel, "Mandible").getRenderProps ().setVisible (false);
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
      Assist.GetRigidBody (mechModel, "Mandible").getRenderProps ().setVisible (true);
      mechModel
         .removeMeshBody (
            Assist.GetMeshBody (mechModel, "ReconstructedMandible"));
      mechModel
         .removeRigidBody (
            Assist.GetRigidBody (mechModel, "Mandible Halving Plane - Ceph Calc"));
      ArrayList<MeshComponent> tempMeshList = new ArrayList<> ();
      for (MeshComponent mesh : mechModel.meshBodies ()) {
         if (mesh.getName () != null
         && (mesh.getName ().contains ("DonorSegment")|| mesh.getName ().contains ("TrimmedSegment"))) {
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
      if (Assist.GetRigidBody (mechModel, "UnresectedDonor") != null) {
         Assist
            .GetRigidBody (mechModel, "UnresectedDonor").getRenderProps ()
            .setVisible (true);
      }
   }

   @Override
   public String getName () {
      // TODO Auto-generated method stub
      return "Transform";
   }
}
