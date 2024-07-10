 package artisynth.istar.Atabak;
 
import java.awt.event.ActionEvent;
import java.io.File;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import javax.swing.AbstractAction;
import javax.swing.JCheckBox;

import artisynth.core.driver.Main;
import artisynth.core.gui.editorManager.UndoManager;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.util.ArtisynthPath;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.BVIntersector;
import maspack.geometry.MeshFactory;
import maspack.geometry.OBB;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.PolylineMesh;
import maspack.geometry.Vertex3d;
import maspack.interpolation.Interpolation;
import maspack.interpolation.NumericList;
import maspack.interpolation.NumericListKnot;
import maspack.interpolation.Interpolation.Order;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

public class createMandibleGuideClicked extends AbstractAction {
      ImprovedFormattedTextField p0value = null;
      ImprovedFormattedTextField p1value = null;
      ImprovedFormattedTextField p2value = null;
      ImprovedFormattedTextField p3value = null;
      JCheckBox platingDir;
      JCheckBox oneHolderOption;
      JCheckBox screwsCheckBox;
      JCheckBox connectorOption;
      ReconstructionModel root;
      MechModel mechModel;
      Assist Assist = new Assist ();
      HelperMeshFunctions meshHelper = new HelperMeshFunctions ();
      HelperMathFunctions mathHelper = new HelperMathFunctions ();
      
      public createMandibleGuideClicked (ImprovedFormattedTextField p0value,
      ImprovedFormattedTextField p1value, ImprovedFormattedTextField p2value,
      ImprovedFormattedTextField p3value, JCheckBox platingDir,
      JCheckBox oneHolderOption, JCheckBox screwsCheckBox, JCheckBox connectorOption, ReconstructionModel root,
      MechModel mechModel) {
         putValue (NAME, "Create Mandible Guide Primitive");
         this.p0value = p0value;
         this.p1value = p1value;
         this.p2value = p2value;
         this.p3value = p3value;
         this.platingDir = platingDir;
         this.oneHolderOption = oneHolderOption;
         this.screwsCheckBox=screwsCheckBox;
         this.connectorOption = connectorOption;
         this.root = root;
         this.mechModel = mechModel;
      }

      @Override
      public void actionPerformed (ActionEvent evt) {
         System.out.println ("Begin Creating Mandible Guide Primitive");

         // Getting Info from Plane1 and Plane2 Mesh to create Planes from
         // plane1 and plane2 that can be used for finding intersection between
         // mandible and plane1&2
         RigidTransform3d pose1 =
            Assist.GetMeshBody (mechModel, "Plane1").getPose ();
         RigidTransform3d pose2 =
            Assist.GetMeshBody (mechModel, "Plane2").getPose ();
//         
         Vector3d normalPlane1 = Assist.GetMesh (mechModel, "Plane1").getNormal (0);
         Vector3d normalPlane2 = Assist.GetMesh (mechModel, "Plane2").getNormal (0);
         Point3d plane1centroid = new Point3d ();
         Point3d plane2centroid = new Point3d ();
//         Assist.GetMesh (mechModel, "Plane1").computeCentroid (plane1centroid);
//         Assist.GetMesh (mechModel, "Plane2").computeCentroid (plane2centroid);
         plane1centroid.transform (pose1);
         plane2centroid.transform (pose2);

         Vector3d plane1normal =
            new Vector3d (
               Assist.GetMesh (mechModel, "Plane1").getNormal (0).copy ());
         Vector3d plane2normal =
            new Vector3d (
               Assist.GetMesh (mechModel, "Plane2").getNormal (0).copy ());
//         plane1normal.transform (pose1);
//         plane2normal.transform (pose2);

         OBB Plane1BB = Assist.GetMesh (mechModel, "Plane1").computeOBB ();
         OBB Plane2BB = Assist.GetMesh (mechModel, "Plane2").computeOBB ();
         Vector3d[] paxis = new Vector3d[3];
         for (int i = 0; i < paxis.length; i++) {
            paxis[i] = new Vector3d ();
         }
         Plane1BB.getSortedAxes (paxis);
         Vector3d plane1dir = new Vector3d (paxis[0]);
         plane1dir.transform (pose1);
         Plane2BB.getSortedAxes (paxis);
         Vector3d plane2dir = new Vector3d (paxis[0]);
         plane2dir.transform (pose2);

         RigidTransform3d createPlane1 = new RigidTransform3d (pose1);
         RigidTransform3d createPlane2 = new RigidTransform3d (pose2);

         // Finding info from loaded plane values
            RigidTransform3d poseP1 = new RigidTransform3d ();
            Assist.GetMesh (mechModel, "Plane1").getMeshToWorld (poseP1);
            RigidTransform3d poseP2 = new RigidTransform3d ();
            Assist.GetMesh (mechModel, "Plane2").getMeshToWorld (poseP2);

            RotationMatrix3d plane1Rotation =
               meshHelper
                  .rotatePlane (
                     new Vector3d (0, 0, 1), new Vector3d (normalPlane1));
            RotationMatrix3d plane2Rotation =
               meshHelper
                  .rotatePlane (
                     new Vector3d (0, 0, 1), new Vector3d (normalPlane2));

            Point3d centerPlane1 = new Point3d ();
            centerPlane1 = Assist.GetMeshBody (mechModel, "Plane1").getPosition ();
            Point3d centerPlane2 = new Point3d ();
            centerPlane2 = Assist.GetMeshBody (mechModel, "Plane2").getPosition ();

            plane1normal = new Vector3d (normalPlane1);
            plane2normal = new Vector3d (normalPlane2);


            plane1normal.transform (poseP1);
            plane2normal.transform (poseP2);


            PolygonalMesh tryP1 = MeshFactory.createPlane (90, 90);
            FixedMeshBody t1 = new FixedMeshBody ("tryP1", tryP1);
            RigidTransform3d poset1 = t1.getPose ();
            meshHelper
               .createPlane (
                  new Vector3d (0, 0, 1), new Vector3d (plane1normal),
                  new Point3d (plane1centroid), poset1, tryP1);

            PolygonalMesh tryP2 = MeshFactory.createPlane (90, 90);
            FixedMeshBody t2 = new FixedMeshBody ("tryP2", tryP2);
            RigidTransform3d poset2 = t2.getPose ();
            meshHelper
               .createPlane (
                  new Vector3d (0, 0, 1), new Vector3d (plane2normal),
                  new Point3d (plane2centroid), poset2, tryP2);
            
         // Finding points on PlateMesh that are closest to plane1 and plane2
         int idP1 = 0;
         int idP2 = 0;
         int idP1a = 0;
         int idP2a = 0;
         double dP = 0.0;
         double dR = 0.0;
         double minP = 10000000.0;
         double threshold = 25;
         Vector3d v = null;
         PolylineMesh PlateMesh = (PolylineMesh)Assist.GetNonPolyMesh (mechModel, "Plate");
         int numVP = PlateMesh.numVertices ();
         for (int i = 0; i < numVP; i++) {
            v = PlateMesh.getVertex (i).getPosition ();
            dP =
               mathHelper
                  .PlanePointDistance (
                     new Vector3d (plane1normal), new Vector3d (plane1centroid),
                     new Vector3d (v));
            if (dP < minP) {
               idP1 = i;
               minP = dP;
            }
         }

         minP = 10000000.0;
         for (int i = 0; i < numVP; i++) {
            v = PlateMesh.getVertex (i).getPosition ();
            dP =
               mathHelper
                  .PlanePointDistance (
                     new Vector3d (plane1normal), new Vector3d (plane1centroid),
                     new Vector3d (v));
            dR = v.distance (PlateMesh.getVertex (idP1).getPosition ());
            if ((dR > threshold) && (dP < minP)) {
               minP = dP;
               idP1a = i;
            }
         }

         minP = 1000000000.0;
         for (int i = 0; i < numVP; i++) {
            v = PlateMesh.getVertex (i).getPosition ();
            dP =
               mathHelper
                  .PlanePointDistance (
                     new Vector3d (plane2normal), new Vector3d (plane2centroid),
                     new Vector3d (v));
            if (dP < minP) {
               idP2 = i;
               minP = dP;
            }
         }

         minP = 10000000.0;
         for (int i = 0; i < numVP; i++) {
            v = PlateMesh.getVertex (i).getPosition ();
            dP =
               mathHelper
                  .PlanePointDistance (
                     new Vector3d (plane2normal), new Vector3d (plane2centroid),
                     new Vector3d (v));
            dR = v.distance (PlateMesh.getVertex (idP2).getPosition ());
            if ((dR > threshold) && (dP < minP)) {
               minP = dP;
               idP2a = i;
            }
         }

         if (PlateMesh
            .getVertex (idP1a).getPosition ()
            .distance (plane1centroid) < PlateMesh
               .getVertex (idP1).getPosition ().distance (plane1centroid)) {
            idP1 = idP1a;
         }

         if (PlateMesh
            .getVertex (idP2a).getPosition ()
            .distance (plane2centroid) < PlateMesh
               .getVertex (idP2).getPosition ().distance (plane2centroid)) {
            idP2 = idP2a;
         }

         // Switching the planes if idP2 < idP1 so that plane1 is always located
         // closest to the plating's start point
         if (idP2 < idP1) {
            Point3d temp = new Point3d ();
            temp = plane1centroid;
            plane1centroid = plane2centroid;
            plane2centroid = temp;

            Vector3d temp2 = new Vector3d ();
            temp2 = plane1dir;
            plane1dir = plane2dir;
            plane2dir = temp2;

            Vector3d temp3 = plane1normal;
            plane1normal = plane2normal;
            plane2normal = temp3;

            RigidTransform3d posetemp = pose1;
            pose1 = pose2;
            pose2 = posetemp;

            posetemp = createPlane1;
            createPlane1 = createPlane2;
            createPlane2 = posetemp;

            int tID = idP1;
            idP1 = idP2;
            idP2 = tID;

         }

         Plane plane1 = new Plane ();
         plane1.set (createPlane1);
         Plane plane2 = new Plane ();
         plane2.set (createPlane2);

         // Finding where Plane1 and Plane2 intersects the PlateMesh
         Point3d centroid0 = new Point3d ();
         Point3d centroid1 = new Point3d ();

         centroid0 = PlateMesh.getVertex (idP1).getPosition ();
         centroid1 = PlateMesh.getVertex (idP2).getPosition ();

         // Finding mandcent
         Point3d mandcent = new Point3d ();
         Assist.GetMesh (mechModel, "Mandible").computeCentroid (mandcent);
         RigidTransform3d poseMandible =
            Assist.GetMeshBody (mechModel, "ReconstructedMandible").getPose ();
         mandcent.transform (poseMandible);

         // Getting Plating Direction with Respect to centroid0 and centroid1
         // locations
         String platedir = "rightleft";
         if (platingDir.isSelected ()) {
            platedir = "leftright";
         }

         // Finding Intersection Contour between Plane1 and mandibleMesh
         BVIntersector contourintersector = new BVIntersector ();
         ArrayList<LinkedList<Point3d>> contour1 =
            new ArrayList<LinkedList<Point3d>> ();
         contour1 =
            contourintersector
               .intersectMeshPlane (
                  Assist.GetMesh (mechModel, "Mandible").clone (), plane1,
                  0.01);
         ArrayList<PolygonalMesh> contourPlane1 =
            new ArrayList<PolygonalMesh> ();
         for (int h = 0; h < contour1.size (); h++) {
            contourPlane1.add (h, new PolygonalMesh ());
            for (int g = 0; g < contour1.get (h).size (); g++) {
               Vertex3d newVert = new Vertex3d (contour1.get (h).get (g));
               contourPlane1.get (h).addVertex (newVert);
            }
         }
         
         PolygonalMesh chosenContourPlane1 = new PolygonalMesh ();
         FixedMeshBody chosenContour1Body = new FixedMeshBody ();
         double closestcentroid = 100000;
         for (int h = 0; h < contour1.size (); h++) {
            Point3d contourcentroid = new Point3d ();
            contourPlane1.get (h).computeCentroid (contourcentroid);
            FixedMeshBody contour =
               new FixedMeshBody ("contour1", contourPlane1.get (h));
            RigidTransform3d X = contour.getPose ();
            contourcentroid.transform (X);
            double distanceCentroid = contourcentroid.distance (centroid0);
            OBB contourOBB = contourPlane1.get (h).computeOBB ();
            Vector3d cw = new Vector3d ();
            contourOBB.getWidths (cw);
            double area = meshHelper.ComputeContourArea (cw);
            if ((distanceCentroid < closestcentroid) && (area > 100)) {
               closestcentroid = distanceCentroid;
               chosenContourPlane1 = new PolygonalMesh (contourPlane1.get (h));
               chosenContour1Body =
                  new FixedMeshBody ("Contour1", chosenContourPlane1);
            }
         }
         OBB contour1OBB = chosenContourPlane1.computeOBB ();
         Vector3d[] cAxis1 = new Vector3d[3];
         for (int g = 0; g < 3; g++) {
            cAxis1[g] = new Vector3d ();
         }
         contour1OBB.getSortedAxes (cAxis1);
         RigidTransform3d posecontour1 = chosenContour1Body.getPose ();
         plane1dir = new Vector3d (cAxis1[0]);
         plane1dir.transform (posecontour1);
         Vector3d vectorToSide0 = new Vector3d (cAxis1[1]);
         vectorToSide0.transform (posecontour1);

         // Check Direction of VectorToSide, making sure that it is directed to
         // out of mandible
         Vector3d checkDir = new Vector3d (centroid0);
         checkDir.sub (mandcent);
         checkDir.normalize ();
         if (checkDir.dot (vectorToSide0) < 0) {
            vectorToSide0.negate ();
         }

         // Check Direction of PlaneDir, making sure that it points downwards
         Vector3d planenormal = new Vector3d (plane1normal);
         if (platedir == "rightleft") {
            planenormal.negate ();
         }
         checkDir = new Vector3d (vectorToSide0);
         checkDir.cross (planenormal);
         if (checkDir.dot (plane1dir) < 0) {
            plane1dir.negate ();
         }

         Point3d centroidContour1 = new Point3d ();
         chosenContourPlane1.computeCentroid (centroidContour1);
         RigidTransform3d X = chosenContour1Body.getPose ();
         centroidContour1.transform (X);

         // Finding Intersection Contour between Plane2 and mandibleMesh
         BVIntersector contourintersector2 = new BVIntersector ();
         ArrayList<LinkedList<Point3d>> contour2 =
            new ArrayList<LinkedList<Point3d>> ();
         contour2 =
            contourintersector2
               .intersectMeshPlane (
                  Assist.GetMesh (mechModel, "Mandible").clone (), plane2,
                  0.01);
         ArrayList<PolygonalMesh> contourPlane2 =
            new ArrayList<PolygonalMesh> ();
         for (int h = 0; h < contour2.size (); h++) {
            contourPlane2.add (h, new PolygonalMesh ());
            for (int g = 0; g < contour2.get (h).size (); g++) {
               Vertex3d newVert = new Vertex3d (contour2.get (h).get (g));
               contourPlane2.get (h).addVertex (newVert);
            }
         }

         PolygonalMesh chosenContourPlane2 = new PolygonalMesh ();
         FixedMeshBody chosenContour2Body = new FixedMeshBody ();
         closestcentroid = 100000;
         for (int h = 0; h < contour2.size (); h++) {
            Point3d contourcentroid = new Point3d ();
            contourPlane2.get (h).computeCentroid (contourcentroid);
            FixedMeshBody contour =
               new FixedMeshBody ("contour2", contourPlane2.get (h));
            X = contour.getPose ();
            contourcentroid.transform (X);
            double distanceCentroid = contourcentroid.distance (centroid1);
            OBB contourOBB = contourPlane2.get (h).computeOBB ();
            Vector3d cw = new Vector3d ();
            contourOBB.getWidths (cw);
            double area = meshHelper.ComputeContourArea (cw);
            if ((distanceCentroid < closestcentroid) && (area > 100)) {
               closestcentroid = distanceCentroid;
               chosenContourPlane2 = new PolygonalMesh (contourPlane2.get (h));
               chosenContour2Body =
                  new FixedMeshBody ("Contour2", chosenContourPlane2);
            }
         }

         OBB contour2OBB = chosenContourPlane2.computeOBB ();
         Vector3d[] cAxis2 = new Vector3d[3];
         for (int g = 0; g < 3; g++) {
            cAxis2[g] = new Vector3d ();
         }
         contour2OBB.getSortedAxes (cAxis2);
         RigidTransform3d posecontour2 = chosenContour2Body.getPose ();
         plane2dir = new Vector3d (cAxis2[0]);
         plane2dir.transform (posecontour2);
         Vector3d vectorToSide1 = new Vector3d (cAxis2[1]);
         vectorToSide1.transform (posecontour2);

         // Check Direction of VectorToSide, making sure that it is directed to
         // out of mandible
         checkDir = new Vector3d (centroid1);
         checkDir.sub (mandcent);
         checkDir.normalize ();
         if (checkDir.dot (vectorToSide1) < 0) {
            vectorToSide1.negate ();
         }

         // Check Direction of PlaneDir, making sure that it points downwards
         planenormal = new Vector3d (plane2normal);
         if (platedir == "leftright") {
            planenormal.negate ();
         }
         checkDir = new Vector3d (vectorToSide1);
         checkDir.cross (planenormal);
         if (checkDir.dot (plane2dir) < 0) {
            plane2dir.negate ();
         }

         Point3d centroidContour2 = new Point3d ();
         chosenContourPlane2.computeCentroid (centroidContour2);
         X = chosenContour2Body.getPose ();
         centroidContour2.transform (X);

         Vector3d contour1width = new Vector3d ();
         OBB p1 = chosenContourPlane1.computeOBB ();
         p1.getWidths (contour1width);
         PolygonalMesh contour1BB =
            MeshFactory
               .createBox (
                  contour1width.get (0), contour1width.get (1),
                  contour1width.get (2));
         contour1BB.transform (p1.getTransform ());
         FixedMeshBody C1 = new FixedMeshBody ("C1", contour1BB);
         // mechModel.addMeshBody (C1);

         Vector3d contour2width = new Vector3d ();
         OBB p2 = chosenContourPlane2.computeOBB ();
         p2.getWidths (contour2width);
         PolygonalMesh contour2BB =
            MeshFactory
               .createBox (
                  contour2width.get (0), contour2width.get (1),
                  contour2width.get (2));
         contour2BB.transform (p2.getTransform ());
         FixedMeshBody C2 = new FixedMeshBody ("C2", contour2BB);
         // mechModel.addMeshBody (C2);
         PolygonalMesh[] OuterBox = new PolygonalMesh[2];
         PolygonalMesh[] InnerBox = new PolygonalMesh[2];
         FixedMeshBody[] OutBody = new FixedMeshBody[2];
         FixedMeshBody[] InBody = new FixedMeshBody[2];
         
         String homedir = ArtisynthPath.getHomeDir ();
         File pathHome = new File (homedir);
         String homeParent = pathHome.getParentFile ().getAbsolutePath ();
         String path =
         homeParent
         + "/artisynth_projects/src/artisynth/models/Prisman/stl/squareFrame30.stl";
         PolygonalMesh squareframe = meshHelper.readMesh (path, "squareFrame30.stl");
         // Creating Boxes for slots
         OuterBox[0] = new PolygonalMesh (squareframe.clone ());
         OuterBox[1] = new PolygonalMesh (squareframe.clone ());
         
         path =
         homeParent
         + "/artisynth_projects/src/artisynth/models/Prisman/stl/bladeGuideUnion30.stl";
         PolygonalMesh metalinsert = meshHelper.readMesh (path, "bladeGuideUnion30.stl");
         InnerBox[0] = new PolygonalMesh (metalinsert.clone ());
         InnerBox[1] = new PolygonalMesh (metalinsert.clone ());
         OutBody[0] = new FixedMeshBody ("Outbox0", OuterBox[0]);
         InBody[0] = new FixedMeshBody ("InBox0", InnerBox[0]);
         OutBody[1] = new FixedMeshBody ("Outbox1", OuterBox[1]);
         InBody[1] = new FixedMeshBody ("InBox1", InnerBox[1]);

         // Getting Information from OuterBox
         OBB OutBB = OuterBox[0].computeOBB ();
         Vector3d[] axes = new Vector3d[3];
         for (int j = 0; j < 3; j++) {
            axes[j] = new Vector3d ();
         }
         OutBB.getSortedAxes (axes);
         Vector3d OutlengthDir = new Vector3d (axes[0].copy ());
         Vector3d OutNormal = new Vector3d (axes[1].copy ());
         RigidTransform3d pose = OutBody[0].getPose ();
         OutlengthDir.transform (pose);
         OutNormal.transform (pose);
         OutNormal.negate ();
         Point3d outc = new Point3d ();
         OuterBox[0].computeCentroid (outc);
         outc.transform (pose);
         System.out.println (OutlengthDir);
         System.out.println (OutNormal);

         Vector3d centroidOut0 = new Vector3d ();
         OuterBox[0].computeCentroid (centroidOut0);

         // Transforming slots so they are aligned with plane1 and plane2
         for (int i = 0; i < 2; i++) {
            Point3d point1 = new Point3d (outc);
            Point3d point2 = new Point3d (outc);
            System.out.println (point1);
            OutlengthDir.normalize ();
            Vector3d ext = new Vector3d (OutlengthDir);
            ext.scale (30);
            point1.add (ext);
            System.out.println (point1);
            point2.sub (ext);
            Point3d[] PFTOutBox =
               meshHelper.pointsForTransform (point1, point2, OutNormal);
            if (i == 0) {
               // Shifting centroid0 out
               Point3d newLocationPlane = new Point3d (centroidContour1);
               Vector3d addition = new Vector3d (vectorToSide0);
               addition.normalize ().scale (10);
               newLocationPlane.add (addition);
               point1 = new Point3d (newLocationPlane);
               System.out.println (point1);
               point2 = new Point3d (newLocationPlane);
               plane1dir.normalize ();
               ext = new Vector3d (plane1dir);
               ext.scale (30);
               point1.add (ext);
               System.out.println (point1);
               point2.sub (ext);
               planenormal = new Vector3d (plane1normal);
               if (platedir == "leftright") {
                  planenormal.negate ();
               }
               Point3d[] PFTCuttingPlane =
                  meshHelper.pointsForTransform (point1, point2, planenormal);

               RigidTransform3d tOut =
                  meshHelper.SVDRegistration (PFTCuttingPlane, PFTOutBox);
               OuterBox[i].transform (tOut);
               InnerBox[i].transform (tOut);

               System.out.println ("p1 is " + point1);
               System.out.println ("p2 is " + point2);
               System.out.println ("pnormals is " + planenormal);

               Vector3d[] points = new Vector3d[3];
               points[0] = new Vector3d (point1.x, point1.y, point1.z);
               points[1] = new Vector3d (point2.x, point2.y, point2.z);
               points[2] = planenormal;

               for (FixedMeshBody f : meshHelper.createSpheres2 (points)) {
                  mechModel.addMeshBody (f);
               }

            }
            else {
               Point3d newLocationPlane = new Point3d (centroidContour2);
               Vector3d addition = new Vector3d (vectorToSide1);
               addition.normalize ();
               addition.scale (10);
               newLocationPlane.add (addition);
               point1 = new Point3d (newLocationPlane);
               point2 = new Point3d (newLocationPlane);
               plane2dir.normalize ();
               ext = new Vector3d (plane2dir);
               ext.scale (30);
               point1.add (ext);
               point2.sub (ext);
               planenormal = new Vector3d (plane2normal);
               if (platedir == "rightleft") {
                  planenormal.negate ();
               }
               Point3d[] PFTCuttingPlane =
                  meshHelper.pointsForTransform (point1, point2, planenormal);

               System.out.println ("p1 is " + point1);
               System.out.println ("p2 is " + point2);
               System.out.println ("pnormals is " + planenormal);

               Vector3d[] points = new Vector3d[3];
               points[0] = new Vector3d (point1.x, point1.y, point1.z);
               points[1] = new Vector3d (point2.x, point2.y, point2.z);
               points[2] = planenormal;

               for (FixedMeshBody f : meshHelper.createSpheres2 (points)) {
                  mechModel.addMeshBody (f);
               }

               RigidTransform3d tOut =
                  meshHelper.SVDRegistration (PFTCuttingPlane, PFTOutBox);
               OuterBox[i].transform (tOut);
               InnerBox[i].transform (tOut);
            }
         }

         // Check if master guide's normal is inverted. If yes, correct it
         PolygonalMesh masterMandibleMesh = Assist.GetMesh (mechModel, "Master Mandible");
         PolygonalMesh masterGuideMesh = Assist.GetMesh (mechModel, "Master Guide");
         if (masterMandibleMesh == null){
            masterMandibleMesh = Assist.GetMesh (mechModel, "BackupMandible");
            masterGuideMesh = Assist.GetMesh (mechModel, "BackupMandible");
            FixedMeshBody b1 = new FixedMeshBody("Master Mandible");
            b1.setMesh (masterMandibleMesh);
            FixedMeshBody b2 = new FixedMeshBody("Master Mandible");
            b2.setMesh (masterGuideMesh);
            mechModel.addMeshBody (b1);
            mechModel.addMeshBody(b2);
         }
         boolean inverted = false;
         Vector3d normalmasterGuide =
            meshHelper
               .normalAtClosestVertex (
                  masterMandibleMesh, new Point3d (centroid0));
         Vector3d normalmasterGuide2 =
            meshHelper
               .normalAtClosestVertex (
                  masterMandibleMesh, new Point3d (centroid1));
         if ((normalmasterGuide.dot (vectorToSide0) < 0)
         || (normalmasterGuide2.dot (vectorToSide1) < 0)) {
            System.out.println ("Masters are inverted");
            inverted = true;
         }
         if (inverted) {
            masterGuideMesh.flip ();
            masterMandibleMesh.flip ();
         }
         PolygonalMesh translatedMasterMandible =
            new PolygonalMesh (masterMandibleMesh.clone ());
         PolygonalMesh translatedMasterGuide =
            new PolygonalMesh (masterGuideMesh.clone ());

         translatedMasterGuide.transform (masterGuideMesh.getMeshToWorld ());
         translatedMasterMandible
            .transform (masterMandibleMesh.getMeshToWorld ());
//         Weird translate
         Vector3d translate = new Vector3d (root.TranslateToCentroidMandible);
         AffineTransform3d t = new AffineTransform3d ();
         t.setTranslation (translate);
         translatedMasterMandible.inverseTransform (t);
         translatedMasterGuide.inverseTransform (t);

         // Exporting registered master mandible and master guide
         maspack.geometry.io.StlWriter writer = null;
         try {
            writer =
               new maspack.geometry.io.StlWriter ("MasterMandibleReg.stl");
            writer.writeMesh (translatedMasterMandible);
            writer = new maspack.geometry.io.StlWriter ("MasterGuideReg.stl");
            writer.writeMesh (translatedMasterGuide);
         }
         catch (Exception e) {
            System.out.println ("Unable to save");
         }

         // Creating 4 Planes to cut Master Guide, creating holders for the
         // slots
         PolygonalMesh[] GuideTrim = new PolygonalMesh[4];
         FixedMeshBody[] GuideTrimBody = new FixedMeshBody[4];
         for (int i = 0; i < 4; i++) {
            GuideTrim[i] = MeshFactory.createPlane (90, 90, 15, 15);
            GuideTrimBody[i] =
               new FixedMeshBody (
                  "GuideTrimPlane" + String.valueOf (i), GuideTrim[i]);
         }

         // Getting Plane locations that are user specified distance away
         // away from vertex0 and vertex1
         // TODO: Figure out how to access text
         // System.out.println(Assist.getSplitText(ControlPanel, 0));
         double distance0 = Float.parseFloat (this.p0value.getText ());
         double distance1 = Float.parseFloat (this.p1value.getText ());
         double distance2 = Float.parseFloat (this.p2value.getText ());
         double distance3 = Float.parseFloat (this.p3value.getText ());

         int numVert = PlateMesh.numVertices ();
         // plane0
         int i = idP1;
         double distance = 0;
         while ((distance < distance0) && (i > 0) && (i < numVert)) {
            if (i == 0) {
               break;
            }
            else {
               v = PlateMesh.getVertex (i - 1).getPosition ();
               distance += v.distance (PlateMesh.getVertex (i).getPosition ());
               i--;
            }
         }
         int loc1a = i;
         Vector3d location1a =
            new Vector3d (PlateMesh.getVertex (loc1a).getPosition ());

         // plane1

         i = idP1;
         if (!oneHolderOption.isSelected ()) {
            distance = 0;
            while ((distance < distance1) && (i > 0) && (i < numVert)) {
               if (i == (numVert - 1)) {
                  break;
               }
               else {
                  v = PlateMesh.getVertex (i + 1).getPosition ();
                  distance +=
                     v.distance (PlateMesh.getVertex (i).getPosition ());
                  i++;
               }
            }
         }
         int loc2a = i;
         Vector3d location2a =
            new Vector3d (PlateMesh.getVertex (loc2a).getPosition ());

         // Normals for plane0 and plane1
         Vector3d normal1a = new Vector3d ();
         Vector3d normal1b = new Vector3d ();
         normal1a =
            new Vector3d (PlateMesh.getVertex (loc1a + 1).getPosition ())
               .sub (location1a);
         if (!oneHolderOption.isSelected ()) {
            normal1b =
               new Vector3d (PlateMesh.getVertex (loc2a - 1).getPosition ())
                  .sub (location2a);
         }

         // Plane2
         distance = 0;
         i = idP2;
         if (!oneHolderOption.isSelected ()) {
            while ((distance < distance2) && (i > 0) && (i < numVert)) {
               if (i == 0) {
                  break;
               }
               else {
                  v = PlateMesh.getVertex (i - 1).getPosition ();
                  distance +=
                     v.distance (PlateMesh.getVertex (i).getPosition ());
                  i--;
               }
            }
         }
         int loc1b = i;
         Vector3d location1b =
            new Vector3d (PlateMesh.getVertex (loc1b).getPosition ());

         // Plane3
         i = idP2;
         distance = 0;
         while ((distance < distance3) && (i > 0) && (i < numVert)) {
            if (i == (numVert - 1)) {
               break;
            }
            else {
               v = PlateMesh.getVertex (i + 1).getPosition ();
               distance += v.distance (PlateMesh.getVertex (i).getPosition ());
               i++;
            }
         }

         int loc2b = i;
         Vector3d location2b =
            new Vector3d (PlateMesh.getVertex (loc2b).getPosition ());

         // Normals for plane2 and plane3
         Vector3d normal2a = new Vector3d ();
         Vector3d normal2b = new Vector3d ();
         if (!oneHolderOption.isSelected ()) {
            normal2a =
               new Vector3d (PlateMesh.getVertex (loc1b + 1).getPosition ())
                  .sub (location1b);
         }
         normal2b =
            new Vector3d (PlateMesh.getVertex (loc2b - 1).getPosition ())
               .sub (location2b);

         // Rotating the 4 Planes
         Vector3d sourcenormal = GuideTrim[0].getNormal (0);
         RigidTransform3d poseG = GuideTrimBody[0].getPose ();
         sourcenormal.transform (poseG);
         RotationMatrix3d rotate =
            meshHelper.rotatePlane (sourcenormal, normal1a);
         AffineTransform3d trans = new AffineTransform3d ();
         trans.setRotation (rotate);
         GuideTrim[0].transform (trans);
         rotate = meshHelper.rotatePlane (sourcenormal, normal1b);
         trans.setRotation (rotate);
         if (!oneHolderOption.isSelected ()) {
            GuideTrim[1].transform (trans);
            rotate = meshHelper.rotatePlane (sourcenormal, normal2a);
            trans.setRotation (rotate);
            GuideTrim[2].transform (trans);
         }
         rotate = meshHelper.rotatePlane (sourcenormal, normal2b);
         trans.setRotation (rotate);
         GuideTrim[3].transform (trans);

         // Setting 4 Planes' center
         meshHelper.setPlaneOrigin (GuideTrim[0], new Vector3d (location1a));
         if (!oneHolderOption.isSelected ()) {
            meshHelper.setPlaneOrigin (GuideTrim[1], new Vector3d (location2a));
            meshHelper.setPlaneOrigin (GuideTrim[2], new Vector3d (location1b));
         }
         meshHelper.setPlaneOrigin (GuideTrim[3], new Vector3d (location2b));
         
         // Creating Connector from a bunch of rounded cylinder based on
         // RDPMesh. RDPMesh
         // is split up into a number of sections. In each junction, there is a
         // plane with normal vector equals to the direction of the plate at
         // that point. We find out extension vector (to push the points on the
         // plate outward so that it becomes the points of the connector) by
         // finding the intersection between the planes in each junction and the
         // mandible and getting information from the bounding box of the
         // intersection contour. After the points are extended outward from the
         // mandible, the points are used to create a bunch of rounded cylinder
         // that are then joined together to create a continuous, one-piece
         // connector.

         FixedMeshBody connectorBody = null;
         if (!connectorOption.isSelected ()) {

            System.out.println ("Creating Connector");

            // Finding start and end point of RDPMesh
            Vector3d startPoint = new Vector3d (location2a);
            startPoint.add (centroid0);
            startPoint.scale (0.5);

            Vector3d endPoint = new Vector3d (location1b);
            endPoint.add (centroid1);
            endPoint.scale (0.5);

            int num = PlateMesh.numVertices ();
            double d = 10000;
            int startID = 0;
            for (int g = 0; g < num; g++) {
               Vector3d vPV = PlateMesh.getVertex (g).getPosition ();
               if (vPV.distance (startPoint) < d) {
                  d = vPV.distance (startPoint);
                  startID = g;
               }
            }

            d = 10000;
            int endID = 0;
            for (int g = 0; g < num; g++) {
               Vector3d vPV = PlateMesh.getVertex (g).getPosition ();
               if (vPV.distance (endPoint) < d) {
                  d = vPV.distance (endPoint);
                  endID = g;
               }
            }

            // Copying plateMesh that are within the RDP's start and end.

            ArrayList<Vector3d> points = new ArrayList<Vector3d> ();
            ArrayList<Integer> pV = new ArrayList<Integer> ();
            double dis = 0;
            int aID = 1;
            points
               .add (
                  0,
                  new Vector3d (PlateMesh.getVertex (startID).getPosition ()));
            pV.add (0, startID);
            for (int j = (startID); j != endID; j++) {
               dis =
                  dis + PlateMesh
                     .getVertex (j).getPosition ()
                     .distance (PlateMesh.getVertex (j + 1).getPosition ());
            }

            double thres = dis / 8.0;

            // Breaking the copied plateMesh into sections
            dis = 0.0;
            for (int j = startID; j != endID; j++) {
               dis =
                  dis + PlateMesh
                     .getVertex (j).getPosition ()
                     .distance (PlateMesh.getVertex (j - 1).getPosition ());
               if (aID > 7) {
                  break;
               }
               else if (dis >= thres) {
                  dis = 0;
                  pV.add (aID, j);
                  points
                     .add (
                        aID,
                        new Vector3d (PlateMesh.getVertex (j).getPosition ()));
                  aID++;
               }
            }
            pV.add (aID, endID);
            points
               .add (
                  aID,
                  new Vector3d (PlateMesh.getVertex (endID).getPosition ()));

            // Creating plane at each junction where the normal of the plane is
            // the direction of the plate at that point
            ArrayList<Plane> sPlanes = new ArrayList<Plane> ();
            for (int j = 0; j < points.size (); j++) {
               RigidTransform3d sPose = new RigidTransform3d ();
               sPose.setTranslation (new Vector3d (points.get (j)));
               Vector3d targetnormal = new Vector3d ();
               if (platedir == "rightleft") {
                  targetnormal =
                     new Vector3d (
                        PlateMesh.getVertex (pV.get (j) - 1).getPosition ());
                  targetnormal
                     .sub (PlateMesh.getVertex (pV.get (j)).getPosition ());
               }
               else {
                  targetnormal =
                     new Vector3d (
                        PlateMesh.getVertex (pV.get (j) + 1).getPosition ());
                  targetnormal
                     .sub (PlateMesh.getVertex (pV.get (j)).getPosition ());
               }
               targetnormal.normalize ();
               RotationMatrix3d sRot =
                  meshHelper.rotatePlane (new Vector3d (0, 0, 1), targetnormal);
               sPose.setRotation (sRot);
               sPlanes.add (j, new Plane ());
               sPlanes.get (j).set (sPose);
            }

            // Getting contour intersection between each plane and mandible
            // Extension vector is contour's axis[1]
            BVIntersector sIntersect = new BVIntersector ();
            ArrayList<LinkedList<Point3d>> contourS =
               new ArrayList<LinkedList<Point3d>> ();
            ArrayList<PolygonalMesh> contourSP =
               new ArrayList<PolygonalMesh> ();
            ArrayList<Vector3d> extVect = new ArrayList<Vector3d> ();
            ArrayList<Vector3d> extDown = new ArrayList<Vector3d> ();
            for (int j = 0; j < sPlanes.size (); j++) {
               contourS =
                  sIntersect
                     .intersectMeshPlane (
                        Assist.GetMesh (mechModel, "Mandible").clone (),
                        sPlanes.get (j), 0.01);
               contourSP.clear ();
               for (int h = 0; h < contourS.size (); h++) {
                  contourSP.add (h, new PolygonalMesh ());
                  for (int g = 0; g < contourS.get (h).size (); g++) {
                     Vertex3d newVert = new Vertex3d (contourS.get (h).get (g));
                     contourSP.get (h).addVertex (newVert);
                  }
               }

               PolygonalMesh chosenContourS = new PolygonalMesh ();
               FixedMeshBody chosenContourSBody = new FixedMeshBody ();
               closestcentroid = 100000;
               for (int h = 0; h < contourSP.size (); h++) {
                  Point3d contourcentroid = new Point3d ();
                  contourSP.get (h).computeCentroid (contourcentroid);
                  FixedMeshBody contourBody =
                     new FixedMeshBody ("contour", contourSP.get (h));
                  X = contourBody.getPose ();
                  contourcentroid.transform (X);
                  double distanceCentroid =
                     contourcentroid.distance (points.get (j));
                  OBB contourOBB = contourSP.get (h).computeOBB ();
                  Vector3d cw = new Vector3d ();
                  contourOBB.getWidths (cw);
                  double area = meshHelper.ComputeContourArea (cw);
                  if ((distanceCentroid < closestcentroid) && (area > 100)) {
                     closestcentroid = distanceCentroid;
                     chosenContourS = new PolygonalMesh (contourSP.get (h));
                     chosenContourSBody =
                        new FixedMeshBody (
                           "ContourS" + String.valueOf (j), chosenContourS);
                  }
               }

               OBB contoursOBB = chosenContourS.computeOBB ();
               Vector3d[] sAxis = new Vector3d[3];
               for (int g = 0; g < 3; g++) {
                  sAxis[g] = new Vector3d ();
               }
               contoursOBB.getSortedAxes (sAxis);
               RigidTransform3d poseSP = chosenContourSBody.getPose ();
               extVect.add (j, sAxis[1]);
               extVect.get (j).transform (poseSP);
               Vector3d sCheck = new Vector3d (points.get (j));
               sCheck.sub (mandcent);
               if (sCheck.dot (extVect.get (j)) < 0) {
                  extVect.get (j).negate ();
               }
               extDown.add (j, sAxis[0]);
               extDown.get (j).transform (poseSP);
               sCheck = new Vector3d (extVect.get (j));
               sCheck.cross (sPlanes.get (j).getNormal ());
               if (sCheck.dot (extDown.get (j)) < 0) {
                  extDown.get (j).negate ();
               }
               Vector3d widthscontours = new Vector3d ();
               contoursOBB.getWidths (widthscontours);
               RigidTransform3d trs = contoursOBB.getTransform ();
               PolygonalMesh sBB =
                  MeshFactory
                     .createBox (
                        widthscontours.get (0), widthscontours.get (1),
                        widthscontours.get (2));
               sBB.transform (trs);
               FixedMeshBody ContourBodyS =
                  new FixedMeshBody ("ContourS" + String.valueOf (j), sBB);
               root.rerender ();
            }

            // Pulling points out of the mandible
            ArrayList<Vector3d> newspoints = new ArrayList<Vector3d> ();
            int scale = 3;
            Vector3d exts = null;
            for (int j = 0; j < points.size (); j++) {
               exts = new Vector3d (extVect.get (j));
               if (j > 0) {
                  double dot = exts.dot (extVect.get (j - 1));
                  dot = dot / (exts.norm () * extVect.get (j - 1).norm ());
                  if (dot > Math.cos (Math.toRadians (30))) {
                     exts.add (new Vector3d (extVect.get (j - 1)));
                     exts.scale (0.5);
                  }
               }

               Vector3d down = new Vector3d (extDown.get (j));
               Vector3d downout = new Vector3d (extVect.get (j));
               downout.add (new Vector3d (extDown.get (j)));
               downout.scale (0.5);
               newspoints.add (j, points.get (j));
               int mid = points.size () / 2;
               Vector3d slanted = new Vector3d (down);
               slanted.add (exts);
               slanted.scale (0.5);

               if ((j != 0) && (j != (points.size () - 1))) {
                  if (j <= mid) {
                     scale = scale + 5;
                  }
                  else {
                     scale = scale - 5;
                  }
                  exts.scale (scale);

               }
               else {
                  exts.scale (3);

               }
               down.scale (7.5);
               newspoints.get (j).add (down);
               newspoints.get (j).add (exts);
            }
            exts = null;

            // Projecting Points to a plane calculated by crossing vector
            // (newspoints(last)-newspoints(0)) and vector
            // (newspoint(mid)-mandible's centroid)) to reduce connector's
            // squiggles.
            Vector3d betweenSlots = new Vector3d (newspoints.get (0));
            betweenSlots.sub (newspoints.get (newspoints.size () - 1));
            betweenSlots.normalize ();

            if (platingDir.isSelected ()) {
               betweenSlots.negate ();
            }

            Point3d midPoint = new Point3d (newspoints.get (0));
            midPoint.add (newspoints.get (newspoints.size () - 1));
            midPoint.scale (0.5);

            Vector3d centroidToMid = new Vector3d (midPoint);
            centroidToMid.sub (mandcent);
            centroidToMid.normalize ();

            Vector3d ConnectorPlaneNormal = new Vector3d (betweenSlots);
            ConnectorPlaneNormal.cross (centroidToMid);
            ConnectorPlaneNormal.normalize ();

            PolygonalMesh ConnectorPlane = MeshFactory.createPlane (90, 90);
            FixedMeshBody ConnectorPlaneBody =
               new FixedMeshBody ("ConnectorPlane", ConnectorPlane);
            RigidTransform3d Pose = ConnectorPlaneBody.getPose ();
            meshHelper
               .createPlane (
                  new Vector3d (0, 0, 1), new Vector3d (ConnectorPlaneNormal),
                  new Point3d (midPoint), Pose, ConnectorPlane);

            List<Vector3d> ProjectedPoints = new ArrayList<Vector3d> ();
            for (int g = 0; g < newspoints.size (); g++) {
               Vector3d projected =
                  new Vector3d (
                     mathHelper
                        .ProjectPointToPlane (
                           new Vector3d (newspoints.get (g)),
                           new Vector3d (midPoint),
                           new Vector3d (ConnectorPlaneNormal)));
               ProjectedPoints.add (g, projected);
            }

            // Interpolating projectedPoints to make the set of points smoother
            Point3d[] cnpoints = new Point3d[ProjectedPoints.size ()];
            NumericList cn = new NumericList (3);
            Interpolation cubic = new Interpolation ();
            cubic.setOrder (Order.SphericalCubic);
            cn.setInterpolation (cubic);
            int numberOfSubdivisions = 50;
            int size = ProjectedPoints.size ();

            for (int j = 0; j < size; j++) {
               cnpoints[j] = new Point3d (ProjectedPoints.get (j));
               cn.add (cnpoints[j], j * numberOfSubdivisions / (size - 1));
            }

            VectorNd[] interpolatedVectors = new VectorNd[numberOfSubdivisions];

            for (int j = 0; j < numberOfSubdivisions; j++) {
               interpolatedVectors[j] = new VectorNd ();
               interpolatedVectors[j].setSize (3);
               cn.interpolate (interpolatedVectors[j], j);

            }

            // Adding Results of interpolation to the numericList
            for (int j = 0; j < numberOfSubdivisions; j++) {
               cn.add (interpolatedVectors[j], j);
            }

            int newSize = cn.getNumKnots ();
            Point3d[] interpolatedPoints = new Point3d[newSize];
            int[][] indices = new int[newSize - 1][2];

            Iterator<NumericListKnot> itr = cn.iterator ();

            int m = 0;
            while (itr.hasNext ()) {
               interpolatedPoints[m] = new Point3d (itr.next ().v);
               m++;
            }

            // Creating Rounded Cylinders from InterpolatedPoints
            size = interpolatedPoints.length;
            PolygonalMesh[] cyl = new PolygonalMesh[size - 1];
            FixedMeshBody[] cylBody = new FixedMeshBody[size - 1];
            for (int j = 0; j < (size - 1); j++) {
               double length =
                  interpolatedPoints[j].distance (interpolatedPoints[j + 1]);
               cyl[j] =
                  MeshFactory.createRoundedCylinder (3.5, length, 30, 8, false);
               cylBody[j] =
                  new FixedMeshBody ("Boxes" + String.valueOf (j), cyl[j]);
            }
            Vector3d[] cnaxes = new Vector3d[3];
            for (int l = 0; l < 3; l++) {
               cnaxes[l] = new Vector3d ();
            }
            for (int j = 0; j < (size - 1); j++) {
               RigidTransform3d poseC = cylBody[j].getPose ();
               OBB CylCC = cyl[j].computeOBB ();
               CylCC.getSortedAxes (cnaxes);
               Vector3d sourcedir = new Vector3d (cnaxes[0]);
               sourcedir.transform (poseC);
               Vector3d centroid = new Vector3d ();
               cyl[j].computeCentroid (centroid);
               centroid.transform (poseC);
               Point3d point1 = new Point3d (centroid);
               Point3d point2 = new Point3d (centroid);
               Vector3d ext = new Vector3d (sourcedir);
               double length =
                  interpolatedPoints[j].distance (interpolatedPoints[j + 1]);
               ext.scale (length + 2.5);
               point1.add (ext);
               point2.sub (ext);
               Point3d[] PFTMandible =
                  meshHelper
                     .pointsForTransform (
                        Assist.GetMesh (mechModel, "Mandible"),
                        interpolatedPoints[j], interpolatedPoints[j + 1]);
               Point3d[] PFTCyl =
                  meshHelper.pointsForTransform (cyl[j], point1, point2);
               RigidTransform3d transforms =
                  meshHelper.SVDRegistration (PFTMandible, PFTCyl);
               cyl[j].transform (transforms);
            }

            // Uniting the rounded cylinders to create a continuous, one-piece
            // connector
            PolygonalMesh connector = new PolygonalMesh (cyl[0].clone ());
            for (int j = 1; j < (cyl.length); j++) {
               connector = MeshFactory.getUnion (connector, cyl[j].clone ());
            }

            // Creating reinforcement bar to avoid the guide from bending
            // double length =
            // ProjectedPoints.get (2).distance (ProjectedPoints.get (6));
            // PolygonalMesh bar =
            // MeshFactory.createRoundedCylinder (3.0, length, 30, 8, false);
            // FixedMeshBody barBody = new FixedMeshBody ("bar", bar);
            // RigidTransform3d poseC = barBody.getPose ();
            // OBB CylCC = bar.computeOBB ();
            // CylCC.getSortedAxes (cnaxes);
            // Vector3d sourcedir = new Vector3d (cnaxes[0]);
            // sourcedir.transform (poseC);
            // Vector3d centroid = new Vector3d ();
            // bar.computeCentroid (centroid);
            // centroid.transform (poseC);
            // Point3d point1 = new Point3d (centroid);
            // Point3d point2 = new Point3d (centroid);
            // Vector3d ext = new Vector3d (sourcedir);
            // ext.scale (length);
            // point1.add (ext);
            // point2.sub (ext);
            // Point3d[] PFTMandible =
            // meshHelper.pointsForTransform (
            // mandibleMesh, ProjectedPoints.get (2),
            // ProjectedPoints.get (6));
            // Point3d[] PFTCyl =
            // meshHelper.pointsForTransform (bar, point1, point2);
            // AffineTransform3d transforms =
            // meshHelper.SVDRegistration (PFTMandible, PFTCyl);
            // bar.transform (transforms);
            //
            // connector = MeshFactory.getUnion (connector, bar.clone ());

            connectorBody = new FixedMeshBody ("Connector", connector);

         }
         ArrayList<Integer> loc0 = new ArrayList<Integer> ();
         ArrayList<Integer> loc1 = new ArrayList<Integer> ();
         ArrayList<Integer> loc2 = new ArrayList<Integer> ();
         ArrayList<Integer> loc3 = new ArrayList<Integer> ();
         ArrayList<Vector3d> newLocation = new ArrayList<Vector3d> ();
         ArrayList<PolygonalMesh> screws = new ArrayList<PolygonalMesh> ();
         ArrayList<FixedMeshBody> screwBody = new ArrayList<FixedMeshBody> ();
         // Adding ScrewHoles
         loc0.clear ();
         loc1.clear ();
         loc2.clear ();
         loc3.clear ();
         newLocation.clear ();
         screwBody.clear ();
         screws.clear ();

         System.out.println ("adding screwholes");
         OBB guideBB = masterGuideMesh.computeOBB ();

         Vector3d[] obaxis = new Vector3d[3];
         for (int j = 0; j < 3; j++) {
            obaxis[j] = new Vector3d ();
         }

         PolygonalMesh slot0 = new PolygonalMesh (OuterBox[0].clone ());
         slot0 = MeshFactory.getUnion (slot0, InnerBox[0].clone ());
         OBB slot0BB = slot0.computeOBB ();

         i = idP1;
         distance = 0;
         while (slot0BB
            .containsPoint (PlateMesh.getVertex (i).getPosition ())) {
            if (i <= 0) {
               i = 0;
               break;
            }
            else {
               i--;
            }
         }

         while (distance < 3) {
            if (i <= 0) {
               i = 0;
               break;
            }
            else {
               v = PlateMesh.getVertex (i - 1).getPosition ();
               distance += v.distance (PlateMesh.getVertex (i).getPosition ());
               i--;
            }
         }
         int start0 = i;

         i = idP1;
         distance = 0;
         while (slot0BB
            .containsPoint (PlateMesh.getVertex (i).getPosition ())) {
            if (i >= (numVert - 2)) {
               i = numVert - 2;
               break;
            }
            else {
               i++;
            }
         }

         while (distance < 3) {
            if (i >= (numVert - 2)) {
               i = numVert - 2;
               break;
            }
            else {
               v = PlateMesh.getVertex (i + 1).getPosition ();
               distance += v.distance (PlateMesh.getVertex (i).getPosition ());
               i++;
            }
         }
         int start1 = i;

         PolygonalMesh slot1 = new PolygonalMesh (OuterBox[1].clone ());
         slot1.addMesh (InnerBox[1].clone ());
         OBB slot1BB = slot1.computeOBB ();

         i = idP2;
         distance = 0;
         while (slot1BB
            .containsPoint (PlateMesh.getVertex (i).getPosition ())) {
            if (i <= 0) {
               i = 0;
               break;
            }
            else {
               i--;
            }
         }

         while (distance < 3) {
            if (i <= 0) {
               i = 0;
               break;
            }
            else {
               v = PlateMesh.getVertex (i - 1).getPosition ();
               distance += v.distance (PlateMesh.getVertex (i).getPosition ());
               i--;
            }
         }
         int start2 = i;

         i = idP2;
         distance = 0;
         while (slot1BB
            .containsPoint (PlateMesh.getVertex (i).getPosition ())) {
            if (i >= (numVert - 2)) {
               i = numVert - 2;
               break;
            }
            else {
               i++;
            }
         }

         while (distance < 3) {
            if (i >= (numVert - 2)) {
               i = numVert - 2;
               break;
            }
            else {
               v = PlateMesh.getVertex (i + 1).getPosition ();
               distance += v.distance (PlateMesh.getVertex (i).getPosition ());
               i++;
            }
         }
         int start3 = i;

         // Finding Screw Positions to the side of GuideTrimPlane0
         int numScrew0 = (int)(distance0) / 4;
         if ((distance0 % 4) < 4) {
            numScrew0 = numScrew0 - 1;
         }
         distance = 0;
         i = start0;

         ArrayList<Point3d> locations0 = new ArrayList<Point3d> ();
         ArrayList<Vector3d> normal0 = new ArrayList<Vector3d> ();

         for (int l = 0; l < numScrew0; l++) {
            if (l != 0) {
               while ((distance < 3.5) && (i > 0) && (i < numVert)) {
                  if (i <= 0) {
                     break;
                  }
                  else {
                     v = PlateMesh.getVertex (i - 1).getPosition ();
                     distance +=
                        v.distance (PlateMesh.getVertex (i).getPosition ());
                     i--;
                  }
               }
            }

            boolean inside =
               guideBB
                  .containsPoint (
                     new Point3d (PlateMesh.getVertex (i).getPosition ()));
            if (inside) {
               locations0.add (l, PlateMesh.getVertex (i).getPosition ());
               loc0.add (i);
               normal0
                  .add (
                     new Vector3d (locations0.get (locations0.size () - 1))
                        .sub (PlateMesh.getVertex (i + 1).getPosition ()));
            }
            distance = 0;
         }

         // Finding Screws Locations to the side of plane1
         int numScrew1 = 0;
         if (screwsCheckBox.isSelected ()) {
            numScrew1 = (int)(distance1) / 4;
            if ((distance1 % 4) < 4) {
               numScrew1 = numScrew1 - 1;
            }
         }

         distance = 0;
         i = start1;
         ArrayList<Point3d> locations1 = new ArrayList<Point3d> ();
         ArrayList<Vector3d> normal1 = new ArrayList<Vector3d> ();

         for (int l = 0; l < numScrew1; l++) {
            if (l != 0) {
               while ((distance < 3.5) && (i > 0) && (i < numVert)) {
                  if (i >= (numVert - 2)) {
                     break;
                  }
                  else {
                     v = PlateMesh.getVertex (i + 1).getPosition ();
                     distance +=
                        v.distance (PlateMesh.getVertex (i).getPosition ());
                     i++;
                  }
               }
            }

            boolean inside =
               guideBB
                  .containsPoint (
                     new Point3d (PlateMesh.getVertex (i).getPosition ()));
            if (inside) {
               locations1.add (PlateMesh.getVertex (i).getPosition ());
               loc1.add (i);
               normal1
                  .add (
                     new Vector3d (locations1.get (locations1.size () - 1))
                        .sub (PlateMesh.getVertex (i + 1).getPosition ()));
            }
            distance = 0;

         }

         // Finding Screws Locations to the side of plane2
         int numScrew2 = 0;
         if (screwsCheckBox.isSelected ()) {
            numScrew2 = (int)(distance2) / 4;
            if ((distance2 % 4) < 4) {
               numScrew2 = numScrew2 - 1;
            }
         }
         distance = 0;
         i = start2;
         ArrayList<Point3d> locations2 = new ArrayList<Point3d> ();
         ArrayList<Vector3d> normal2 = new ArrayList<Vector3d> ();

         for (int l = 0; l < numScrew2; l++) {
            if (l != 0) {
               while ((distance < 3.5) && (i > 0) && (i < numVert)) {
                  if (i <= 0) {
                     break;
                  }
                  else {
                     v = PlateMesh.getVertex (i - 1).getPosition ();
                     distance +=
                        v.distance (PlateMesh.getVertex (i).getPosition ());
                     i--;
                  }
               }
            }
            boolean inside =
               guideBB
                  .containsPoint (
                     new Point3d (PlateMesh.getVertex (i).getPosition ()));
            if (inside) {
               locations2.add (PlateMesh.getVertex (i).getPosition ());
               loc2.add (i);
               normal2
                  .add (
                     new Vector3d (locations2.get (locations2.size () - 1))
                        .sub (PlateMesh.getVertex (i + 1).getPosition ()));
            }
            distance = 0;
         }

         // Finding Screws Locations to the side of plane3
         int numScrew3 = (int)(distance3) / 4;
         if ((distance3 % 4) < 4) {
            numScrew3 = numScrew3 - 1;
         }
         distance = 0;
         i = start3;
         ArrayList<Point3d> locations3 = new ArrayList<Point3d> ();
         ArrayList<Vector3d> normal3 = new ArrayList<Vector3d> ();

         for (int l = 0; l < numScrew3; l++) {
            if (l != 0) {
               while ((distance < 3.5) && (i > 0) && (i < numVert)) {
                  if (i >= (numVert - 2)) {
                     break;
                  }
                  else {
                     v = PlateMesh.getVertex (i + 1).getPosition ();
                     distance +=
                        v.distance (PlateMesh.getVertex (i).getPosition ());
                     i++;
                  }
               }
            }

            boolean inside =
               guideBB
                  .containsPoint (
                     new Point3d (PlateMesh.getVertex (i).getPosition ()));
            if (inside) {
               locations3.add (PlateMesh.getVertex (i).getPosition ());
               loc3.add (i);
               normal3
                  .add (
                     new Vector3d (locations3.get (locations3.size () - 1))
                        .sub (PlateMesh.getVertex (i + 1).getPosition ()));
            }
            distance = 0;
         }

         // calculating pose plane for finding contour and combining
         // locations(i) into variable loc
         ArrayList<RigidTransform3d> planepose =
            new ArrayList<RigidTransform3d> ();
         ArrayList<Vector3d> loc = new ArrayList<Vector3d> ();
         for (int l = 0; l < loc0.size (); l++) {
            planepose.add (l, new RigidTransform3d ());
            RotationMatrix3d r =
               meshHelper.rotatePlane (new Vector3d (0, 0, 1), normal0.get (l));
            planepose.get (l).setRotation (r);
            planepose.get (l).setTranslation (locations0.get (l));
            loc.add (l, new Vector3d (locations0.get (l)));
         }

         int upto = loc0.size () + loc1.size ();
         int upless = loc0.size ();
         for (int l = upless; l < (upto); l++) {
            planepose.add (l, new RigidTransform3d ());
            RotationMatrix3d r =
               meshHelper
                  .rotatePlane (
                     new Vector3d (0, 0, 1), normal1.get (l - upless));
            planepose.get (l).setRotation (r);
            planepose.get (l).setTranslation (locations1.get (l - upless));
            loc.add (l, new Vector3d (locations1.get (l - upless)));
         }

         upto = upto + loc2.size ();
         upless = upless + loc1.size ();

         for (int l = upless; l < (upto); l++) {
            planepose.add (l, new RigidTransform3d ());
            RotationMatrix3d r =
               meshHelper
                  .rotatePlane (
                     new Vector3d (0, 0, 1), normal2.get (l - upless));
            planepose.get (l).setRotation (r);
            planepose.get (l).setTranslation (locations2.get (l - upless));
            loc.add (l, new Vector3d (locations2.get (l - upless)));
         }

         upto = upto + loc3.size ();
         upless = upless + loc2.size ();
         for (int l = upless; l < (upto); l++) {
            planepose.add (l, new RigidTransform3d ());
            RotationMatrix3d r =
               meshHelper
                  .rotatePlane (
                     new Vector3d (0, 0, 1), normal3.get (l - upless));
            planepose.get (l).setRotation (r);
            planepose.get (l).setTranslation (locations3.get (l - upless));
            loc.add (l, new Vector3d (locations3.get (l - upless)));
         }

         // Creating plane for finding contour
         ArrayList<Plane> screwPlanes = new ArrayList<Plane> ();
         for (int l = 0; l < (loc0.size () + loc1.size () + loc2.size ()
         + loc3.size ()); l++) {
            screwPlanes.add (l, new Plane ());
            screwPlanes.get (l).set (planepose.get (l));
         }

         // Finding contour between plane & guide and plane & mandible at each
         // of the screw locations
         // LengthDir of Screw is axis[1] of plane & mandible contour's bounding
         // box
         // Position of Screw is centroid of plane & guide contour's bounding
         // box + axis[0] of plane & mandible contour's bounding box
         // Where axis[0] is scaled by largest width of plane & guide contour's
         // bounding box divided by 6

         ArrayList<Vector3d> screwLengthDir = new ArrayList<Vector3d> ();
         for (int l = 0; l < (loc0.size () + loc1.size () + loc2.size ()
         + loc3.size ()); l++) {
            BVIntersector intersect = new BVIntersector ();
            ArrayList<LinkedList<Point3d>> contour =
               new ArrayList<LinkedList<Point3d>> ();
            contour =
               intersect
                  .intersectMeshPlane (
                     masterGuideMesh.clone (), screwPlanes.get (l), 0.01);
            ArrayList<PolygonalMesh> contourScrews =
               new ArrayList<PolygonalMesh> ();
            for (int h = 0; h < contour.size (); h++) {
               contourScrews.add (h, new PolygonalMesh ());
               for (int g = 0; g < contour.get (h).size (); g++) {
                  Vertex3d newVert = new Vertex3d (contour.get (h).get (g));
                  contourScrews.get (h).addVertex (newVert);
               }
            }
            ArrayList<LinkedList<Point3d>> contourMand =
               new ArrayList<LinkedList<Point3d>> ();
            contourMand =
               intersect
                  .intersectMeshPlane (
                     Assist.GetMesh (mechModel, "Mandible").clone (),
                     screwPlanes.get (l), 0.01);
            ArrayList<PolygonalMesh> contourMandible =
               new ArrayList<PolygonalMesh> ();
            for (int h = 0; h < contourMand.size (); h++) {
               contourMandible.add (h, new PolygonalMesh ());
               for (int g = 0; g < contourMand.get (h).size (); g++) {
                  Vertex3d newVert = new Vertex3d (contourMand.get (h).get (g));
                  contourMandible.get (h).addVertex (newVert);
               }
            }

            PolygonalMesh chosenContourScrew = new PolygonalMesh ();
            FixedMeshBody chosenContourScrewBody = new FixedMeshBody ();
            closestcentroid = 100000;
            for (int h = 0; h < contour.size (); h++) {
               Point3d contourcentroid = new Point3d ();
               contourScrews.get (h).computeCentroid (contourcentroid);
               FixedMeshBody contourBody =
                  new FixedMeshBody ("contour", contourScrews.get (h));
               X = contourBody.getPose ();
               contourcentroid.transform (X);
               double distanceCentroid = contourcentroid.distance (loc.get (l));
               OBB contourOBB = contourScrews.get (h).computeOBB ();
               Vector3d cw = new Vector3d ();
               contourOBB.getWidths (cw);
               double area = meshHelper.ComputeContourArea (cw);
               if ((distanceCentroid < closestcentroid) && (area > 50)) {
                  closestcentroid = distanceCentroid;
                  chosenContourScrew =
                     new PolygonalMesh (contourScrews.get (h));
                  chosenContourScrewBody =
                     new FixedMeshBody (
                        "ContourGuide" + String.valueOf (l),
                        chosenContourScrew);
               }
            }

            OBB contourscrewOBB = chosenContourScrew.computeOBB ();
            Vector3d[] cAxis = new Vector3d[3];
            for (int g = 0; g < 3; g++) {
               cAxis[g] = new Vector3d ();
            }
            contourscrewOBB.getSortedAxes (cAxis);
            RigidTransform3d posecontour = chosenContourScrewBody.getPose ();
            Point3d centroidContour = new Point3d ();
            chosenContourScrew.computeCentroid (centroidContour);
            centroidContour.transform (posecontour);
            newLocation.add (l, new Vector3d (centroidContour));
            Vector3d widthscontour = new Vector3d ();
            contourscrewOBB.getWidths (widthscontour);
            double value = 0;
            int maxID = 0;
            for (int g = 0; g < 3; g++) {
               if (widthscontour.get (g) > value) {
                  maxID = g;
                  value = widthscontour.get (g);
               }
            }
            double scaleFactor = widthscontour.get (maxID) / 6;

            PolygonalMesh chosenContourMand = new PolygonalMesh ();
            FixedMeshBody chosenContourMandBody = new FixedMeshBody ();
            closestcentroid = 100000;
            for (int h = 0; h < contourMand.size (); h++) {
               Point3d contourcentroid = new Point3d ();
               contourMandible.get (h).computeCentroid (contourcentroid);
               FixedMeshBody contourBody =
                  new FixedMeshBody ("contour", contourMandible.get (h));
               X = contourBody.getPose ();
               contourcentroid.transform (X);
               double distanceCentroid = contourcentroid.distance (loc.get (l));
               OBB contourOBB = contourMandible.get (h).computeOBB ();
               Vector3d cw = new Vector3d ();
               contourOBB.getWidths (cw);
               double area = meshHelper.ComputeContourArea (cw);
               if ((distanceCentroid < closestcentroid) && (area > 100)) {
                  closestcentroid = distanceCentroid;
                  chosenContourMand =
                     new PolygonalMesh (contourMandible.get (h));
                  chosenContourMandBody =
                     new FixedMeshBody (
                        "ContourMand" + String.valueOf (l), chosenContourMand);
               }
            }

            OBB contourmandOBB = chosenContourMand.computeOBB ();
            Vector3d[] cAxism = new Vector3d[3];
            for (int g = 0; g < 3; g++) {
               cAxism[g] = new Vector3d ();
            }
            contourmandOBB.getSortedAxes (cAxism);
            posecontour = chosenContourMandBody.getPose ();
            screwLengthDir.add (l, new Vector3d (cAxism[1]));
            screwLengthDir.get (l).transform (posecontour);
            Vector3d upext = new Vector3d (cAxism[0]);
            Vector3d upDirRef = new Vector3d ();
            if (l < (numScrew0 + numScrew1)) {
               upDirRef = new Vector3d (plane1dir);
               if (upDirRef.dot (upext) > 0) {
                  upext.negate ();
               }
            }
            else {
               upDirRef = new Vector3d (plane2dir);
               if (upDirRef.dot (upext) > 0) {
                  upext.negate ();
               }
            }
            upext.transform (posecontour);
            upext.normalize ().scale (scaleFactor);
            newLocation.get (l).add (upext);
            Point3d mandcentroidContour = new Point3d ();
            chosenContourMand.computeCentroid (mandcentroidContour);
            mandcentroidContour.transform (posecontour);
         }

         // Checking if there's any NaN in the list of screw's locations and
         // remove it
         ArrayList<Integer> nan = new ArrayList<Integer> ();
         for (int l = 0; l < newLocation.size (); l++) {
            double n1 = newLocation.get (l).get (0);
            double n2 = newLocation.get (l).get (1);
            double n3 = newLocation.get (l).get (2);
            if ((Double.isNaN (n1)) || (Double.isNaN (n2))
            || (Double.isNaN (n3))) {
               nan.add (l);
            }
         }

         int l0 = loc0.size ();
         int l1 = loc1.size ();
         int l2 = loc2.size ();
         for (int l = 0; l < nan.size (); l++) {
            newLocation.remove (l);
            screwLengthDir.remove (l);
            if (l < l0) {
               loc0.remove (l);
               locations0.remove (l);
            }
            else if (l < (l0 + l1)) {
               loc1.remove (l - l0);
               locations1.remove (l - l0);
            }
            else if (l < (l0 + l1 + l2)) {
               loc2.remove ((l - (l0 + l1)));
               locations2.remove ((l - (l0 + l1)));
            }
            else {
               loc3.remove ((l - (l0 + l1 + l2)));
               locations3.remove ((l - (l0 + l1 + l2)));
            }
         }

         SurfaceMeshIntersector intersect = new SurfaceMeshIntersector ();
         PolygonalMesh base1 = intersect.findDifference01 (masterGuideMesh.clone (), GuideTrim[0]);

         PolygonalMesh base2 = null;
         if (!oneHolderOption.isSelected ()) {
            base1 = intersect.findDifference01 (base1.clone (), GuideTrim[1]);
            base2  =
               intersect
                  .findDifference01 (masterGuideMesh.clone (), GuideTrim[2]);
            base2 = intersect.findDifference01 (base2.clone (), GuideTrim[3]);
         }
         else {
            base1 = intersect.findDifference01 (base1.clone (), GuideTrim[3]);
         }

         FixedMeshBody base1Body = new FixedMeshBody ("Base1", base1);
         FixedMeshBody base2Body =null;
         if (!oneHolderOption.isSelected ()) {
            base2Body  = new FixedMeshBody ("Base2", base2);
         }

         Assist.GetMesh (mechModel, "Master Mandible").getRenderProps ().setVisible (false);

         System.out.println ("slot0Body has pose " + OutBody[0].getPose ());
         Vector3d centroidSlot0 = new Vector3d ();
         OuterBox[0].computeCentroid (centroidOut0);
         System.out.println ("Slot0 has centroid " + centroidSlot0);

         MandibleGuidePrimitiveCommand cmd =
            new MandibleGuidePrimitiveCommand (
               "Create Mandible Guide Primitives", OutBody, InBody,
               connectorBody, screwBody, base1Body, base2Body, Assist.GetMeshBody (mechModel, "Master Guide"),
               mechModel, connectorOption.isSelected (),
               oneHolderOption.isSelected ());

         Main main = Main.getMain ();
         UndoManager undo = main.getUndoManager ();
         undo.execute (cmd);

         root.getMainViewer ().autoFit ();
      }
   }