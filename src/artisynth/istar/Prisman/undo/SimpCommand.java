package artisynth.istar.Prisman.undo;

import java.util.ArrayList;
import java.util.Iterator;

import artisynth.core.gui.editorManager.Command;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.istar.Assist.Assist;
import artisynth.istar.Prisman.HelperMathFunctions;
import artisynth.istar.Prisman.HelperMeshFunctions;
import artisynth.istar.Prisman.ImprovedFormattedTextField;
import artisynth.istar.Prisman.NumericListSimplification;
import artisynth.istar.Prisman.ReconstructionModel;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.MeshFactory;
import maspack.geometry.OBB;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.PolylineMesh;
import maspack.geometry.Vertex3d;
import maspack.interpolation.NumericList;
import maspack.interpolation.NumericListKnot;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer.LineStyle;

public class SimpCommand implements Command {
   private String myName;
   ImprovedFormattedTextField rdpMinDistance;
   ImprovedFormattedTextField rdpMaxSegments;
   MechModel mechModel = null;
   ReconstructionModel root = null;
   Assist Assist = new Assist ();
   HelperMathFunctions mathHelper = new HelperMathFunctions ();
   HelperMeshFunctions meshHelper = new HelperMeshFunctions ();

   public SimpCommand (ReconstructionModel root, MechModel mechModel,
   ImprovedFormattedTextField rdpMinDistance,
   ImprovedFormattedTextField rdpMaxSegments) {
      myName = "Simplify RDP";
      this.rdpMaxSegments = rdpMaxSegments;
      this.rdpMinDistance = rdpMinDistance;
      this.mechModel = mechModel;
      this.root = root;
   }

   public void execute () {
      // Remove RDP line if it already exists
      if (Assist.GetMeshBody (mechModel, "Clipped Plate") != null) {
         mechModel
            .removeMeshBody (Assist.GetMeshBody (mechModel, "Clipped Plate"));
      }

      FixedMeshBody plane1MeshBody = Assist.GetMeshBody (mechModel, "Plane1");
      PolygonalMesh plane1Mesh = (PolygonalMesh)plane1MeshBody.getMesh ();

      // Find some data on the planes
      Point3d c1 = new Point3d ();
      plane1Mesh.computeCentroid (c1);
      Vector3d planenormal1 = new Vector3d (plane1Mesh.getNormal (0));
      System.out.println (planenormal1.x + ", " + planenormal1.y + ", " + planenormal1.z); //
      RigidTransform3d pose = plane1MeshBody.getPose ();
      Point3d centroid1 = new Point3d (c1);
      centroid1.transform (pose);
      planenormal1.transform (pose);
      
      //
      System.out.println (planenormal1.x + ", " + planenormal1.y + ", " + planenormal1.z); 
      PolygonalMesh newMesh = MeshFactory.createPlane (90, 90, 10, 10);
      
      meshHelper.createPlane 
         (newMesh.getNormal (0), new Vector3d (planenormal1), centroid1, pose, newMesh);
      //

      FixedMeshBody plane2MeshBody = Assist.GetMeshBody (mechModel, "Plane2");
      PolygonalMesh plane2Mesh = (PolygonalMesh)plane2MeshBody.getMesh ();

      Point3d c2 = new Point3d ();
      plane2Mesh.computeCentroid (c2);
      Vector3d planenormal2 = new Vector3d (plane2Mesh.getNormal (0));
      pose = plane2MeshBody.getPose ();
      Point3d centroid2 = new Point3d (c2);
      centroid2.transform (pose);
      planenormal2.transform (pose);

      ArrayList<PolygonalMesh> implantList = meshHelper.getImplantList (mechModel);
      
      NumericList generalNumericList = new NumericList (3);
      if (!implantList.isEmpty ()) {
         generalNumericList = root.dentalImplantList;
      }
      else {
         generalNumericList = root.plateNumericList; // All references to
         // plateNumericList replaced
         // with general NumericList
      }
      
      // Get first and last point
      NumericListKnot knot1 =
         mathHelper
            .closestNumericListKnotToPlane (
               planenormal1, centroid1, generalNumericList);

      NumericListKnot knot2 =
         mathHelper
            .closestNumericListKnotToPlane (
               planenormal2, centroid2, generalNumericList);

      // Creating new numericList to only contain knots between the bounds
      // Extending the knots be 1 on eachside so that the polyline is a
      // little past the plane, for csg
      NumericListKnot lowestKnot = new NumericListKnot (knot1.getPrev ());
      NumericListKnot highestKnot = new NumericListKnot (knot2.getNext ());
      if (knot2.t < lowestKnot.t) {
         lowestKnot = new NumericListKnot (knot2.getPrev ());
         highestKnot = new NumericListKnot (knot1.getNext ());
      }

      // Only knots between the bounds are copied to curatedList
      NumericList curatedList = new NumericList (3);
      Iterator<NumericListKnot> itr = generalNumericList.iterator ();
      NumericListKnot tempKnot;
      while (itr.hasNext ()) {
         tempKnot = new NumericListKnot (itr.next ());
         if ((tempKnot.t > lowestKnot.t) && (tempKnot.t < highestKnot.t)) {
            curatedList.add (new NumericListKnot (tempKnot));
         }
      }

      // Creating a numericList that is the result of the line simplification
      NumericListSimplification simplifier = new NumericListSimplification ();
      NumericList simpList = new NumericList (3);

      Integer distance = Integer.parseInt(rdpMinDistance.getText ());
      Integer segments = Integer.parseInt(rdpMaxSegments.getText ());
      simplifier
         .bisectSimplifyDouglasPeucker (
            curatedList, distance,
            segments, simpList);
      
      
//      
      
//      //1 segment recon
//      simpList.add (knot1);   
//      simpList.add (knot2);
//      //1 segment recon
      
//      //2 segment recon
//      simpList.add (knot1);
//    
//      NumericListKnot[] arrayKnots = new NumericListKnot[curatedList.getNumKnots ()];
//      Iterator<NumericListKnot> iterator = curatedList.iterator ();
//      int index = 0;
//      while (iterator.hasNext ()) {
//         arrayKnots[index] = iterator.next ();
//         index++;
//      }
//    
//      int s = curatedList.getNumKnots ();
//      int g = s / 2;
//    
//      System.out.print (simpList);
//      simpList.add (new NumericListKnot(arrayKnots[g]));
//      System.out.print (simpList);
//    
//      simpList.add (knot2);
//      //2 segment recon
      
//      //3 segment recon
//      simpList.add (knot1);
//      
//      NumericListKnot[] arrayKnots = new NumericListKnot[curatedList.getNumKnots ()];
//      Iterator<NumericListKnot> iterator = curatedList.iterator ();
//      int index = 0;
//      while (iterator.hasNext ()) {
//         arrayKnots[index] = iterator.next ();
//         index++;
//      }
//      
//      int s = curatedList.getNumKnots ();
//      int g = s / 3;
//      int y = 2 * s / 3;
//      
//      System.out.print (simpList);
//      simpList.add (new NumericListKnot(arrayKnots[g]));
//      simpList.add (arrayKnots[y].v, arrayKnots[y].t);
//      System.out.print (simpList);
//      
//      simpList.add (knot2);
//      //3 segment recon
      
//      //4 segment recon
//      simpList.add (knot1);
//      
//      NumericListKnot[] arrayKnots = new NumericListKnot[curatedList.getNumKnots ()];
//      Iterator<NumericListKnot> iterator = curatedList.iterator ();
//      int index = 0;
//      while (iterator.hasNext ()) {
//         arrayKnots[index] = iterator.next ();
//         index++;
//      }
//      
//      int s = curatedList.getNumKnots ();
//      int g = s / 4;
//      int h = s / 2;
//      int f = 3 * s / 4;
//      
//      simpList.add (new NumericListKnot (arrayKnots[g]));
//      simpList.add (new NumericListKnot (arrayKnots[h]));
//      simpList.add (new NumericListKnot (arrayKnots[f]));
//
//      simpList.add (knot2);
//      //4 segment recon
      
//      //5 segment recon
//      simpList.add (knot1);
//      
//      NumericListKnot[] arrayKnots = new NumericListKnot[curatedList.getNumKnots ()];
//      Iterator<NumericListKnot> iterator = curatedList.iterator ();
//      int index = 0;
//      while (iterator.hasNext ()) {
//         arrayKnots[index] = iterator.next ();
//         index++;
//      }
//      
//      int s = curatedList.getNumKnots ();
//      int g = s / 5;
//      int h = 2 * s / 5;
//      int f = 3 * s / 5;
//      int e = 4 * s / 5;
//      
//      simpList.add (new NumericListKnot (arrayKnots[g]));
//      simpList.add (new NumericListKnot (arrayKnots[h]));
//      simpList.add (new NumericListKnot (arrayKnots[f]));
//      simpList.add (new NumericListKnot (arrayKnots[e]));
//
//      
//      simpList.add (knot2);
//      //5 segment recon

      int size = simpList.getNumKnots ();
      root.numberOfSegments = size - 1;
      
      System.out.println (size);

      Point3d[] curatedPoints = new Point3d[size];
      int[][] indices = new int[root.numberOfSegments][2];

      Iterator<NumericListKnot> simpItr = simpList.iterator ();
      int i = 0;
      while (simpItr.hasNext ()) {
         curatedPoints[i] = new Point3d (simpItr.next ().v);
         i++;
      }

      for (i = 0; i < root.numberOfSegments; i++) {
         indices[i][0] = i;
         indices[i][1] = i + 1;
      }

      if (root.DEBUG) {
         System.out.println ("Indices Length: " + indices.length);
         System.out.println ("Points Length: " + curatedPoints.length);
      }
            
      PolylineMesh sphericalPolyLine =
         MeshFactory.createSphericalPolyline (50, 50, 50);

      PolylineMesh rdpMesh = new PolylineMesh ();
      rdpMesh.addMesh (sphericalPolyLine);
      rdpMesh.set (curatedPoints, indices);
      FixedMeshBody rdpMeshBody = new FixedMeshBody ("Clipped Plate", rdpMesh);

      Point3d[] simpPoints = new Point3d[simpList.getNumKnots ()];
      simpItr = simpList.iterator ();
      int l = 0;
      while (simpItr.hasNext ()) {
         simpPoints[l] = new Point3d (simpItr.next ().v);
         l++;
      }
      
      mechModel.addMeshBody (rdpMeshBody);
      RenderProps.setLineStyle (rdpMeshBody, LineStyle.CYLINDER);
      root.simpList = (NumericList)simpList.clone ();
      
//      
//      FrameMarker[] frameMarkers = null;
//      frameMarkers =
//         Assist.GetRigidBody (mechModel, "Mandible").getFrameMarkers ();
//      Point3d[] changPoints = new Point3d[6];
//      PolygonalMesh negativeMand = Assist.GetMesh (mechModel, "Mandible");
//      ArrayList<Point3d> reconstructedPoints = new ArrayList<Point3d> ();
//      
//      ArrayList<Vertex3d> vertices = rdpMesh.copy().getVertices ();    
//      ArrayList<Point3d> vertexPoints = new ArrayList<Point3d> ();
//      
//      for (int m = 0; m < vertices.size (); m++) {
//         vertexPoints.add (vertices.get (m).getPosition ());
//      }
//      
//      System.out.println ("CHECKING");
//      
//      for (int m = 0; m < 6; m++) {
//         changPoints[m] = frameMarkers[frameMarkers.length - m - 1].getPosition ();
//         double changDist = negativeMand.distanceToPoint (changPoints[m]);
//         
//         System.out.println ("CHANG DISTANCE: " + changDist);
//         
//         if (changDist > 0.0001) {
//            reconstructedPoints.add (changPoints[m]);
//         }
//      }
//      
//      System.out.println ("CHECKING AGAIN");
//      
//      //add points from where the cuts were made
//      reconstructedPoints.add (simpPoints[simpPoints.length - 1]);
//      reconstructedPoints.add (simpPoints[0]);
//      //get pre-reconstructed angles
//      //to get recon angles, use points on RDP mesh closest to the chang Points
//      ArrayList<Point3d> currPoints = new ArrayList<Point3d> ();
//      BVFeatureQuery query = new BVFeatureQuery();
//      
//      for (int j = 1; j < reconstructedPoints.size() - 1; j++) {
//         Point3d prev = reconstructedPoints.get (j - 1);
//         Point3d p = reconstructedPoints.get (j);
//         Point3d next = reconstructedPoints.get (j + 1);
//         currPoints.add (next);
//         currPoints.add (prev);
//         currPoints.add (p);
//         
//         Vector3d dist1 = prev.sub (p);
//         Vector3d dist2 = next.sub (p);
//         
//         double costheta = (dist1.dot (dist2)) / (dist1.norm () * dist2.norm ());
//         double angle = Math.acos (costheta);
//         angle = 180 - Math.toDegrees (angle);
//         
//         System.out.println ("Previous angle at point " + j + ": " + angle);
//         
//         Vertex3d vtxPrev = query.nearestVertexToPoint (rdpMesh, prev);
//         Vertex3d vtxP = query.nearestVertexToPoint (rdpMesh, p);
//         Vertex3d vtxNext = query.nearestVertexToPoint (rdpMesh, next);
//
//         Point3d nearPnt = new Point3d ();
//         query.nearestEdgeToPoint (nearPnt, null, rdpMesh, p);
//         double nearest = p.distance (nearPnt);
//         System.out.println ("Distance from reconstructed point " + j + ": " + nearest);
//         
//         ArrayList<Point3d> reconPoints = new ArrayList<Point3d> ();
//         for (int n = 0; n < 3; n++) {
//            double minDistance = Double.MAX_VALUE;
//            Point3d temp = currPoints.get (n);
//            Point3d reconPoint = new Point3d ();
//            for (int o = 0; o < vertexPoints.size (); o++) {
//               Vector3d distCalc = temp.sub (vertexPoints.get (o));
//               double tempDist = distCalc.norm ();
//               if (tempDist < minDistance) {
//                  minDistance = tempDist;
//                  reconPoint.set(vertexPoints.get (o));
//               }
//            }
//            if (n == 1) {
//               System.out.println ("Distance from reconstructed point " + j + ": " + minDistance);
//            }
//            reconPoints.add (reconPoint);
//         }
//         
//         Point3d reconPrev = reconPoints.get (0);
//         Point3d reconP = reconPoints.get (1);
//         Point3d reconNext = reconPoints.get (2);
//         
//         Vector3d reconDist1 = reconPrev.sub (reconP);
//         Vector3d reconDist2 = reconNext.sub (reconP);
//         
//         double reconCosTheta = 
//            (reconDist1.dot (reconDist2)) / (reconDist1.norm () * reconDist2.norm ());
//         double reconAngle = Math.acos (reconCosTheta);
//         reconAngle = 180 - Math.toDegrees (reconAngle);
//         
//         System.out.println ("Recon angle at point " + j + ": " + reconAngle);
//         
//         currPoints.clear ();
//         reconPoints.clear ();
//      }
//      /////////////////////////////////////////////////////////////////////////////////////////////
//      
//      //now go through arraylist and calculate angles between each of the points and compare to reconstruction
//     
//      
//      Point3d end = simpPoints[simpPoints.length - 1];
//      Point3d start = simpPoints[0];
//      
//      double simpBeginX = start.x;
//      double simpBeginY = start.y;
//      
//      double simpEndX = end.x;
//      double simpEndY = end.y;
//      
////      System.out.println ("Coordinates of start: " + start.x + " " + start.y);
////      
////      System.out.println ("Coordinates of end: " + end.x + " " + end.y);
//
//      
//      //get x and y coordinates of simpPoint list, compare to x and y of frame marker; if in the middle, 
//      //it's being reconstructed and we need to measure
//      
//      //j = 0, works up to 6. as long as frame markers length - j 
//      
//      for (int j = 0; j < 6; j++) {
//         Point3d p = frameMarkers[frameMarkers.length - j - 1].getPosition ();
//         double frameX = p.x;
//         double frameY = p.y;
//         
//         System.out.println ("Coordinates of point " + j + ": " + p.x + " " + p.y);
//
//         OBB resectBox = new OBB (Assist.GetMesh (mechModel, "Resection"));
//         if (!resectBox.containsPoint (p)) {
//               System.out.println ("Point " + j + " is being reconstructed");
//               
//               Vector3d distance1 = end.sub (p);
//               Vector3d distance2 = start.sub (p);
//               
//               double costheta = (distance1.dot (distance2)) / (distance1.norm () * distance2.norm ());
//               double angle = Math.acos (costheta);
//               angle = 180 - Math.toDegrees (angle);
//               
//               System.out.println ("Angle at point " + j + ": " + angle);
//               
//               //get distance between Chang point and RDP line point. Get angle at RDP line point as well. 
//               double min = Double.MAX_VALUE;
//               Point3d closestPoint = end;
//               for (int x = 0; x < simpPoints.length; x++) {
//                  Point3d temp = simpPoints[x];
//                  Vector3d tempDist = temp.sub (p);
//                  if (tempDist.norm () < min) {
//                     min = tempDist.norm ();
//                     closestPoint = temp;
//                  }
//               }
//               
//               System.out.println ("Displacement of point " + j + ": " + min);
//               
//               Vector3d distance3 = end.sub (closestPoint);
//               Vector3d distance4 = start.sub (closestPoint);
//               
//               double costheta1 = (distance3.dot (distance4)) / (distance3.norm () * distance4.norm ());
//               double angle1 = Math.acos (costheta1);
//               angle1 = 180 - Math.toDegrees (angle1);
//
//               System.out.println ("Reconstructed angle of point " + j + ": " + angle1);
//         }
//         
//         
//         if (frameX < simpEndX && frameY < simpEndY && 
//            frameX > simpBeginX && frameY > simpBeginY) {
//            System.out.println ("Point " + j + " is being reconstructed");
//            
//            Vector3d distance1 = end.sub (p);
//            Vector3d distance2 = start.sub (p);
//            
//            double costheta = (distance1.dot (distance2)) / (distance1.norm () * distance2.norm ());
//            double angle = Math.acos (costheta);
//            angle = 180 - Math.toDegrees (angle);
//            
//            System.out.println ("Angle at point " + j + ": " + angle);
//            
//            //get distance between Chang point and RDP line point. Get angle at RDP line point as well. 
//            double min = Double.MAX_VALUE;
//            Point3d closestPoint = end;
//            for (int x = 0; x < simpPoints.length; x++) {
//               Point3d temp = simpPoints[x];
//               Vector3d tempDist = temp.sub (p);
//               if (tempDist.norm () < min) {
//                  min = tempDist.norm ();
//                  closestPoint = temp;
//               }
//            }
//            
//            System.out.println ("Displacement of point " + j + ": " + min);
//            
//            Vector3d distance3 = end.sub (closestPoint);
//            Vector3d distance4 = start.sub (closestPoint);
//            
//            double costheta1 = (distance3.dot (distance4)) / (distance3.norm () * distance4.norm ());
//            double angle1 = Math.acos (costheta1);
//            angle1 = 180 - Math.toDegrees (angle1);
//
//            System.out.println ("Reconstructed angle of point " + j + ": " + angle1);
//         }
//         
//      }
      
      
//      for (i = 0; i < curatedPoints.length - 1; i++) {
//         Point3d p1 = curatedPoints[i];
//         Point3d p2 = curatedPoints[i + 1];
//         
//         Vector3d v = p1.add (p2);
//         System.out.println ("Length Segment " + i + ": " + v.norm ());
//      }
//
//      for (i = 1; i < curatedPoints.length - 1; i++) {
//         Point3d p1 = curatedPoints[i - 1];
//         Point3d p2 = curatedPoints[i];
//         Point3d p3 = curatedPoints[i + 1];
//         
//         Vector3d v1 = p1.add (p2);
//         Vector3d v2 = p3.add (p2);
//         
//         double costheta = (v1.dot (v2)) / (v1.norm () * v2.norm ());
//         double angle = Math.acos (costheta);
//         angle = 180 - Math.toDegrees (angle);
//         
//         System.out.println ("Angle at point " + i + ": " + angle);
//      }
   }

   public void undo () {
      if (Assist.GetMeshBody (mechModel, "Clipped Plate") != null) {
         mechModel
            .removeMeshBody (Assist.GetMeshBody (mechModel, "Clipped Plate"));
      }
   }

   public String getName () {
      return myName;
   }
}
