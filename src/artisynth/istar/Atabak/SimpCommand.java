package artisynth.istar.Atabak;

import java.util.Iterator;

import artisynth.core.gui.editorManager.Command;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;


import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.PolylineMesh;
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
      RigidTransform3d pose = plane1MeshBody.getPose ();
      Point3d centroid1 = new Point3d (c1);
      centroid1.transform (pose);
      planenormal1.transform (pose);

      FixedMeshBody plane2MeshBody = Assist.GetMeshBody (mechModel, "Plane2");
      PolygonalMesh plane2Mesh = (PolygonalMesh)plane2MeshBody.getMesh ();

      Point3d c2 = new Point3d ();
      plane2Mesh.computeCentroid (c2);
      Vector3d planenormal2 = new Vector3d (plane2Mesh.getNormal (0));
      pose = plane2MeshBody.getPose ();
      Point3d centroid2 = new Point3d (c2);
      centroid2.transform (pose);
      planenormal2.transform (pose);

      // Get first and last point
      NumericListKnot knot1 =
         mathHelper
            .closestNumericListKnotToPlane (
               planenormal1, centroid1, root.plateNumericList);

      NumericListKnot knot2 =
         mathHelper
            .closestNumericListKnotToPlane (
               planenormal2, centroid2, root.plateNumericList);

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
      Iterator<NumericListKnot> itr = root.plateNumericList.iterator ();
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
