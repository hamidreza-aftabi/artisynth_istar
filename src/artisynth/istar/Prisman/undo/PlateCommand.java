package artisynth.istar.Prisman.undo;

import java.util.ArrayList;
import java.util.Iterator;

import artisynth.core.gui.editorManager.Command;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.istar.Assist.Assist;
import artisynth.istar.Prisman.HelperMeshFunctions;
import artisynth.istar.Prisman.ReconstructionModel;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.PolylineMesh;
import maspack.interpolation.Interpolation;
import maspack.interpolation.Interpolation.Order;
import maspack.interpolation.NumericList;
import maspack.interpolation.NumericListKnot;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.render.RenderProps;
import maspack.render.Renderer.LineStyle;

public class PlateCommand implements Command {
   private String myName;
   // private FixedMeshBody myPlate;
   private MechModel mechModel;
   private ReconstructionModel root;
   private String RigidBodyName;
   Assist Assist = new Assist ();
   HelperMeshFunctions meshHelper = new HelperMeshFunctions ();

   public PlateCommand (ReconstructionModel root, MechModel mechModel, String RigidBodyName) {
      myName = "Plate Mandible";
      this.mechModel = mechModel;
      this.root = root;
      this.RigidBodyName = RigidBodyName;
   }

   public void execute () {
      // plates = new FixedMeshBody ("Plate", myPlate);
//      NumericList plateNumericList =
//         (NumericList)root.plateNumericList.clone ();
      int numberOfSubdivisions = 5000;
      // int numberOfSubdivisions = 500;
      FrameMarker[] frameMarkers = null;
      mechModel.remove (Assist.GetMeshBody (mechModel, "Plate"));
      frameMarkers =
         Assist.GetRigidBody (mechModel, RigidBodyName).getFrameMarkers ();      
      int size = frameMarkers.length - 6;
      // System.out.println (size);
      NumericList plateNumericList = new NumericList (3);
      Interpolation cubic = new Interpolation ();
      cubic.setOrder (Order.SphericalCubic);
      plateNumericList.setInterpolation (cubic);

      Point3d[] points = new Point3d[size];
      for (int i = 0; i < size; i++) {
         points[i] = frameMarkers[i].getPosition ();
         plateNumericList
            .add (points[i], i * numberOfSubdivisions / (size - 1));

      }
      VectorNd[] interpolatedVectors = new VectorNd[numberOfSubdivisions];

      for (int i = 0; i < numberOfSubdivisions; i++) {
         interpolatedVectors[i] = new VectorNd ();
         interpolatedVectors[i].setSize (3);
         plateNumericList.interpolate (interpolatedVectors[i], i);

      }

      // Adding Results of interpolation to the numericList
      for (int i = 0; i < numberOfSubdivisions; i++) {
         plateNumericList.add (interpolatedVectors[i], i);
      }

      int newSize = plateNumericList.getNumKnots ();
      Point3d[] interpolatedPoints = new Point3d[newSize];
      int[][] indices = new int[newSize - 1][2];

      Iterator<NumericListKnot> itr = plateNumericList.iterator ();

      int i = 0;
      while (itr.hasNext ()) {
         interpolatedPoints[i] = new Point3d (itr.next ().v);
         i++;
      }

      for (i = 0; i < newSize - 1; i++) {
         indices[i][0] = i;
         indices[i][1] = i + 1;
      }
      
      
      ArrayList<PolygonalMesh> implantList = meshHelper.getImplantList (mechModel);
      
      if (!implantList.isEmpty ()) { 
       
         Integer[] dentalImplantsRawOrder = meshHelper.getRawOrderDental 
            (implantList, interpolatedPoints);
       
         Vector3d[] dentalImplantIntersections = meshHelper.getDentIntersections 
            (implantList, interpolatedPoints);

         Integer[] orderedDentalImplantIndices = meshHelper.orderImplantIndeces 
            (dentalImplantsRawOrder, implantList);
       
         Vector3d[] dentalImplantIntersectionsOrdered = meshHelper.orderImplantIntersections 
            (dentalImplantIntersections, orderedDentalImplantIndices, implantList);
            
         NumericList implantNumericList = meshHelper.getImplantNumeric 
            (dentalImplantIntersectionsOrdered, interpolatedVectors, numberOfSubdivisions, 
               implantList);
                         
         int newSizeImplants = implantNumericList.getNumKnots ();
         Point3d[] interpolatedPointsImplants = new Point3d[newSizeImplants];
         int[][] indicesImplants = new int[newSizeImplants - 1][2];
       
         Iterator<NumericListKnot> itrImplants =
            implantNumericList.iterator ();

         i = 0;
         while (itrImplants.hasNext ()) {
            interpolatedPointsImplants[i] =
               new Point3d (itrImplants.next ().v);
            i++;
         }
       
         for (i = 0; i < newSizeImplants - 1; i++) {
            indicesImplants[i][0] = i;
            indicesImplants[i][1] = i + 1;
         }
         PolylineMesh sphericalPolyLineImplant =
            MeshFactory.createSphericalPolyline (50, 50, 50);
       
         PolylineMesh implantPlateMesh = new PolylineMesh ();
         implantPlateMesh.addMesh (sphericalPolyLineImplant);
         implantPlateMesh.set (interpolatedPointsImplants, indicesImplants);

         FixedMeshBody implantPlate =
            new FixedMeshBody ("ImplantPlate", implantPlateMesh);
         mechModel.addMeshBody (implantPlate);
         root.dentalImplantList = (NumericList)implantNumericList.clone ();
         
      }
      else {
         PolylineMesh sphericalPolyLine =
            MeshFactory.createSphericalPolyline (50, 50, 50);

         PolylineMesh PlateMesh = new PolylineMesh ();
         // PlateMesh2 = new PolygonalMesh();
         PlateMesh.addMesh (sphericalPolyLine);
         PlateMesh.set (interpolatedPoints, indices);
         //
         // try {
         // GenericMeshWriter.writeMesh ("Simplified.stl", PlateMesh2);
         // }
         // catch (IOException e) {
         // // TODO Auto-generated catch block
         // e.printStackTrace();
         // }

         FixedMeshBody plate = new FixedMeshBody ("Plate", PlateMesh);
         mechModel.addMeshBody (plate);
         RenderProps.setLineStyle (plate, LineStyle.CYLINDER);
         root.plateNumericList = (NumericList)plateNumericList.clone ();      
      
      }
   }

   public void undo () {
      mechModel.removeMeshBody (Assist.GetMeshBody (mechModel, "Plate"));
   }

   public String getName () {
      return myName;
   }
}
