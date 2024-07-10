package artisynth.istar.MandibleRecon;

import java.awt.event.ActionEvent;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import javax.swing.AbstractAction;

import artisynth.istar.Prisman.HelperMeshFunctions;
import artisynth.istar.Prisman.ReconstructionModel;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.istar.Prisman.ReconstructionModel;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.io.GenericMeshWriter;
import maspack.interpolation.NumericListKnot;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;

public class createFibulaScrewsButtonClicked extends AbstractAction {
   ReconstructionModel root = null;
   MechModel mechModel = null;
   HelperMeshFunctions meshHelper = new HelperMeshFunctions ();


   public createFibulaScrewsButtonClicked (ReconstructionModel root,
   MechModel mechModel) {
      putValue (NAME, "Add Fibula Screws");
      this.root = root;
      this.mechModel = mechModel;
   }

   @Override
   public void actionPerformed (ActionEvent evt) {
      ArrayList<FixedMeshBody> screwBodies = new ArrayList<FixedMeshBody> ();
      ArrayList<PolygonalMesh> screwMeshes = new ArrayList<PolygonalMesh> ();

      for (MeshComponent mesh : mechModel.meshBodies ()) {
         System.out.println(mesh.getName ());
         if (mesh.getName ()!=null && mesh.getName ().contains ("Screw")) {
            screwBodies.add ((FixedMeshBody)mesh);
            screwMeshes.add (((PolygonalMesh)mesh.getMesh ()));
         }
      }
      double[] screwIntersectionVolume = new double[screwMeshes.size ()];
      int[] screwCorrespondingSegment = new int[screwMeshes.size ()];
      PolygonalMesh[] screwMeshesCopy = new PolygonalMesh[screwMeshes.size ()];
      FixedMeshBody[] screwBodiesCopy = new FixedMeshBody[screwMeshes.size ()];

      Vector3d translate = new Vector3d (root.TranslateToCentroidMandible);
      AffineTransform3d t = new AffineTransform3d ();
      t.setTranslation (translate);

      List<PolygonalMesh> screwsExport = new ArrayList<PolygonalMesh> ();
      for (int i = 0; i < screwMeshes.size (); i++) {
         screwsExport.add (i, new PolygonalMesh (screwMeshes.get (i).clone ()));
         screwsExport.get (i).inverseTransform (t);
         try {
            GenericMeshWriter
               .writeMesh (
                  "Screws" + String.valueOf (i) + ".stl", screwsExport.get (i));
            System.out.println ("line 5779");
         }
         catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace ();
         }
      }

      System.out.println ("screw size " + screwMeshes.get (0).computeArea ());

      for (int i = 0; i < screwMeshes.size (); i++) {
         screwIntersectionVolume[i] = 0;
         screwCorrespondingSegment[i] = -1;

         screwMeshesCopy[i] = screwMeshes.get (i).clone ();
         screwBodiesCopy[i] =
            new FixedMeshBody ("screwBody" + i, screwMeshesCopy[i]);

         Vector3d centroidScrew = new Vector3d ();
         screwMeshesCopy[i].computeCentroid (centroidScrew);
         RigidTransform3d transform = new maspack.matrix.RigidTransform3d ();
         transform.setTranslation (centroidScrew);
         screwMeshesCopy[i].inverseTransform (transform);
         screwBodiesCopy[i].setPose (transform);
         screwBodiesCopy[i].scaleMesh (1.2);
      }

      ArrayList<PolygonalMesh> donorSegmentMeshes =
         new ArrayList<PolygonalMesh> ();
      for (MeshComponent mesh : mechModel.meshBodies ()) {
         if (mesh.getName ()!=null && mesh.getName ().contains ("DonorSegment")) {
            donorSegmentMeshes.add (((PolygonalMesh)mesh.getMesh ()));
         }
      }
      for (int i = 0; i < donorSegmentMeshes.size (); i++) {
         PolygonalMesh donorSegment = donorSegmentMeshes.get (i);
         System.out.println ("i is " + i);
         for (int j = 0; j < screwMeshes.size (); j++) {
            System.out.println ("j is " + j);
            PolygonalMesh screw = screwMeshesCopy[j];
            PolygonalMesh screwIntersection =
               MeshFactory.getIntersection (donorSegment, screw);
            double vol = screwIntersection.computeVolume ();
            if (vol > screwIntersectionVolume[j]) {
               if (screwIntersectionVolume[j] != 0) {
                  System.out
                     .println (
                        "!!screw num " + j + " intersects with both segments "
                        + i + " and " + screwCorrespondingSegment[j]);
                  screwIntersectionVolume[j] = 0;
                  screwCorrespondingSegment[j] = -1;
               }
               else {
                  System.out
                     .println (
                        "screw num " + j + " has intersect vol " + vol
                        + " with seg " + i);
                  screwIntersectionVolume[j] = vol;
                  screwCorrespondingSegment[j] = i;
               }

            }

         }
      }

      // // do not transform any screws intersect with unresected mandible
      // for (int j=0; j<screwMeshes.length; j++) {
      // PolygonalMesh screw = screwMeshesCopy[j];
      // PolygonalMesh screwIntersectionUnresected1 =
      // MaxillaReconstructionNewest.findIntersection (nonResectionMesh1,
      // screw);
      // PolygonalMesh screwIntersectionUnresected2 =
      // MaxillaReconstructionNewest.findIntersection (nonResectionMesh2,
      // screw);
      // double vol1 = screwIntersectionUnresected1.computeVolume ();
      // double vol2 = screwIntersectionUnresected2.computeVolume ();
      //
      // if (vol1 != 0 || vol2 != 0) {
      // System.out.println ("!!screw num " + j + " intersects with
      // unresected mandible");
      // screwIntersectionVolume[j] = 0;
      // screwCorrespondingSegment[j] = -1;
      // }
      //
      // }

      // for (int j = 0; j < screwMeshes.size(); j++) {
      // if (screwIntersectionVolume[j] != 0) {
      // System.out
      // .println (
      // "screw mesh num " + j + " has max intersect vol "
      // + screwIntersectionVolume[j] + " with seg "
      // + screwCorrespondingSegment[j]);
      // screwMeshes.get (j)
      // .inverseTransform (
      // transforms.get (screwCorrespondingSegment[j]));
      // screwMeshes.get (j)
      // .transform (translateBack.get (screwCorrespondingSegment[j]));
      // screwAlreadyTransformed[j] = 1;
      // }
      // else {
      // mechModel.removeMeshBody (screwBodies.get(j));
      // }
      // }
      for (RigidTransform3d transform: root.transforms) {
         System.out.println(transform.toString ());
      }
      for (int j = 0; j < screwMeshes.size(); j++) {
         
         if (screwIntersectionVolume[j] != 0) {
            System.out
               .println (
                  "screw mesh num " + j + " has max intersect vol "
                  + screwIntersectionVolume[j] + " with seg "
                  + screwCorrespondingSegment[j]);
            PolygonalMesh temp = screwMeshes.get (j).copy();
            temp.setName ("FibScrew"+j);
            temp.inverseTransform (root.transforms.get (screwCorrespondingSegment[j]));
            temp.transform (root.translateBack.get (screwCorrespondingSegment[j]));
            FixedMeshBody tempbody = new FixedMeshBody("FibScrew"+j, temp);
            mechModel.addMeshBody (tempbody);
//            screwMeshes.get (j)
//               .transform (translateBack.get (screwCorrespondingSegment[j]));
//            screwAlreadyTransformed[j] = 1;
         }
//         else {
//            mechModel.removeMeshBody (screwBodies.get (j));
//         }
      }
   }
}