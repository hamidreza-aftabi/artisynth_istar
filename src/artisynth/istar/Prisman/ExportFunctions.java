/**
 * Copyright (c) 2014, by the Authors: John E Lloyd (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package artisynth.istar.Prisman;

import java.awt.event.ActionEvent;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import artisynth.istar.Assist.Assist;

import javax.swing.AbstractAction;
import javax.swing.JFileChooser;
import javax.swing.JFrame;

import artisynth.core.driver.Main;
import artisynth.core.gui.editorManager.UndoManager;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.istar.Prisman.undo.PrepCommand;
import maspack.geometry.MeshFactory;
import maspack.geometry.MeshICP;
import maspack.collision.SurfaceMeshIntersector;
import maspack.collision.SurfaceMeshIntersectorTest;
import maspack.collision.SurfaceMeshIntersector.CSG;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.PolylineMesh;
import maspack.geometry.Vertex3d;
import maspack.geometry.MeshICP.AlignmentType;
import maspack.geometry.OBB;
import maspack.geometry.io.GenericMeshWriter;
import maspack.geometry.io.StlReader;
import maspack.geometry.io.WavefrontWriter;
import maspack.interpolation.NumericList;
import maspack.interpolation.NumericListKnot;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.AxisAngle;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.SVDecomposition;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.spatialmotion.SpatialInertia;
import maspack.util.InternalErrorException;

public class ExportFunctions {
   Assist Assist = new Assist ();
   MechModel mechModel = null;

   public ExportFunctions (MechModel mechModel) {
      this.mechModel = mechModel;
   }

   // Export reconstructed mandible with Donor Guide holes
   public void ExportReconstructedMandible (
      Vector3d TranslateToCentroidMandible, boolean donorIsScapulaCheckBox) {
      try {
         Vector3d translate = new Vector3d (TranslateToCentroidMandible);
         AffineTransform3d t = new AffineTransform3d ();
         t.setTranslation (translate);
         PolygonalMesh reconCopy = new PolygonalMesh ();
         PolygonalMesh translated = new PolygonalMesh ();
         if ((Assist.GetMesh (mechModel, "Donor Guide") != null)) {
            reconCopy =
               new PolygonalMesh (Assist.GetMesh (mechModel, "Recon").clone ());
         }
         else {
            System.out.println ("No Recon");
         }
         reconCopy.inverseTransform (t);
         GenericMeshWriter.writeMesh ("ReconstructedMandible.stl", reconCopy);
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }

   // Export donor guide
   public void ExportDonorGuide (Vector3d TranslateToCentroidMandible) {
      try {
         PolygonalMesh translatedGuide =
            new PolygonalMesh (
               Assist.GetMesh (mechModel, "Donor Guide").clone ());
         Vector3d translate = new Vector3d (TranslateToCentroidMandible);
         AffineTransform3d t = new AffineTransform3d ();
         t.setTranslation (translate);
         translatedGuide.inverseTransform (t);

         GenericMeshWriter.writeMesh ("DonorGuide.stl", translatedGuide);
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }

   // Export mandible guide
   public void ExportMandibleGuide (
      boolean connectorOption, boolean oneHolderOption,
      PolygonalMesh MandibleGuide, Vector3d TranslateToCentroidMandible,
      PolygonalMesh guide1, PolygonalMesh guide2) {
      try {
         if (!connectorOption || oneHolderOption) {
            PolygonalMesh translatedGuide =
               new PolygonalMesh (MandibleGuide.clone ());
            Vector3d translate = new Vector3d (TranslateToCentroidMandible);
            AffineTransform3d t = new AffineTransform3d ();
            t.setTranslation (translate);
            translatedGuide.inverseTransform (t);

            GenericMeshWriter.writeMesh ("MandibleGuide.stl", translatedGuide);
         }
         else {
            PolygonalMesh translatedGuide = new PolygonalMesh (guide1.clone ());
            Vector3d translate = new Vector3d (TranslateToCentroidMandible);
            AffineTransform3d t = new AffineTransform3d ();
            t.setTranslation (translate);
            translatedGuide.inverseTransform (t);
            GenericMeshWriter.writeMesh ("MandibleGuide1.stl", translatedGuide);
            translatedGuide = new PolygonalMesh (guide2.clone ());
            translate = new Vector3d (TranslateToCentroidMandible);
            t = new AffineTransform3d ();
            t.setTranslation (translate);
            translatedGuide.inverseTransform (t);
            GenericMeshWriter.writeMesh ("MandibleGuide2.stl", translatedGuide);
         }
      }
      catch (IOException e) {
         e.printStackTrace ();
      }

   }

   // Export reconstructed mandible with screw holes from Mandible Guide
   // and Donor Guide
   public void ExportReconScrewholes (
      boolean donorIsScapulaCheckBox, PolygonalMesh DonorGuide,
      PolygonalMesh reconstructedWithDonorGuideHoles,
      PolygonalMesh mandiblereconstructed, ArrayList<PolygonalMesh> screws,
      MechModel mechModel, Vector3d TranslateToCentroidMandible) {
      try {
         PolygonalMesh translated = new PolygonalMesh ();
         if (!donorIsScapulaCheckBox && (DonorGuide != null)) {
            translated =
               new PolygonalMesh (reconstructedWithDonorGuideHoles.clone ());
         }
         else {
            translated = new PolygonalMesh (mandiblereconstructed.clone ());
         }
         for (int i = 0; i < screws.size (); i++) {
            translated =
               MeshFactory.getSubtraction (translated, screws.get (i));
         }
         FixedMeshBody translatedBody =
            new FixedMeshBody ("ReconstructionWHoles", translated);
         mechModel.addMeshBody (translatedBody);
         Vector3d translate = new Vector3d (TranslateToCentroidMandible);
         AffineTransform3d t = new AffineTransform3d ();
         t.setTranslation (translate);
         PolygonalMesh trans = new PolygonalMesh (translated.clone ());
         trans.inverseTransform (t);
         GenericMeshWriter.writeMesh ("ReconWScrewHoles.stl", trans);
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }

   // Export donor segments
   public void ExportDonorSegments (
      Vector3d TranslateToCentroidMandible,
      List<PolygonalMesh> donorSegmentMeshes) {
      try {
         Vector3d translate = new Vector3d (TranslateToCentroidMandible);
         AffineTransform3d t = new AffineTransform3d ();
         t.setTranslation (translate);
         List<PolygonalMesh> translated = new ArrayList<PolygonalMesh> ();
         for (int i = 0; i < donorSegmentMeshes.size (); i++) {
            translated.add (i, new PolygonalMesh (donorSegmentMeshes.get (i)));
            translated.get (i).inverseTransform (t);
            GenericMeshWriter
               .writeMesh (
                  "DonorSegments" + String.valueOf (i) + ".stl",
                  translated.get (i));
         }
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }

   // Export non-resected mandible
   public void ExportNonResectedMandible (
      PolygonalMesh nonResectionMesh1, PolygonalMesh nonResectionMesh2,
      Vector3d TranslateToCentroidMandible) {
      try {
         PolygonalMesh translated =
            new PolygonalMesh (nonResectionMesh1.clone ());
         translated =
            MeshFactory.getUnion (translated, nonResectionMesh2.clone ());
         Vector3d translate = new Vector3d (TranslateToCentroidMandible);
         AffineTransform3d t = new AffineTransform3d ();
         t.setTranslation (translate);
         translated.inverseTransform (t);

         GenericMeshWriter.writeMesh ("NonResectionMandible.stl", translated);
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }

   // Export mandible guide parts
   public void ExportMandibleGuideParts (
      Vector3d TranslateToCentroidMandible, PolygonalMesh[] OuterBox,
      PolygonalMesh[] InnerBox, boolean connectorOption,
      PolygonalMesh connector, boolean oneHolderOption, PolygonalMesh base1,
      PolygonalMesh base2, ArrayList<PolygonalMesh> screws) {

      try {
         Vector3d translate = new Vector3d (TranslateToCentroidMandible);
         AffineTransform3d t = new AffineTransform3d ();
         t.setTranslation (translate);

         // Exporting Cutting Slots
         PolygonalMesh[] out = new PolygonalMesh[2];
         PolygonalMesh[] in = new PolygonalMesh[2];

         out[0] = new PolygonalMesh (OuterBox[0].clone ());
         in[0] = new PolygonalMesh (InnerBox[0].clone ());
         out[1] = new PolygonalMesh (OuterBox[1].clone ());
         in[1] = new PolygonalMesh (InnerBox[1].clone ());

         out[0].inverseTransform (t);
         in[0].inverseTransform (t);
         out[1].inverseTransform (t);
         in[1].inverseTransform (t);

         GenericMeshWriter.writeMesh ("Out0.stl", out[0]);
         GenericMeshWriter.writeMesh ("In0.stl", in[0]);
         GenericMeshWriter.writeMesh ("Out1.stl", out[1]);
         GenericMeshWriter.writeMesh ("In1.stl", in[1]);

         // Exporting Connector
         if (!connectorOption) {
            PolygonalMesh c = new PolygonalMesh (connector.clone ());
            c.inverseTransform (t);
            GenericMeshWriter.writeMesh ("connector.stl", c);
         }

         // Exporting Bases
         if (!oneHolderOption) {
            PolygonalMesh b1 = new PolygonalMesh (base1.clone ());
            b1.inverseTransform (t);
            GenericMeshWriter.writeMesh ("base1.stl", b1);

            PolygonalMesh b2 = new PolygonalMesh (base2.clone ());
            b2.inverseTransform (t);
            GenericMeshWriter.writeMesh ("base2.stl", b2);
         }
         else {
            PolygonalMesh b1 = new PolygonalMesh (base1.clone ());
            b1.inverseTransform (t);
            GenericMeshWriter.writeMesh ("base1.stl", b1);
         }

         // Exporting Screws
         List<PolygonalMesh> screwsExport = new ArrayList<PolygonalMesh> ();
         for (int i = 0; i < screws.size (); i++) {
            screwsExport.add (i, new PolygonalMesh (screws.get (i).clone ()));
            screwsExport.get (i).inverseTransform (t);
            GenericMeshWriter
               .writeMesh (
                  "Screws" + String.valueOf (i) + ".stl", screwsExport.get (i));
         }
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }

   public void ExportDonorGuideParts (
      Vector3d TranslateToCentroidMandible, List<PolygonalMesh> large,
      List<PolygonalMesh> small, boolean donorIsScapulaCheckBox,
      List<PolygonalMesh> ScrewHoles, PolygonalMesh[] screwsabove,
      PolygonalMesh[] screwsbelow, PolygonalMesh L) {

      try {
         Vector3d translate = new Vector3d (TranslateToCentroidMandible);
         AffineTransform3d t = new AffineTransform3d ();
         t.setTranslation (translate);

         // Exporting Cylinder and Base
         // if (!donorIsScapulaCheckBox.isSelected ()) {
         // PolygonalMesh Cy = new PolygonalMesh (cylinder.clone ());
         // Cy.inverseTransform (t);
         // GenericMeshWriter.writeMesh ("Cylinder.stl", Cy);
         // }

         // PolygonalMesh B = new PolygonalMesh (Base.clone ());
         // B.inverseTransform (t);
         // GenericMeshWriter.writeMesh ("Base.stl", B);

         // Exporting cutting slots
         List<PolygonalMesh> largeExport = new ArrayList<PolygonalMesh> ();
         for (int i = 0; i < large.size (); i++) {
            largeExport.add (i, new PolygonalMesh (large.get (i).clone ()));
            largeExport.get (i).inverseTransform (t);
            GenericMeshWriter
               .writeMesh (
                  "Large" + String.valueOf (i) + ".stl", largeExport.get (i));
         }

         List<PolygonalMesh> smallExport = new ArrayList<PolygonalMesh> ();
         for (int i = 0; i < small.size (); i++) {
            smallExport.add (i, new PolygonalMesh (small.get (i).clone ()));
            smallExport.get (i).inverseTransform (t);
            GenericMeshWriter
               .writeMesh (
                  "Small" + String.valueOf (i) + ".stl", smallExport.get (i));
         }

         if (!donorIsScapulaCheckBox) {
            // Exporting Screws
            List<PolygonalMesh> screwsExport = new ArrayList<PolygonalMesh> ();
            for (int i = 0; i < ScrewHoles.size (); i++) {
               screwsExport
                  .add (i, new PolygonalMesh (ScrewHoles.get (i).clone ()));
               screwsExport.get (i).inverseTransform (t);
               GenericMeshWriter
                  .writeMesh (
                     "ScrewHoles" + String.valueOf (i) + ".stl",
                     screwsExport.get (i));
            }
            PolygonalMesh[] sa = new PolygonalMesh[2];
            PolygonalMesh[] sb = new PolygonalMesh[2];

            sa[0] = new PolygonalMesh (screwsabove[0].clone ());
            sb[0] = new PolygonalMesh (screwsbelow[0].clone ());

            sa[0].inverseTransform (t);
            sb[0].inverseTransform (t);

            GenericMeshWriter.writeMesh ("SA0.stl", sa[0]);
            GenericMeshWriter.writeMesh ("SB0.stl", sb[0]);

            if (screwsabove[1] != null) {
               sa[1] = new PolygonalMesh (screwsabove[1].clone ());
               sa[1].inverseTransform (t);
               GenericMeshWriter.writeMesh ("SA1.stl", sa[1]);
            }
            if (screwsbelow[1] != null) {
               sb[1] = new PolygonalMesh (screwsbelow[1].clone ());
               sb[1].inverseTransform (t);
               GenericMeshWriter.writeMesh ("SB1.stl", sb[1]);
            }

            // exporting Marking
            PolygonalMesh LM = new PolygonalMesh (L.clone ());
            LM.inverseTransform (t);
            GenericMeshWriter.writeMesh ("Marking.stl", LM);
         }
      }
      catch (IOException e) {
         e.printStackTrace ();
      }

   }

   public void ExportClippedDonor (
      PolygonalMesh clippedDonorMesh, Vector3d TranslateToCentroidMandible) {
      try {
         PolygonalMesh translatedDonor =
            new PolygonalMesh (clippedDonorMesh.clone ());
         Vector3d translate = new Vector3d (TranslateToCentroidMandible);
         AffineTransform3d t = new AffineTransform3d ();
         t.setTranslation (translate);
         translatedDonor.inverseTransform (t);

         GenericMeshWriter.writeMesh ("ClippedDonor.stl", translatedDonor);
      }
      catch (IOException e) {
         e.printStackTrace ();
      }

   }

   public void ExportFramemarkers (RigidBody mandibleMeshBody) {
      try {
         PrintWriter out = new PrintWriter ("plateInfo.txt", "UTF-8");
         FrameMarker[] frameMarkers = mandibleMeshBody.getFrameMarkers ();
         int size = frameMarkers.length;
         RigidTransform3d poseM = mandibleMeshBody.getPose ();
         for (int i = 0; i < size; i++) {
            Point3d position = frameMarkers[i].getPosition ();
            position.transform (poseM);
            out.println (position);
         }
         out.close ();
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
      Main main = Main.getMain ();
      JFrame frame = main.getMainFrame ();
      JFileChooser chooser = new JFileChooser ();
      chooser.setCurrentDirectory (main.getModelDirectory ());
      chooser.setFileSelectionMode (JFileChooser.FILES_AND_DIRECTORIES);
      chooser.setSelectedFile (new File ("fiducials.txt"));
      int retVal = chooser.showSaveDialog (frame);
      if (retVal == JFileChooser.APPROVE_OPTION) {
         try {
            PrintWriter out = new PrintWriter (chooser.getSelectedFile (), "UTF-8");
            FrameMarker[] frameMarkers = mandibleMeshBody.getFrameMarkers ();
            int size = frameMarkers.length;
            RigidTransform3d poseM = mandibleMeshBody.getPose ();
            for (int i = 0; i < size; i++) {
               Point3d position = frameMarkers[i].getPosition ();
               position.transform (poseM);
               out.println (position);
            }
            out.close ();
         }
         catch (IOException e) {
            e.printStackTrace ();
         }
      }
   }

   public void ExportPlaneInfo (
      FixedMeshBody plane1MeshBody, FixedMeshBody plane2MeshBody,
      PolygonalMesh plane1Mesh, PolygonalMesh plane2Mesh) {
      Point3d center1 = new Point3d ();
      Point3d center2 = new Point3d ();
      Vector3d normal1 = new Vector3d ();
      Vector3d normal2 = new Vector3d ();
      RigidTransform3d pose1 = plane1MeshBody.getPose ();
      RigidTransform3d pose2 = plane2MeshBody.getPose ();

      plane1Mesh.computeCentroid (center1);
      plane2Mesh.computeCentroid (center2);
      normal1 = new Vector3d (plane1Mesh.getNormal (0));
      normal2 = new Vector3d (plane2Mesh.getNormal (0));

      Point3d c1 = new Point3d (center1);
      Point3d c2 = new Point3d (center2);
      c1.transform (pose1);
      c2.transform (pose2);
      normal1.transform (pose1);
      normal2.transform (pose2);

      try {
         PrintWriter out = new PrintWriter ("planeInfo.txt", "UTF-8");

         out.println (normal1);
         out.println (c1);
         out.println (normal2);
         out.println (c2);
         out.close ();
      }
      catch (IOException e) {
         e.printStackTrace ();
      }

   }

   public void ExportPlaneValuesAxisAngle (
      FixedMeshBody plane1MeshBody, FixedMeshBody plane2MeshBody,
      PolygonalMesh plane1Mesh, PolygonalMesh plane2Mesh,
      Vector3d TranslateToCentroidMandible) {

      HelperMeshFunctions meshHelper = new HelperMeshFunctions ();

      Point3d center1 = new Point3d ();
      Point3d center2 = new Point3d ();
      AxisAngle normal1 = new AxisAngle ();
      AxisAngle normal2 = new AxisAngle ();
      RigidTransform3d pose1 = plane1MeshBody.getPose ();
      RigidTransform3d pose2 = plane2MeshBody.getPose ();

      plane1Mesh.computeCentroid (center1);
      plane2Mesh.computeCentroid (center2);
      Point3d c1 = new Point3d (center1);
      Point3d c2 = new Point3d (center2);
      c1.transform (pose1);
      c2.transform (pose2);
      c1.sub (TranslateToCentroidMandible);
      c2.sub (TranslateToCentroidMandible);

      Vector3d n1 = new Vector3d (plane1Mesh.getNormal (0));
      Vector3d n2 = new Vector3d (plane2Mesh.getNormal (0));
      n1.transform (pose1);
      n2.transform (pose2);

      RotationMatrix3d R =
         meshHelper.rotatePlane (new Vector3d (0, 0, 1), new Vector3d (n1));
      normal1 = new AxisAngle (R.getAxisAngle ());
      R = meshHelper.rotatePlane (new Vector3d (0, 0, 1), new Vector3d (n2));
      normal2 = new AxisAngle (R.getAxisAngle ());

      try {
         PrintWriter out = new PrintWriter ("planeInfo.txt", "UTF-8");

         out.println (normal1);
         out.println (n1);
         out.println (c1);
         out.println (normal2);
         out.println (n2);
         out.println (c2);
         out.close ();
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }

   public void ExportFibulaJig (PolygonalMesh fibulaJigMesh) {
      try {
         GenericMeshWriter.writeMesh ("FibulaJig.stl", fibulaJigMesh);
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }

   public void ExportRDPCurve (PolylineMesh rdpMesh) {
      try {
         WavefrontWriter ww = new WavefrontWriter ("rdpline.obj");
         ww.writeMesh (rdpMesh);
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }

   public void ExportResectedMandible (
      PolygonalMesh resectionMesh, Vector3d TranslateToCentroidMandible) {
      try {
         PolygonalMesh translated = new PolygonalMesh (resectionMesh);
         Vector3d translate = new Vector3d (TranslateToCentroidMandible);
         AffineTransform3d t = new AffineTransform3d ();
         t.setTranslation (translate);
         translated.inverseTransform (t);

         GenericMeshWriter.writeMesh ("ResectedMandible.stl", translated);
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }

   public void ExportNonResectedDonor (
      PolygonalMesh nonResectionFibulaMesh,
      Vector3d TranslateToCentroidMandible) {
      try {
         PolygonalMesh translated = new PolygonalMesh (nonResectionFibulaMesh);
         Vector3d translate = new Vector3d (TranslateToCentroidMandible);
         AffineTransform3d t = new AffineTransform3d ();
         t.setTranslation (translate);
         translated.inverseTransform (t);

         GenericMeshWriter.writeMesh ("NonResectedFibulaMesh.stl", translated);
      }
      catch (IOException e) {
         e.printStackTrace ();
      }
   }
}
