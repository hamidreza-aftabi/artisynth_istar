package artisynth.istar.MaxillaRecon;

import java.awt.event.ActionEvent;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;

import javax.swing.AbstractAction;
import javax.swing.JComboBox;
import javax.swing.JFileChooser;
import javax.swing.JFrame;

import artisynth.core.driver.Main;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.util.*;
import artisynth.istar.Assist.Assist;
import artisynth.istar.Prisman.HelperMeshFunctions;
import artisynth.istar.Prisman.ReconstructionModel;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.widgets.*;

public class exportFiles extends AbstractAction {
   MechModel model = null;
   JComboBox exports = null;
   ReconstructionModel root = null;
   JFileChooser fileChooser = new JFileChooser ();
   Assist Assist = new Assist ();
   HelperMeshFunctions meshHelper = new HelperMeshFunctions ();

   // Change MandibleReconstruction to Root Model once the appropriate constants
   // r extracted
   public exportFiles (ReconstructionModel root, MechModel model,
   JComboBox exports) {
      putValue (NAME, "Load Stl Files");
      this.model = model;
      this.exports = exports;
      this.root = root;
   }

   @Override
   public void actionPerformed (ActionEvent e) {
      // TODO Auto-generated method stub
      switch ((String)exports.getSelectedItem ()) {
         case ("Maxilla"):
            Assist.ExportWorldMeshes (model, "Maxilla", "Maxilla");
            break;
         case "Donor Guide":
            Assist.ExportWorldMeshes (model, "Donor Guide");
            break;
         case "Screws":
            ArrayList<String> out1 = new ArrayList<String> ();
            for (MeshComponent mesh : model.meshBodies ()) {
               if (mesh.getName () != null
               && mesh.getName ().contains ("Screw")) {
                  out1.add (mesh.getName ());
                  System.out.println(mesh.getName ());
               }
            }
            Assist.ExportWorldMeshes (model, out1);
            break;
         case "Recon":
            Assist.ExportWorldMeshes (model, "ReconstructedMaxilla", "ReconstructedMaxilla");
            break;
         case ("Planes"): {
            ArrayList<FixedMeshBody> planeBodies =
               new ArrayList<FixedMeshBody> ();
            for (MeshComponent meshcomp : model.meshBodies ()) {
               if (meshcomp.getName () != null
               && meshcomp.getName ().contains ("plane")) {
                  planeBodies.add ((FixedMeshBody)meshcomp);
               }
            }
            try {
               Main main = Main.getMain ();
               JFrame frame = main.getMainFrame ();
               JFileChooser chooser = new JFileChooser ();
               chooser.setCurrentDirectory (main.getModelDirectory ());
               chooser
                  .setFileSelectionMode (JFileChooser.FILES_AND_DIRECTORIES);
               chooser.setSelectedFile (new File ("planes.txt"));
               int retVal = chooser.showSaveDialog (frame);
               if (retVal == JFileChooser.APPROVE_OPTION) {
                  PrintWriter printFile =
                     new PrintWriter (chooser.getSelectedFile (), "UTF-8");
                  for (FixedMeshBody planeBody : planeBodies) {
                     Vector3d center = new Point3d ();
                     Vector3d normal = new Vector3d ();
                     RigidTransform3d pose = planeBody.getPose ();
                     pose.R.getColumn (2, normal);
                     center = pose.p;
                     printFile.println (normal);
                     printFile.println (center);
                  }
                  printFile.close ();
               }
            }
            catch (IOException e1) {
               e1.printStackTrace ();
            }
            break;
         }
         case ("Markers"): {
            Main main = Main.getMain ();
            if (model.frameMarkers().size() == 0) {
               GuiUtils.showError (main.getMainFrame(), "No markers defined");
               return;
            }
            JFrame frame = main.getMainFrame ();
            JFileChooser chooser = new JFileChooser ();
            chooser.setCurrentDirectory (main.getModelDirectory ());
            chooser.setSelectedFile (new File ("markers.txt"));
            int retval = fileChooser.showDialog (frame, "Save");
            if (retval == JFileChooser.APPROVE_OPTION) {
               File file = fileChooser.getSelectedFile ();
               exportMaxillaMarkers (fileChooser.getSelectedFile());
            }
            break;
         }
            
      }
   }

   void exportMaxillaMarkers (File file) {
      PrintWriter pw = null;
      try {
         pw = ArtisynthIO.newIndentingPrintWriter(file);
         Point3d pos = new Point3d();
         for (FrameMarker mkr : model.frameMarkers()) {
            pw.println (mkr.getPosition());
         }
      }
      catch (IOException e) {
         System.out.println ("Error writing "+file+": "+e);
         return;
      }
      finally {
         pw.close();
      }
   }
}
