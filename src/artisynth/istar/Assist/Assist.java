package artisynth.istar.Assist;

import java.awt.Component;
import java.awt.Container;
import java.io.BufferedOutputStream;
//import java.io.File;
import java.io.*;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

import javax.swing.AbstractAction;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JSplitPane;

import artisynth.core.driver.Main;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.istar.Prisman.ImprovedFormattedTextField;
import maspack.geometry.MeshBase;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.io.GenericMeshReader;
import maspack.geometry.io.GenericMeshWriter;
import maspack.geometry.io.StlReader;
import maspack.geometry.io.VtkAsciiReader;
import maspack.matrix.AffineTransform3dBase;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;

/**
 * @author Matthew Mong Assist functions built for reconstructive surgery
 */

public class Assist {

   public Assist () {

   }

   public PolygonalMesh loadGeometry (String meshDir, String fileName) {
      try {
         PolygonalMesh mesh;
         String meshNameLower = fileName.toLowerCase ();
         if (meshNameLower.endsWith (".vtk") == true)
            mesh = VtkAsciiReader.read (meshDir + fileName);
         else if (meshNameLower.endsWith (".stl") == true) {
            mesh = StlReader.read (meshDir + fileName);
         }
         else
            mesh =
               (PolygonalMesh)GenericMeshReader.readMesh (meshDir + fileName);
         mesh.setName (fileName);
         return mesh;
      }
      catch (Exception e) {
         e.printStackTrace ();
         System.out.println ("Failed to read geometry: " + fileName);
         return null;
      }
   }

   public PolygonalMesh loadGeometry (String meshLocation) {
      try {
         PolygonalMesh mesh;
         if (meshLocation.endsWith (".vtk") == true)
            mesh = VtkAsciiReader.read (meshLocation);
         else
            mesh = (PolygonalMesh)GenericMeshReader.readMesh (meshLocation);
         return mesh;
      }
      catch (Exception e) {
         e.printStackTrace ();
         System.out.println ("Failed to read geometry at: " + meshLocation);
         return null;
      }
   }

   public void attachVisiblityToPanel (RigidBody body, ControlPanel panel) {
      panel.addWidget (body.getName (), body, "renderProps.visible");
   }

   public void attachVisiblityToPanel (FixedMeshBody body, ControlPanel panel) {
      panel.addWidget (body.getName (), body, "renderProps.visible");
   }

   /**
    * Helper function to create Buttons for Swing UI
    * 
    * @param title
    * the name of the button
    * @param Action
    * Action to be performed when clicking, should be a subclass of
    * AbstractAction
    * @return JButton
    */
   public JButton createButton (String title, AbstractAction Action) {
      JButton Button = new JButton (title);
      Button.addActionListener (Action);
      return Button;
   }

   /**
    * Helper function to create Buttons for Swing UI
    * 
    * @param title
    * the name of the CheckBox
    * @param def
    * default value
    * @param Action
    * Action to be performed when clicking, should be a subclass of
    * AbstractAction
    * @return JCheckBox
    */
   public JCheckBox createCheckBox (
      String title, boolean def, AbstractAction Action) {
      JCheckBox check = new JCheckBox (title);
      check.setSelected (def);
      if (Action != null) {
         check.addActionListener (Action);
      }
      return check;

   }

   public RigidBody FileToRigidBody (File file, String name, Boolean center) {
      Assist Assist = new Assist ();
      RigidBody MeshBody = new RigidBody (name);
      PolygonalMesh Mesh = Assist.loadGeometry (file.getAbsolutePath ());
      Mesh.removeDisconnectedVertices ();
      Mesh.setName (file.getName ().replaceFirst ("[.][^.]+$", ""));
      MeshBody.setSurfaceMesh (Mesh);
      if (center == true) {
         Point3d centroid = new Point3d ();
         Mesh.computeCentroid (centroid);
         Vector3d CentroidVector = new Vector3d ();
         Mesh.computeCentroid (CentroidVector);
         // This is from the legacy code but it's only ever used once?
         // TranslateToCentroidMandible = new Vector3d (centroid).negate ();
         Mesh.translateToCentroid ();
      }
      return (MeshBody);
   }

   public FixedMeshBody FileToFixedMeshBody (
      File file, String name, Boolean center) {
      Assist Assist = new Assist ();
      FixedMeshBody MeshBody = new FixedMeshBody (name);
      PolygonalMesh Mesh = Assist.loadGeometry (file.getAbsolutePath ());
      Mesh.removeDisconnectedVertices ();
      Mesh.setName (file.getName ().replaceFirst ("[.][^.]+$", ""));
      MeshBody.setMesh (Mesh);
      if (center == true) {
         Point3d centroid = new Point3d ();
         Mesh.computeCentroid (centroid);
         Vector3d CentroidVector = new Vector3d ();
         Mesh.computeCentroid (CentroidVector);
         // This is from the legacy code but it's only ever used once?
         // TranslateToCentroidMandible = new Vector3d (centroid).negate ();
         Mesh.translateToCentroid ();
      }
      return (MeshBody);
   }

   public Vector3d CenterTransform (File file) {
      PolygonalMesh mandibleMesh = null;
      try {
         mandibleMesh = StlReader.read (file);
         mandibleMesh.removeDisconnectedVertices ();
         RigidBody mandibleBody2 = new RigidBody ("Mandible");
         mandibleBody2.setSurfaceMesh (mandibleMesh);
         Point3d centroid = new Point3d ();
         mandibleMesh.computeCentroid (centroid);
         Vector3d mandibleCentroidOriginal = new Vector3d ();
         mandibleMesh.computeCentroid (mandibleCentroidOriginal);
         mandibleMesh.translateToCentroid ();
         return (new Vector3d (centroid).negate ());
      }
      catch (Exception e) {
         System.out.println (e);
      }
      return null;
   }

   public RigidBody GetRigidBody (MechModel model, String name) {
      // System.out.println(model.rigidBodies ().get (name).getName ());
      return model.rigidBodies ().get (name);
   }

   public FixedMeshBody GetMeshBody (MechModel model, String name) {
      return (FixedMeshBody)model.meshBodies ().get (name);
   }

   public PolygonalMesh GetMesh (MechModel model, String name) {
      if (model.rigidBodies ().get (name) != null) {
         if (model.rigidBodies ().get (name).getSurfaceMesh () != null) {
            return model.rigidBodies ().get (name).getSurfaceMesh ();
         }
      }
      else if (model.meshBodies ().get (name) != null) {
         if (model.meshBodies ().get (name).getMesh () != null) {
            return (PolygonalMesh)model.meshBodies ().get (name).getMesh ();
         }
      }
      System.out.println (name + " is not present in the model");
      return null;
   }

   public MeshBase GetNonPolyMesh (MechModel model, String name) {
      if (model.rigidBodies ().get (name) != null) {
         if (model.rigidBodies ().get (name).getSurfaceMesh () != null) {
            return model.rigidBodies ().get (name).getSurfaceMesh ();
         }
      }
      else if (model.meshBodies ().get (name) != null) {
         if (model.meshBodies ().get (name).getMesh () != null) {
            return model.meshBodies ().get (name).getMesh ();
         }
      }
      System.out.println (name + " is not present in the model");
      return null;
   }

   public ArrayList<PolygonalMesh> getAllMeshes (MechModel model, String name) {
      ArrayList<PolygonalMesh> returnlist = new ArrayList<PolygonalMesh> ();
      if (model.rigidBodies ().get (name) != null) {
         try {
            for (PolygonalMesh mesh : model
               .rigidBodies ().get (name).getSurfaceMeshes ()) {
               returnlist.add (mesh);
            }
         }
         catch (Exception e) {
            System.out.println ("No Surface Mesh");
         }
      }
      else if (model.meshBodies ().get (name) != null) {
         try {
            returnlist
               .add ((PolygonalMesh)model.meshBodies ().get (name).getMesh ());
         }
         catch (Exception e) {
            System.out.println ("No Surface Mesh");
         }
      }
      if (returnlist.size () == 0) {
         System.out.println ("No Meshes Detected");
      }
      return (returnlist);
   }

   // TODO: Make set origin not call same mesh
   public Vector3d SetOriginTransform (String OriginMesh) {
      Assist Assist = new Assist ();
      PolygonalMesh mandibleMesh = Assist.loadGeometry (OriginMesh);
      int remove = mandibleMesh.removeDisconnectedVertices ();
      RigidBody mandibleBody2 = new RigidBody ("Mandible");
      mandibleBody2.setSurfaceMesh (mandibleMesh);
      Point3d centroid = new Point3d ();
      mandibleMesh.computeCentroid (centroid);
      Vector3d mandibleCentroidOriginal = new Vector3d ();
      mandibleMesh.computeCentroid (mandibleCentroidOriginal);
      return (new Vector3d (centroid).negate ());
   }

   public JSplitPane createIntSplitPanel (
      String label, ImprovedFormattedTextField textfield) {
      JSplitPane pane = new JSplitPane (JSplitPane.HORIZONTAL_SPLIT);
      JLabel Jlabel = new JLabel (label);
      pane.setLeftComponent (Jlabel);
      pane.setRightComponent (textfield);
      return pane;
   }

   public ArrayList<String> getSplitText (
      ControlPanel Controlpanel, Integer ComponentNumber) {
      ImprovedFormattedTextField text = null;
      ArrayList<String> list = new ArrayList<String> ();
      Component comp =
         Controlpanel.getPropertyPanel ().getWidget (ComponentNumber);
      for (Component siftedComp : getAllComponents ((Container)comp)) {
         System.out.println (siftedComp.getClass ());
      }
      return list;
   }

   public void saveMesh (MeshBase mesh, AffineTransform3dBase X, File file) {
      try {
         if (mesh instanceof PolygonalMesh) {
            GenericMeshWriter writer = new GenericMeshWriter (file);
            writer.writeMesh (mesh);
            writer.close ();
         }
         else {
            PrintWriter pw =
               new PrintWriter (
                  new BufferedOutputStream (new FileOutputStream (file)));
            mesh.write (pw, "%g");
            pw.close ();
         }
      }
      catch (Exception ex) {
         ex.printStackTrace ();
         // GuiUtils.showError (frame, "Error saving file: "+ex.getMessage());
      }
   }

   public void ExportWorldMeshes (MechModel model, ArrayList<String> input) {
      Assist Assist = new Assist ();
      Main main = Main.getMain ();
      JFrame frame = main.getMainFrame ();
      JFileChooser chooser = new JFileChooser ();
      chooser.setCurrentDirectory (main.getModelDirectory ());
      chooser.setFileSelectionMode (JFileChooser.DIRECTORIES_ONLY);
      ArrayList<PolygonalMesh> MeshList = new ArrayList<PolygonalMesh> ();
      for (String Body : input) {
         MeshList.addAll (Assist.getAllMeshes (model, Body));
      }
      int retVal = chooser.showSaveDialog (frame);
      if (retVal == JFileChooser.APPROVE_OPTION) {
         for (PolygonalMesh mesh : MeshList) {
            System.out.println(mesh.getName ());
            File file = null;
            if (mesh.getName () != null) {
               file =
                  new File (
                     chooser.getSelectedFile (), mesh.getName () + ".stl");
            }
            else {
               file =
                  new File (
                     chooser.getSelectedFile (), mesh.toString () + ".stl");
            }
            System.out.println (file.getAbsolutePath ());
            Assist.saveMesh (mesh, mesh.getMeshToWorld (), file);
         }
         // main.setModelDirectory (chooser.getCurrentDirectory ());
      }
   }

   /**
    * @param model should be the MechModel from ReconstructionModel
    * @param input string name of body that should be exported
    */
   public void ExportWorldMeshes (MechModel model, String input) {
      Assist Assist = new Assist ();
      Main main = Main.getMain ();
      JFrame frame = main.getMainFrame ();
      JFileChooser chooser = new JFileChooser ();
      chooser.setCurrentDirectory (main.getModelDirectory ());
      chooser.setFileSelectionMode (JFileChooser.DIRECTORIES_ONLY);
      ArrayList<PolygonalMesh> MeshList = new ArrayList<PolygonalMesh> ();
      MeshList.addAll (Assist.getAllMeshes (model, input));
      int retVal = chooser.showSaveDialog (frame);
      if (retVal == JFileChooser.APPROVE_OPTION) {
         for (PolygonalMesh mesh : MeshList) {
            File file = null;
            if (mesh.getName () != null) {
               file =
                  new File (
                     chooser.getSelectedFile (), mesh.getName () + ".stl");
            }
            else {
               file =
                  new File (
                     chooser.getSelectedFile (), mesh.toString () + ".stl");
            }
            System.out.println (file.getAbsolutePath ());
            Assist.saveMesh (mesh, mesh.getMeshToWorld (), file);
         }
         main.setModelDirectory (chooser.getCurrentDirectory ());
      }
   }

   /** Saves the mesh relative to the world as an STL file 
    * @param model is the mechModel from ReconstructionModel
    * @param input is the body that should be saved
    * @param fileName is the name of the file to be saved
    */
   public void ExportWorldMeshes (
      MechModel model, String input, String fileName) {
      Assist Assist = new Assist ();
      Main main = Main.getMain ();
      JFrame frame = main.getMainFrame ();
      JFileChooser chooser = new JFileChooser ();
      chooser.setCurrentDirectory (main.getModelDirectory ());
      chooser.setFileSelectionMode (JFileChooser.DIRECTORIES_ONLY);
      ArrayList<PolygonalMesh> MeshList = new ArrayList<PolygonalMesh> ();
      MeshList.addAll (Assist.getAllMeshes (model, input));
      int retVal = chooser.showSaveDialog (frame);
      if (retVal == JFileChooser.APPROVE_OPTION) {
         for (PolygonalMesh mesh : MeshList) {
            File file =
               new File (chooser.getSelectedFile (), fileName + ".stl");
            System.out.println (file.getAbsolutePath ());
            Assist.saveMesh (mesh, mesh.getMeshToWorld (), file);
         }
         main.setModelDirectory (chooser.getCurrentDirectory ());
      }
   }

   public static List<Component> getAllComponents (final Container c) {
      Component[] comps = c.getComponents ();
      List<Component> compList = new ArrayList<Component> ();
      for (Component comp : comps) {
         compList.add (comp);
         if (comp instanceof Container)
            try {
               compList.addAll (getAllComponents ((Container)comp));
            }
            catch (final ClassCastException e) {
               continue;
            }
      }
      return compList;
   }

   public void setPlaneOrigin (FixedMeshBody plane, Vector3d origin) {
      Vector3d centroid = new Vector3d ();
      plane.getMesh ().computeCentroid (centroid);
      RigidTransform3d transMat = new RigidTransform3d ();
      transMat.setTranslation (new Vector3d (origin).sub (centroid));
      plane.transformGeometry (transMat);
   }
}