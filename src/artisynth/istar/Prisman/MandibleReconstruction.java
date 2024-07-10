package artisynth.istar.Prisman;

import java.awt.Color;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.text.NumberFormat;
import java.util.ArrayList;

import javax.swing.AbstractAction;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JComponent;
import javax.swing.JFileChooser;
import javax.swing.JLabel;
import javax.swing.JSlider;
import javax.swing.JSplitPane;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.util.ArtisynthPath;
import artisynth.istar.Assist.Assist;
import artisynth.istar.MandibleRecon.ClipMandibleButtonClicked;
import artisynth.istar.MandibleRecon.LineSimplificationButtonClicked;
import artisynth.istar.MandibleRecon.LoadFiles;
import artisynth.istar.MandibleRecon.ManualCutButtonClicked;
import artisynth.istar.MandibleRecon.ManualFibGuide;
import artisynth.istar.MandibleRecon.PlateButtonClicked;
import artisynth.istar.MandibleRecon.TransformButtonClicked;
import artisynth.istar.MandibleRecon.createDentalImplantButtonClicked;
import artisynth.istar.MandibleRecon.createDonorGuideClicked;
import artisynth.istar.MandibleRecon.createFibulaScrewsButtonClicked;
import artisynth.istar.MandibleRecon.createMandibleGuideClicked;
import artisynth.istar.MandibleRecon.plateDeformPrepClicked;
import artisynth.istar.MandibleRecon.registerClicked;
import artisynth.istar.Prisman.HelperMathFunctions;
import artisynth.istar.Prisman.HelperMeshFunctions;
import artisynth.istar.Prisman.ImprovedFormattedTextField;
import artisynth.istar.Prisman.ReconstructionModel;
import artisynth.istar.Prisman.MandibleReconstruction.CheckHolderOption;
import artisynth.istar.Prisman.MandibleReconstruction.CheckScapulaOption;
import artisynth.istar.Prisman.MandibleReconstruction.exportClicked;
import artisynth.istar.Prisman.undo.AddPlaneClicked;
import artisynth.istar.Prisman.undo.BonyContactCalc;
import artisynth.istar.Prisman.undo.FinalCutButtonClicked;
import artisynth.istar.Prisman.undo.FinalizeManualClicked;
import artisynth.istar.Prisman.undo.GetMetricsButtonClicked;
import artisynth.istar.Prisman.undo.TransformTesting2Clicked;
import artisynth.istar.Prisman.undo.TransformTestingClicked;
import artisynth.istar.Prisman.ManualSegmentClicked;
import maspack.geometry.PolygonalMesh;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.PointStyle;
import maspack.widgets.ExpandablePropertyPanel;

public class MandibleReconstruction extends ReconstructionModel {
   public Assist Assist = new Assist ();

   ControlPanel visibilityPanel = new ControlPanel ("Visibility", "LiveUpdate");

   final double SAW_BLD_THICK = 1; // mm Set to 1

   int flag = 0;

   public Boolean slicerPlanes;
   public Boolean blenderPlanes;

   JButton hidePlatingFiducials, hideDonorFiducials, register,
   mandibleguidesettings;
   JCheckBox screwsCheckBox;
   JCheckBox platingDir;
   JCheckBox connectorOption;
   JCheckBox dentalImplantsCheckBox;
   JCheckBox oneHolderOption;
   JCheckBox automaticScapulaTransform;
   JComboBox exports, loads;
   JFileChooser fileChooser;
   public ImprovedFormattedTextField DonorDistanceProx;
   public ImprovedFormattedTextField DonorDistanceDis;
   ImprovedFormattedTextField trimScapulaDistance;
   public ImprovedFormattedTextField multiplier;

   ControlPanel myControlPanel;
   MechModel mechModel;

   HelperMathFunctions mathHelper = new HelperMathFunctions ();
   HelperMeshFunctions meshHelper = new HelperMeshFunctions ();

   @Override
   public void build (String[] args) {
      mechModel = new MechModel ("Reconstruction");
      mechModel.setGravity (0, 0, 0);
      RenderProps.setPointStyle (mechModel, PointStyle.SPHERE);
      RenderProps.setPointRadius (mechModel, 1.25);
      RenderProps.setFaceStyle (mechModel, FaceStyle.FRONT_AND_BACK);
      RenderProps.setBackColor (mechModel, Color.GREEN);
      addControlPanel (visibilityPanel);
      addModel (mechModel);

      ControlPanel controlPanel = makeControlPanel ();

      addControlPanel (controlPanel);
      fm.clear ();
      frameMarkerPos.clear ();
      String homedir = ArtisynthPath.getHomeDir ();
      File pathHome = new File (homedir);
      String homeParent = pathHome.getParentFile ().getAbsolutePath ();
      if (super.DEBUG) {
         System.out.println ("Parent Directory: " + homeParent);
      }
   }

   // Created main control panel
   private ControlPanel makeControlPanel () {
      ArrayList<JComponent> list = new ArrayList<JComponent> ();
      myControlPanel = new ControlPanel ("Control Panel", "");
      myControlPanel.addWidget (scrollPane);
      log.append ("Mandible Fibula reconstruction selected\n");
      log.append ("Please check Donor is Scapula if necessary\n");
      log.append ("Otherwise load files now\n");

      // Buttons
      String[] loadList =
         { "<None>", "Load All 4 (Artisynth)", "Load All 4 (Slicer)",
           "Mandible", "Donor", "Fiducial (Artisynth)", "Fiducial (Slicer)",
           "Resection Plane Info (Artisynth)", "Resection Plane Info (Slicer)",
           "Resection Plane Info (Blender)", "Dental Implants", "Maxilla", "Postop Mandible" };
      loads = new JComboBox (loadList);
      loads.setSelectedIndex (0);
      loads.addActionListener (new LoadFiles (this, mechModel, loads));
      JSplitPane loadPane = new JSplitPane ();
      JLabel loadPaneLabel = new JLabel ("Load: ");
      loadPane.setLeftComponent (loadPaneLabel);
      loadPane.setRightComponent (loads);
      list.add (loadPane);

      donorIsScapulaCheckBox = new JCheckBox ("Donor is Scapula");
      donorIsScapulaCheckBox.setSelected (false);
      list.add (donorIsScapulaCheckBox);

      platingDir = new JCheckBox ("Plating Direction: Left to Right");
      platingDir.setSelected (true);
      list.add (platingDir);
      
      list
      .add (
         Assist
            .createButton (
               "Get Native/Postop Metrics", new GetMetricsButtonClicked (this, mechModel)));
      
      list
      .add (
         Assist
            .createButton (
               "Add Plane", new AddPlaneClicked (this, mechModel)));

      list
      .add (
         Assist
            .createButton (
               "Get Bony Contact", new BonyContactCalc (this, mechModel)));
      
      list
         .add (
            Assist
               .createButton (
                  "Plate", new PlateButtonClicked (this, mechModel)));
            
      list
         .add (
            Assist
               .createButton (
                  "Clip Mandible", new ClipMandibleButtonClicked (this, mechModel)));

      ExpandablePropertyPanel RDPex = new ExpandablePropertyPanel ();
      JSplitPane minDistancePane = new JSplitPane ();
      JLabel minDistanceLabel = new JLabel ("Minimum Segment Length");
      minDistancePane.setLeftComponent (minDistanceLabel);
      ImprovedFormattedTextField rdpMinDistance =
         new ImprovedFormattedTextField (NumberFormat.getNumberInstance (), 20);
      minDistancePane.setRightComponent (rdpMinDistance);

      JSplitPane maxSegmentsPane = new JSplitPane ();
      JLabel maxSegmentsLabel = new JLabel ("Max Segments");
      maxSegmentsPane.setLeftComponent (maxSegmentsLabel);
      ImprovedFormattedTextField rdpMaxSegments =
         new ImprovedFormattedTextField (NumberFormat.getIntegerInstance (), 5);
      maxSegmentsPane.setRightComponent (rdpMaxSegments);

      RDPex.addExtraWidget (minDistancePane);
      RDPex.addExtraWidget (maxSegmentsPane);
      RDPex
         .addExtraWidget (
            Assist
               .createButton (
                  "Add Dental Implant",
                  new createDentalImplantButtonClicked (this, mechModel)));

      JSplitPane rdp = new JSplitPane ();
      rdp
         .setLeftComponent (
            Assist
               .createButton (
                  "RDP Simp", new LineSimplificationButtonClicked (
                     this, mechModel, rdpMinDistance, rdpMaxSegments)));
      rdp.setRightComponent (RDPex);
      list.add (rdp);

      DonorDistanceDis =
         new ImprovedFormattedTextField (NumberFormat.getNumberInstance (), 80);
      DonorDistanceProx =
         new ImprovedFormattedTextField (NumberFormat.getNumberInstance (), 80);
      multiplier =
         new ImprovedFormattedTextField (
            NumberFormat.getNumberInstance (), 3.5);
      ExpandablePropertyPanel prepEx = new ExpandablePropertyPanel ();
      // Splitting Plane for Donor distance from Prox and distal ends
      JSplitPane DonorProxPane = new JSplitPane ();
      JLabel DonorProxLabel = new JLabel ("Prox Distance");
      DonorProxPane.setLeftComponent (DonorProxLabel);
      DonorProxPane.setRightComponent (DonorDistanceProx);

      JSplitPane DonorDisPane = new JSplitPane ();
      JLabel DonorDisLabel = new JLabel ("Distal Distance");
      DonorDisPane.setLeftComponent (DonorDisLabel);
      DonorDisPane.setRightComponent (DonorDistanceDis);

      JSplitPane DonorMultiplier = new JSplitPane ();
      JLabel DonorMultiplierLabel = new JLabel ("Multiplier");
      DonorMultiplier.setLeftComponent (DonorMultiplierLabel);
      DonorMultiplier.setRightComponent (multiplier);

      prepEx.addExtraWidget (DonorProxPane);
      prepEx.addExtraWidget (DonorDisPane);
      prepEx.addExtraWidget (DonorMultiplier);
      prepEx
      .addExtraWidget (
         Assist
            .createButton (
               "Add Manual Segment", new ManualSegmentClicked (this, mechModel)));
      prepEx
         .addExtraWidget (
            Assist
               .createButton (
                  "Manual Cut", new ManualCutButtonClicked (this, mechModel)));
      prepEx
         .addExtraWidget (
            Assist
               .createButton (
                  "Add Final Segment", new FinalCutButtonClicked (this, mechModel)));
      prepEx
         .addExtraWidget (
            Assist
               .createButton (
                  "Finalize Manual Recon", new FinalizeManualClicked (this, mechModel)));      

      prepEx
         .addExtraWidget (
            Assist
               .createButton (
                  "Manual Fib Guide", new ManualFibGuide (this, mechModel)));
      
      // Add to options to prep
      JSplitPane prep = new JSplitPane ();
      JLabel preplabel = new JLabel ("Prep Donor Advanced Settings");
      // prep.setLeftComponent (DonorClipButton);
      
      prep.setRightComponent (prepEx);    
      prep
         .setLeftComponent (
            Assist
               .createButton (
                  "Transform", new TransformButtonClicked(this, mechModel, 
                     DonorDistanceProx, DonorDistanceDis, multiplier, SAW_BLD_THICK)));
      list.add (prep);
      
//      list
//      .add (
//         Assist
//            .createButton (
//               "Transform Testing",
//               new TransformTestingClicked (this, mechModel, 
//                  DonorDistanceProx, DonorDistanceDis, multiplier, SAW_BLD_THICK)));
      
      list
      .add (
         Assist
            .createButton (
               "Transform Testing 2",
               new TransformTesting2Clicked (this, mechModel, 
                  DonorDistanceProx, DonorDistanceDis, multiplier, SAW_BLD_THICK)));
      
      list
         .add (
            Assist
               .createButton (
                  "Create Donor Guide",
                  new createDonorGuideClicked (this, mechModel)));

      JSplitPane plateDeform = new JSplitPane (JSplitPane.HORIZONTAL_SPLIT);
      plateDeform
         .setLeftComponent (
            Assist
               .createButton (
                  "Prep Plate Deformation",
                  new plateDeformPrepClicked (this, mechModel, "Mandible")));
      
      // add in slider to control plate stiffness
      JSlider stiffness = new JSlider (0, 20, 1);
      stiffness.setValue (5);

      // paint the ticks and tracks
      stiffness.setPaintTrack (true);
      stiffness.setPaintTicks (true);
      stiffness.setPaintLabels (true);
      class SliderListener implements ChangeListener {

         @Override
         public void stateChanged (ChangeEvent e) {
            // TODO Auto-generated method stub
            FemModel3d femmodel = (FemModel3d)mechModel.models ().get ("fem");
            JSlider source = (JSlider)e.getSource ();
            if (!source.getValueIsAdjusting ()) {
               int stiffness = (int)source.getValue ();
               // femmodel.setStiffnessDamping (stiffness);
               RenderableComponentList<AxialSpring> springs =
                  mechModel.axialSprings ();
               for (AxialSpring i : springs) {
                  i.setStiffness (i, 100 * 2 * stiffness);
               }
            }
         }
      }
      stiffness.addChangeListener (new SliderListener ());
      plateDeform.setRightComponent (stiffness);
      // Create expandable panel for Mandible Guide's Advanced Setting
      list.add (plateDeform);

      list
         .add (
            Assist
               .createButton (
                  "Create Fibula Screws Button",
                  new createFibulaScrewsButtonClicked (this, mechModel)));
      list
         .add (
            Assist
               .createButton (
                  "Registration",
                  new registerClicked (platingDir, this, mechModel)));

      ExpandablePropertyPanel cp = new ExpandablePropertyPanel ();

      screwsCheckBox =
         Assist
            .createCheckBox (
               "Screws at both sides of Cutting slots", false, null);
      connectorOption =
         Assist.createCheckBox ("Don't need connector", true, null);
      oneHolderOption =
         Assist
            .createCheckBox (
               "One Holder (Edit only Plane0 and Plane3 distance)", false,
               new CheckHolderOption ());
      automaticScapulaTransform =
         Assist
            .createCheckBox (
               "Automatic Scapula Transform", false, new CheckScapulaOption ());

      // Setting Input Values for guide planes
      ImprovedFormattedTextField p0value =
         new ImprovedFormattedTextField (NumberFormat.getNumberInstance (), 17);
      JSplitPane p0pane =
         Assist.createIntSplitPanel ("Plane 0 distance", p0value);

      ImprovedFormattedTextField p1value =
         new ImprovedFormattedTextField (NumberFormat.getNumberInstance (), 10);
      JSplitPane p1pane =
         Assist.createIntSplitPanel ("Plane 1 distance", p1value);

      ImprovedFormattedTextField p2value =
         new ImprovedFormattedTextField (NumberFormat.getNumberInstance (), 10);
      JSplitPane p2pane =
         Assist.createIntSplitPanel ("Plane 2 distance", p2value);

      ImprovedFormattedTextField p3value =
         new ImprovedFormattedTextField (NumberFormat.getNumberInstance (), 17);
      JSplitPane p3pane =
         Assist.createIntSplitPanel ("Plane 3 distance", p3value);

      cp.addExtraWidget (p0pane);
      cp.addExtraWidget (p1pane);
      cp.addExtraWidget (p2pane);
      cp.addExtraWidget (p3pane);
      cp.addExtraWidget (oneHolderOption);
      cp.addExtraWidget (connectorOption);
      JSplitPane mand = new JSplitPane ();
      JLabel mandlabel = new JLabel ("Mandible Guide Advanced Settings");
      mand.setRightComponent (cp);
      mand
         .setLeftComponent (
            Assist
               .createButton (
                  "Create Mandible Guide Primitive",
                  new createMandibleGuideClicked (
                     p0value, p1value, p2value, p3value, platingDir,
                     oneHolderOption, screwsCheckBox, connectorOption, this,
                     mechModel)));
      list.add (mand);

      String[] exportList =
         { "Reconstructed Mandible", "Donor Guide", "Mandible Guide",
           "Unresected", "Donor Segments", "Non-Resected Donor", "Screws",
           "Fiducials", "Plane Values", };

      exports = new JComboBox<String> (exportList);
      exports.setSelectedIndex (0);
      exports.addActionListener (new exportClicked ());
      JSplitPane exportPane = new JSplitPane ();
      JLabel exportPaneLabel = new JLabel ("Export: ");
      exportPane.setLeftComponent (exportPaneLabel);
      exportPane.setRightComponent (exports);
      list.add (exportPane);

      donorIsScapulaCheckBox.addActionListener (new ActionListener () {
         public void actionPerformed (ActionEvent ae) {
            if (donorIsScapulaCheckBox.isSelected ()) {
               list.get (9).setEnabled (false);
               list.get (11).setEnabled (false);
            }
            else {
               list.get (9).setEnabled (true);
               list.get (11).setEnabled (true);
            }
         }
      });
      for (JComponent component : list) {
         myControlPanel.addWidget (component);
      }
      return (myControlPanel);
   }
   // Export Function that exports files chosen by user through export drop down
   // menu in the main control panel
   public class exportClicked extends AbstractAction {
      ExportFunctions exportFunctions = new ExportFunctions (mechModel);

      public exportClicked () {
         putValue (NAME, "Export Files");
      }

      @Override
      public void actionPerformed (ActionEvent evt) {
         // Export reconstructed mandible with Donor Guide holes
         switch ((String)exports.getSelectedItem ()) {
            case "Reconstructed Mandible":
               Assist.ExportWorldMeshes (mechModel, "ReconstructedMandible");
               Assist.ExportWorldMeshes (mechModel, "BackupMandible");
               break;
            case "Donor Guide":
               Assist.ExportWorldMeshes (mechModel, "Donor Guide");
               break;
            case "Mandible Guide":
               ArrayList<String> outputs = new ArrayList<String> ();
               outputs.add ("InBox0");
               outputs.add ("InBox1");
               outputs.add ("Outbox0");
               outputs.add ("Outbox1");
               outputs.add ("Base1");
               outputs.add ("Base2");
               Assist.ExportWorldMeshes (mechModel, outputs);
               break;
            case "Unresected":
               Assist.ExportWorldMeshes (mechModel, "Unresected");
               break;
            case "Donor Segments":
               ArrayList<String> out = new ArrayList<String> ();
               for (MeshComponent mesh : mechModel.meshBodies ()) {
                  if (mesh.getName () != null
                  && mesh.getName ().contains ("DonorSegment")) {
                     out.add (mesh.getName ());
                  }
               }
               Assist.ExportWorldMeshes (mechModel, out);
               break;
            case "Screws":
               ArrayList<String> out1 = new ArrayList<String> ();
               for (MeshComponent mesh : mechModel.meshBodies ()) {
                  if (mesh.getName () != null
                  && mesh.getName ().contains ("Screw")) {
                     out1.add (mesh.getName ());
                  }
               }
               Assist.ExportWorldMeshes (mechModel, out1);
               break;
            case "Fiducials":
               exportFunctions
                  .ExportFramemarkers (
                     Assist.GetRigidBody (mechModel, "Mandible"));
               break;
            case "Plane Values":
               System.out
                  .println (mechModel.meshBodies ().get ("Plane1").toString ());
               exportFunctions
                  .ExportPlaneInfo (
                     Assist.GetMeshBody (mechModel, "Plane1"),
                     Assist.GetMeshBody (mechModel, "Plane2"),
                     Assist.GetMesh (mechModel, "Plane1"),
                     Assist.GetMesh (mechModel, "Plane2"));
               break;
            default:
               System.out.println ("Error");
               break;
         }
      }
   }

   // Creating Mandible Guide

   public class CheckHolderOption extends AbstractAction {
      public CheckHolderOption () {
         putValue (NAME, "Check Holder Option");
      }

      @Override
      public void actionPerformed (ActionEvent evt) {
         if (oneHolderOption.isSelected ()) {
            connectorOption.setSelected (true);
            myControlPanel.updateWidgetValues ();
         }
      }
   }

   public class CheckScapulaOption extends AbstractAction {
      public CheckScapulaOption () {
         putValue (NAME, "Check Scapula Option");
      }

      @Override
      public void actionPerformed (ActionEvent evt) {
         if (automaticScapulaTransform.isSelected ()) {
            donorIsScapulaCheckBox.setSelected (true);
            myControlPanel.updateWidgetValues ();
         }
      }
   }
}
