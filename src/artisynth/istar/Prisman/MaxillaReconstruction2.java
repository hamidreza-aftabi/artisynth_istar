package artisynth.istar.Prisman;

import java.awt.Color;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;

import javax.swing.AbstractAction;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JComponent;
import javax.swing.JFileChooser;
import javax.swing.JLabel;
import javax.swing.JScrollPane;
import javax.swing.JSplitPane;
import javax.swing.JTextArea;

import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.util.ArtisynthPath;
import artisynth.istar.Assist.Assist;
import artisynth.istar.MandibleRecon.ClipMandibleButtonClicked;
import artisynth.istar.MandibleRecon.LineSimplificationButtonClicked;
import artisynth.istar.MaxillaRecon.AddPlaneButtonClicked;
import artisynth.istar.MaxillaRecon.ClipMaxillaButtonClicked;
import artisynth.istar.MaxillaRecon.ClipScapula;
import artisynth.istar.MaxillaRecon.LoadFiles;
import artisynth.istar.MaxillaRecon.MaxillaSimpClicked;
import artisynth.istar.MaxillaRecon.MaxillaTransformButtonClicked;
import artisynth.istar.MaxillaRecon.PlateButtonClicked;
import artisynth.istar.MaxillaRecon.exportFiles;
import artisynth.istar.Prisman.undo.ClipMaxillaCommand;
import artisynth.istar.MandibleRecon.TransformButtonClicked;
import artisynth.istar.MaxillaRecon.createDonorGuideClicked;
import artisynth.istar.MandibleRecon.createFibulaScrewsButtonClicked;
import artisynth.istar.MandibleRecon.createMandibleGuideClicked;
import artisynth.istar.MandibleRecon.plateDeformPrepClicked;
import artisynth.istar.MandibleRecon.registerClicked;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.PolylineMesh;
import maspack.interpolation.NumericList;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.PointStyle;
import maspack.widgets.ExpandablePropertyPanel;

public class MaxillaReconstruction2 extends ReconstructionModel {
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
      // Scapula option checkbox
      donorIsScapulaCheckBox = new JCheckBox ("Donor is Scapula");
      donorIsScapulaCheckBox.setSelected (false);
      list.add (donorIsScapulaCheckBox);
      // load
      String[] loadList = { "Maxilla", "Donor", "Planes", "Markers" };
      loads = new JComboBox<String> (loadList);
      loads.addActionListener (new LoadFiles (this, mechModel, loads));
      JSplitPane loadPane = new JSplitPane ();
      JLabel loadPaneLabel = new JLabel ("Load: ");
      loadPane.setLeftComponent (loadPaneLabel);
      loadPane.setRightComponent (loads);
      // export
      String[] exportList = { "Maxilla", "Planes","Recon","Donor Guide","Screws", "Markers" };
      exports = new JComboBox<String> (exportList);
      exports.addActionListener (new exportFiles (this, mechModel, exports));
      JSplitPane exportPane = new JSplitPane ();
      JLabel exportPaneLabel = new JLabel ("Export: ");
      exportPane.setLeftComponent (exportPaneLabel);
      exportPane.setRightComponent (exports);
      list.add (loadPane);

      list
         .add (
            Assist
               .createButton (
                  "Add Plane", new AddPlaneButtonClicked (this, mechModel)));
      list
         .add (
            Assist
               .createButton (
                  "Clip Maxilla",
                  new ClipMaxillaButtonClicked (this, mechModel)));
      list
         .add (
            Assist
               .createButton (
                  "Clip Scapula", new ClipScapula (this, mechModel)));
      list.get (4).setEnabled (false);
      list
         .add (
            Assist
               .createButton (
                  "Plate", new PlateButtonClicked (this, mechModel)));
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

      JSplitPane rdp = new JSplitPane ();
      rdp
         .setLeftComponent (
            Assist
               .createButton (
                  "RDP Simp", new MaxillaSimpClicked (
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
      // Add to options to prep
      JSplitPane prep = new JSplitPane ();
      JLabel preplabel = new JLabel ("Prep Donor Advanced Settings");
      // prep.setLeftComponent (DonorClipButton);
      prep.setRightComponent (prepEx);
      prep
         .setLeftComponent (
            Assist
               .createButton (
                  "Transform",
                  new MaxillaTransformButtonClicked (
                     this, mechModel, DonorDistanceProx, DonorDistanceDis,
                     multiplier, SAW_BLD_THICK)));
      list.add (prep);

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
                  new plateDeformPrepClicked (this, mechModel, "Maxilla")));
      // Create expandable panel for Mandible Guide's Advanced Setting
      list.add (plateDeform);
      list
         .add (
            Assist
               .createButton (
                  "Create Fibula Screws Button",
                  new createFibulaScrewsButtonClicked (this, mechModel)));
      list.add (exportPane);
      // Scapula option checkbox
      donorIsScapulaCheckBox.addActionListener(new ActionListener() {
         public void actionPerformed(ActionEvent ae) {
            if (donorIsScapulaCheckBox.isSelected ()) {
               list.get (4).setEnabled (true);
               list.get (7).getComponent (2).setEnabled (false);
               list.get (8).setEnabled (false);
               list.get (10).setEnabled (false);
            }
            else {
               list.get (4).setEnabled (false);
               list.get (7).getComponent (2).setEnabled (true);
               list.get (7).setEnabled (true);
               list.get (8).setEnabled(true);
               list.get (10).setEnabled (true);
            }
         }
       }
     );
      for (JComponent component : list) {
         myControlPanel.addWidget (component);
      }
      return (myControlPanel);
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