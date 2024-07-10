package artisynth.istar.MandibleRecon;

import java.awt.Component;
import java.awt.Container;
import java.awt.event.ActionListener;
import java.io.IOException;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import javax.swing.AbstractAction;
import javax.swing.AbstractButton;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JSeparator;
import javax.swing.JSplitPane;
import javax.swing.JTextField;

import artisynth.core.femmodels.FemModel.IncompMethod;
import artisynth.core.gui.ControlPanel;
import artisynth.core.materials.FemMaterial;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.Model;
import artisynth.core.workspace.RootModel;
import artisynth.istar.Prisman.ImprovedFormattedTextField;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.io.GenericMeshReader;
import maspack.geometry.io.VtkAsciiReader;
import maspack.render.RenderProps;
import maspack.widgets.ExpandablePropertyPanel;

/**
 * @author Matthew Mong
 * Control Panel Subclass for reconstructive surgery model manipulation
 */
public class ReconstructionControlPanel {

   public ReconstructionControlPanel () {

   }


}