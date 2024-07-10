package artisynth.istar.MaxillaRecon;

import java.awt.event.ActionEvent;
import java.util.ArrayList;

import javax.swing.AbstractAction;

import artisynth.core.driver.Main;
import artisynth.core.gui.editorManager.UndoManager;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.istar.Assist.Assist;
import artisynth.istar.Prisman.ReconstructionModel;
import artisynth.istar.Prisman.undo.ClipMandCommand;
import artisynth.istar.Prisman.undo.ClipMaxillaCommand;

/**
 * @author Matthew Mong Abstract action for clipping a mandible based on plane
 * meshes present in a mech model
 *
 */
public class ClipMaxillaButtonClicked extends AbstractAction {
   /**
    * 
    */
   private static final long serialVersionUID = -7427620342808956735L;
   /**
    * 
    */
   MechModel mechModel = null;
   ReconstructionModel root = null;
   Assist Assist = new Assist ();
   FixedMeshBody resectionMeshBody, nonResectionMeshBody;
   ArrayList<FixedMeshBody> cuttingplanes = new ArrayList<> ();

   public ClipMaxillaButtonClicked (ReconstructionModel root,
   MechModel mechModel) {
      putValue (NAME, "Just Click It");
      this.mechModel = mechModel;
      this.root = root;
   }

   @Override
   public void actionPerformed (ActionEvent evt) {

      ClipMaxillaCommand cmd = new ClipMaxillaCommand (root, mechModel);
      Main main = Main.getMain ();
      UndoManager undo = main.getUndoManager ();
      undo.execute (cmd);

   }
}