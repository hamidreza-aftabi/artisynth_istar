 package artisynth.istar.Atabak;

import java.awt.event.ActionEvent;
import java.util.ArrayList;

import javax.swing.AbstractAction;

import artisynth.core.driver.Main;
import artisynth.core.gui.editorManager.UndoManager;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;

/**
 * @author Matthew Mong Abstract action for clipping a mandible based on plane
 * meshes present in a mech model
 *
 */
public class ClipMandibleButtonClicked extends AbstractAction {
   /**
    * 
    */
   private static final long serialVersionUID = 1496914859360862846L;
   MechModel mechModel = null;
   ReconstructionModel root = null;
   Assist Assist = new Assist ();
   FixedMeshBody resectionMeshBody, nonResectionMeshBody;
   ArrayList<FixedMeshBody> cuttingplanes = new ArrayList<> ();

   public ClipMandibleButtonClicked (ReconstructionModel root,
   MechModel mechModel) {
      putValue (NAME, "Just Click It");
      this.mechModel = mechModel;
      this.root = root;
   }

   @Override
   public void actionPerformed (ActionEvent evt) {

      ClipMandCommand cmd = new ClipMandCommand (root, mechModel);
      Main main = Main.getMain ();
      UndoManager undo = main.getUndoManager ();
      undo.execute (cmd);

   }
}