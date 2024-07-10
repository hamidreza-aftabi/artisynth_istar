package artisynth.istar.MandibleRecon;

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
import artisynth.istar.Prisman.undo.DentalImplantCommand;

/**
 * @author Matthew Mong Abstract action for adding a dental implant to a mech model
 * 
 *
 */
public class createDentalImplantButtonClicked extends AbstractAction {
   /**
    * 
    */
   MechModel mechModel = null;
   ReconstructionModel root = null;
   Assist Assist = new Assist ();
   FixedMeshBody resectionMeshBody, nonResectionMeshBody;
   ArrayList<FixedMeshBody> cuttingplanes = new ArrayList<> ();

   public createDentalImplantButtonClicked (ReconstructionModel root,
   MechModel mechModel) {
      putValue (NAME, "Just Click It");
      this.mechModel = mechModel;
      this.root = root;
   }

   @Override
   public void actionPerformed (ActionEvent evt) {

      DentalImplantCommand cmd = new DentalImplantCommand (root, mechModel);
      Main main = Main.getMain ();
      UndoManager undo = main.getUndoManager ();
      undo.execute (cmd);

   }
}