 package artisynth.istar.Atabak;

import java.awt.event.ActionEvent;

import javax.swing.AbstractAction;

import artisynth.core.driver.Main;
import artisynth.core.gui.editorManager.UndoManager;
import artisynth.core.mechmodels.MechModel;


public class PlateButtonClicked extends AbstractAction {
   ReconstructionModel root;
   MechModel mechModel;

   public PlateButtonClicked (ReconstructionModel root, MechModel mechModel) {
      putValue (NAME, "Plate Mandible");
      this.root = root;
      this.mechModel = mechModel;
   }

   @Override
   public void actionPerformed (ActionEvent e) {
      PlateCommand cmd = new PlateCommand (root, mechModel, "Mandible");
      Main main = Main.getMain ();
      UndoManager undo = main.getUndoManager ();
      undo.execute (cmd);

   }
}