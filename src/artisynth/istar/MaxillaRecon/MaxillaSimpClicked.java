package artisynth.istar.MaxillaRecon;

import java.awt.event.ActionEvent;

import javax.swing.AbstractAction;

import artisynth.core.driver.Main;
import artisynth.core.gui.editorManager.UndoManager;
import artisynth.core.mechmodels.MechModel;
import artisynth.istar.Assist.Assist;
import artisynth.istar.Prisman.HelperMathFunctions;
import artisynth.istar.Prisman.ImprovedFormattedTextField;
import artisynth.istar.Prisman.ReconstructionModel;
import artisynth.istar.Prisman.undo.MaxillaSimpCommand;
import artisynth.istar.Prisman.undo.SimpCommand;

// Simplifying plate to a series of lines that the donor segment can be
// aligned to
public class MaxillaSimpClicked extends AbstractAction {
   /**
    * 
    */
   ImprovedFormattedTextField rdpMinDistance = null;
   ImprovedFormattedTextField rdpMaxSegments = null;
   MechModel mechModel = null;
   ReconstructionModel root = null;
   Assist Assist = new Assist ();
   HelperMathFunctions mathHelper = new HelperMathFunctions ();

   public MaxillaSimpClicked (ReconstructionModel root,
   MechModel mechModel, ImprovedFormattedTextField rdpMinDistance,
   ImprovedFormattedTextField rdpMaxSegments) {
      putValue (NAME, "Line Simplification");
      this.rdpMinDistance = rdpMinDistance;
      this.rdpMaxSegments = rdpMaxSegments;
      this.mechModel = mechModel;
      this.root = root;
   }

   @Override
   public void actionPerformed (ActionEvent evt) {
      // Undo Command
      MaxillaSimpCommand cmd =
         new MaxillaSimpCommand (root, mechModel, rdpMinDistance, rdpMaxSegments);
      Main main = Main.getMain ();
      UndoManager undo = main.getUndoManager ();
      undo.execute (cmd);

   }
}