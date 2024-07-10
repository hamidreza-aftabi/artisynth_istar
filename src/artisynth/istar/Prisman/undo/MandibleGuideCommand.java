package artisynth.istar.Prisman.undo;

import java.util.List;
import artisynth.core.mechmodels.*;
import artisynth.core.gui.editorManager.Command;

public class MandibleGuideCommand implements Command {
   private String myName;
   private FixedMeshBody myGuide;
   private MechModel myModel;
   private FixedMeshBody mG1;
   private FixedMeshBody mG2;
   private FixedMeshBody master;
   private boolean c;
   private FixedMeshBody[] OB;
   private FixedMeshBody[] IB;
   private FixedMeshBody CB;
   private List<FixedMeshBody> SB;
   private FixedMeshBody B1;
   private FixedMeshBody B2;
   private boolean h;

   public MandibleGuideCommand (String name, FixedMeshBody mandguide,
   FixedMeshBody guide1, FixedMeshBody guide2, FixedMeshBody masterguide,
   MechModel model, boolean connector, FixedMeshBody[] OutBody,
   FixedMeshBody[] InBody, FixedMeshBody connectorBody,
   List<FixedMeshBody> screwBody, FixedMeshBody base1, FixedMeshBody base2,
   boolean oneHolder) {
      myName = name;
      myGuide = mandguide;
      myModel = model;
      mG1 = guide1;
      mG2 = guide2;
      master = masterguide;
      c = connector;
      OB = OutBody;
      IB = InBody;
      CB = connectorBody;
      SB = screwBody;
      B1 = base1;
      B2 = base2;
      h = oneHolder;
   }

   public void execute () {
      if (!c || h) {
         myModel.addMeshBody (myGuide);
      }
      else {
         myModel.addMeshBody (mG1);
         myModel.addMeshBody (mG2);        
      }
      master.getRenderProps ().setVisible (false);
      for (int i = 0; i < 2; i++) {
         myModel.removeMeshBody (OB[i]);
      }
      for (int i = 0; i < 2; i++) {
         myModel.removeMeshBody (IB[i]);
      }
      if (!c) {
         myModel.removeMeshBody (CB);
      }
      for (int i = 0; i < SB.size (); i++) {
         myModel.removeMeshBody (SB.get (i));
      }
      myModel.removeMeshBody (B1);
      if (!h) {
         myModel.removeMeshBody (B2);
      }
   }

   public void undo () {
      if (!c || h) {
         myModel.removeMeshBody (myGuide);
      }
      else {
            myModel.removeMeshBody (mG1);
            myModel.removeMeshBody (mG2);
      }
      master.getRenderProps ().setVisible (false);
      for (int i = 0; i < 2; i++) {
         myModel.addMeshBody (OB[i]);
      }
      for (int i = 0; i < 2; i++) {
         myModel.addMeshBody (IB[i]);
      }
      if (!c) {
         myModel.addMeshBody (CB);
      }
      for (int i = 0; i < SB.size (); i++) {
         myModel.addMeshBody (SB.get (i));
      }
      myModel.addMeshBody (B1);
      if (!h) {
         myModel.addMeshBody (B2);
      }

   }

   public String getName () {
      return myName;
   }
}
