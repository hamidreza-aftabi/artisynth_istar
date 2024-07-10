 package artisynth.istar.Atabak;

import artisynth.core.mechmodels.*;
import artisynth.core.gui.editorManager.Command;

public class PrepCommand implements Command {
   private String myName;
   private FixedMeshBody myFib;
   private MechModel myModel;
   private RigidBody clippedFibulaMeshBody;
   private boolean scapulaisdonor;
   private FixedMeshBody trimplane;

   public PrepCommand (String name, RigidBody clippedFibulaMesh,
   FixedMeshBody donorMeshBody, boolean DonorOption, FixedMeshBody scapulaTrimPlaneBody, MechModel model) {
      myName = name;
      myModel = model;
      clippedFibulaMeshBody = clippedFibulaMesh;
      myFib = donorMeshBody;
      scapulaisdonor = DonorOption;
      trimplane = scapulaTrimPlaneBody;
   }

   public void execute () {
      myModel.addRigidBody (clippedFibulaMeshBody);
      myFib.getRenderProps ().setVisible (false);
      if (scapulaisdonor) {
         trimplane.getRenderProps().setVisible (false);
      }
   }

   public void undo () {
      myModel.removeRigidBody (clippedFibulaMeshBody);
      myFib.getRenderProps ().setVisible (true);
      if (scapulaisdonor) {
         trimplane.getRenderProps().setVisible (true);
      }
   }

   public String getName () {
      return myName;
   }

}
