package artisynth.istar.lloyd;

import maspack.geometry.*;
import maspack.util.*;

import artisynth.core.workspace.*;
import artisynth.core.mechmodels.*;
import artisynth.models.dynjaw.JawDemo;

/**
 * Simple test model to make sure packages are all properly linked, etc.
 */
public class TestModel extends RootModel {

   public static boolean omitFromMenu = true;

   public void build (String[] args) {
      MechModel mech = new MechModel ("mech");
      addModel (mech);

      String jawFile = PathFinder.getSourceRelativePath (
         JawDemo.class, "geometry/jaw.obj");
      
      PolygonalMesh jawMesh = null;
      try {
         jawMesh = new PolygonalMesh (jawFile);
      }
      catch (Exception e) {
         System.out.println ("Can't open or load "+jawFile);
      }
      RigidBody jaw = RigidBody.createFromMesh (
         "jaw", jawMesh, /*densoty=*/1000.0, /*scale=*/1.0);
      mech.addRigidBody (jaw);
   }

}
