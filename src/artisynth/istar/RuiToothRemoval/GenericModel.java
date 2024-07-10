package artisynth.istar.RuiToothRemoval;

import java.io.IOException;
import maspack.geometry.io.GenericMeshReader;
import maspack.geometry.io.VtkAsciiReader;
import maspack.geometry.PolygonalMesh;
import maspack.render.RenderProps;
import artisynth.core.femmodels.FemModel.IncompMethod;
import artisynth.core.materials.*;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.workspace.RootModel;

public class GenericModel extends RootModel
{
    protected MechModel mechModel = new MechModel();

    protected String meshDir;
    protected String dataDir;
    protected boolean showVisibilityPanel = true;

    // global physical properties
    protected double frictionCoeff = 0.0;

    protected double rbDensity = 1.0;
    protected double femDensity = 1.0;
    protected FemMaterial femMaterial;
    protected IncompMethod femIncompMethod = IncompMethod.AUTO;
    protected double femParticleDamping = 0.0;
    protected double femStiffnessDamping = 0.0;
    protected RenderProps femRendering;
    protected RenderProps rbRendering;

    public GenericModel (String name) throws IOException 
    {
        addModel(mechModel);
        //      meshDir = ArtisynthPath.getSrcRelativePath ( this, "geometry/");
        //      dataDir = ArtisynthPath.getSrcRelativePath ( this, "data/");
    }

    // --- Mesh Readers --- //
 
    public static PolygonalMesh loadGeometry(String meshDir, String meshName)
    {
        try
        {
            PolygonalMesh mesh;
            String meshNameLower = meshName.toLowerCase();
            if      (meshNameLower.endsWith(".vtk") == true)
                mesh = VtkAsciiReader.read(meshDir + meshName);
            else
                mesh = (PolygonalMesh)GenericMeshReader.readMesh (meshDir + meshName);
                
            //else if (meshNameLower.endsWith(".obj") == true)
            //    mesh = (PolygonalMesh)WavefrontReader.read(meshDir + meshName);
            //else if (meshNameLower.endsWith(".ply") == true)
            //    mesh = (PolygonalMesh)PlyReader.read(meshDir + meshName);
            //else if (meshNameLower.endsWith(".stl") == true)
            //    mesh = (PolygonalMesh)StlReader.read(meshDir + meshName);
            return mesh;
        }
        catch (Exception e)
        {
            e.printStackTrace();
            System.out.println("Failed to read geometry: " + meshName);
            return null;
        }
    }

}

// functionality to add
// * general fem reader, rigidBody reader
// * general auto-attach --> try to make this fast...not node-by-node
// * 
