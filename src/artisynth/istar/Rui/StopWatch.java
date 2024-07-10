package artisynth.istar.Rui;

public class StopWatch
{
    String name;
    double t_init = 0.0;
    double t0 = 0.0;
    double t1 = 0.0;
    
    public enum Units {seconds, milliseconds};
    Units units = Units.seconds;
    
    boolean active = true; // if I want to silence the output...
    
    public StopWatch(String name)
    {
        initialize(name, Units.seconds, true);
    }
    
    public StopWatch(String name, Units units)
    {
        initialize(name, units, true);
    }
    
    public StopWatch(String name, Units units, boolean active)
    {
        initialize(name, units, active);
    }
    
    void initialize(String name, Units units, boolean active)
    {
        this.name = name;
        this.active = active;
        
        this.units = units;
        
        t_init = time();
        t0 = t_init;
        
        if (active == true)
            System.out.println("Starting stopwatch: " + name);
    }
    
    public void checkpoint(String message)
    {
        if (active == true)
        {
            t1 = time();
            System.out.printf("%s:   Split time: %010.6f,  Total time: %010.6f,  event: %s\n", name, t1-t0, t1-t_init, message);
            t0=t1;
        }
    }
    
    double time()
    {
        if (units == Units.seconds)
            return ((double)System.nanoTime())/1000000000.0; // seconds
        else if (units == Units.milliseconds)
            return ((double)System.nanoTime())/1000000.0;    // microseconds
        else
            return Double.NaN;
    }
}