package artisynth.istar.NDI;

public interface Connection {

   public static enum StreamingProtocol {
      TCP,
      UDP
   };

   void dispose();

   boolean isConnected();

   boolean connect (String connectionInfo);

   void disconnect();

   //int read (char[] buffer, int length);

   int read (byte[] buffer, int length);

   //int write (char[] buffer, int length);

   int write (byte[] buffer, int length);

   String connectionName();

   StreamingProtocol getProtocol();

}
