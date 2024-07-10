package artisynth.istar.NDI;

import java.util.*;
import java.io.*;
import java.net.*;

import maspack.util.*;

/**
 * A cross platform socket implementation.
 */
public class TcpConnection implements Connection {

   //! The socket file descriptor
   Socket socket;
   InputStream sin;
   OutputStream sout;

   //! Setup method common to all constructors.
   void init() {
      socket = null;
      sin = null;
      sout = null;
   }

   /**
    * Constructs an empty socket object.
    */
   public TcpConnection() {
      init();
   }

   /**
    * Constructs a socket object and connects immediately, using port 8765.
    * @param hostname The hostname or IP address of the measurement system.
    */
   public TcpConnection (String hostname) {
      this (hostname, 8765);
   }

   /**
    * Constructs a socket object and connects immediately.
    * @param hostname The hostname or IP address of the measurement system.
    * @param port The port to connect on. Port 8765 is default for Vega systems.
    */
   public TcpConnection (String hostname, int port) {
      init();
      connect (hostname, port);
   }

   /**
    * Closes the socket connection and frees memory.
    */
   public void dispose() {
      disconnect();
   }

   /**
    * Closes any existing connection, and connects to the new device.
    * @param hostname The hostname or IP address of the device.
    * @param port The port number to connect on.
    */
   public boolean connect (String hostname, int port) {
      if (socket != null) {
         disconnect();
      }
      try {
         socket = new Socket (hostname, port);
         sout = socket.getOutputStream();
         sin = socket.getInputStream();
      }
      catch (Exception e) {
         System.err.println ("Error creating socket: "+e);
         disconnect();
      }
      return socket != null;
   }

   /**
    * Closes any existing connection, and connects to the new device on port 8765.
    * @param hostname The hostname or IP address of the device.
    */
   public boolean connect (String hostname) {
      connect (hostname, 8765);
      return socket != null;
   }

   /** Closes the socket connection */
   public void disconnect() {
      if (socket != null) {
         try {
            socket.close();
         }
         catch (IOException e) {
         }
         socket = null;
      }
      if (sout != null) {
         try {
            sout.close();
         }
         catch (IOException e) {
         }
         sout = null;
      }
      if (sin != null) {
         try {
            sin.close();
         }
         catch (IOException e) {
         }
         sin = null;
      }
   }

   /** Returns true if the socket connection succeeded */
   public boolean isConnected() {
      return socket != null;
   }

   /**
    * Reads 'length' bytes from the socket into 'buffer'
    * @param buffer The buffer to read into.
    * @param length The number of bytes to read.
    */
   public int read(byte[] buffer, int length) {
      if (sin == null) {
         return -1;
      }
      try {
         return sin.read (buffer, 0, length);
      }
      catch (IOException e) {
         disconnect();
         return -1;
      }
   }

   /**
    * Writes 'length' bytes from 'buffer' to the socket
    * @param buffer The buffer to write from.
    * @param length The number of bytes to write.
    */
   public int write (byte[] buffer, int length) {
      if (sout == null) {
         return -1;
      }
      try {
         sout.write (buffer, 0, length);
         return length;
      }
      catch (IOException e) {
         disconnect();         
         return -1;
      }
   }

   /**
    * Listens for incoming TCP stream connections
    * @param port The port to connect on. Port 8765 is default for Vega systems.
    */
   public void waitForStream (int port) {
      try (ServerSocket listenSocket = new ServerSocket(port)) {
         socket = listenSocket.accept();
         sout = socket.getOutputStream();
         sin = socket.getInputStream();
      }
      catch (IOException e) {
         System.out.println ("Error: waitForStream failed, "+e);
         disconnect();
      }
   }

   /**
    * gets the IP address as a character buffer
    */
   public String connectionName() {
      if (socket != null) {
         return socket.getInetAddress().toString();
      }
      else {
         return "NONE";
      }
   }

   public StreamingProtocol getProtocol() {
      return StreamingProtocol.TCP;
   }

};
