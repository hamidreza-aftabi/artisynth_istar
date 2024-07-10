package artisynth.istar.NDI;

import java.util.*;
import java.io.*;

import maspack.util.*;

import artisynth.istar.NDI.ToolData.ButtonState;
import artisynth.istar.NDI.CombinedApi.TrackingReplyOption;
import artisynth.istar.NDI.CombinedApi.PortHandleSearchRequestOption;

public class CapiSample {

   CombinedApi capi;
   boolean apiSupportsBX2 = true;
   boolean apiSupportsStreaming = true;

   public CapiSample() {
      capi = new CombinedApi();
   }

   public void sleepSeconds(double sec) {
      try {
         Thread.sleep ((int)(1000*sec));
      }
      catch (InterruptedException e) {
      }
   }

   boolean detectKeyStroke() {
      try {
         return System.in.available() > 0;
      }
      catch (Exception e) {
         return false;
      }
   }

   /**
    * Prints a debug message if a method call failed.
    * To use, pass the method name and the error code returned by the method.
    *          Eg: onErrorPrintDebugMessage("capi.initialize()", capi.initialize());
    *          If the call succeeds, this method does nothing.
    *          If the call fails, this method prints an error message to stdout.
    */
   void onErrorPrintDebugMessage (String methodName, int errorCode)  {
      if (errorCode < 0) {
         System.out.println (
            methodName+" failed: " + capi.errorToString(errorCode));
      }
   }

   /**
    * Returns the string: "[tool.id] s/n:[tool.serialNumber]" used in CSV output
    */
   String getToolInfo(String toolHandle) {

      // Get the port handle info from PHINF
      PortHandleInfo info = capi.portHandleInfo(toolHandle);

      // Return the ID and SerialNumber the desired string format
      String outputString = info.getToolId();
      outputString += (" s/n:" + info.getSerialNumber());
      return outputString;
   }

   /**
    * Returns a string representation of the data in CSV format.
    * The CSV format is: "Frame#,ToolHandle,Face,TransformStatus,q0,qx,qy,qz,tx,ty,tz,error,#markers,[Marker1:status,x,y,z],[Marker2..."
    */
   String toolDataToCSV (ToolData toolData) {
      StringBuilder sb = new StringBuilder();
      NumberFormat fmt = new NumberFormat ("%.6g");
      sb.append (toolData.frameNumber + "," + "Port:" + toolData.transform.toolHandle + ",");
      sb.append (toolData.transform.getFaceNumber() + ",");

      if (toolData.transform.isMissing()) {
         sb.append ("Missing,,,,,,,,");
      }
      else {
         sb.append (Transform.Status.findValue(toolData.transform.getErrorCode()) +  ",");
         Transform tr = toolData.transform;
         sb.append (fmt.format(tr.q0)+","+fmt.format(tr.qx)+","+fmt.format(tr.qy)+","+fmt.format(tr.qz)+",");
         sb.append (fmt.format(tr.tx)+","+fmt.format(tr.ty)+","+fmt.format(tr.tz)+","+fmt.format(tr.error));
      }

      // Each marker is printed as: status,tx,ty,tz
      sb.append ("," + toolData.markers.size());
      for ( int i = 0; i < toolData.markers.size(); i++) {
         MarkerData mkr = toolData.markers.get(i);
         sb.append ("," + MarkerData.Status.findValue(mkr.status));
         if (mkr.status == MarkerData.Status.Missing.ordinal()) {
            sb.append (",,,");
         }
         else {
            sb.append (","+fmt.format(mkr.x)+","+fmt.format(mkr.y)+","+fmt.format(mkr.z));
         }
      }
      return sb.toString();
   }

   /**
    * Write tracking data to a CSV file in the format:
    * "#Tools,ToolInfo,Frame#,[Tool1],Frame#,[Tool2]..."
    * 
    * It's worth noting that the number lines in the file does not necessarily
    * match the number of frames collected.  NDI measurement systems support
    * different types of tools: passive, active, and active-wireless.  Because
    * different tool types are detected in different physical ways, each tool
    * type has a separate frame for collecting data for all tools of that
    * type. Each line of the file has the same number of tools, but each tool
    * may have a different frame number that corresponds to its tool type.

    * @param fileName The file to write to.
    * @param numberOfLines The number of lines to write.
    * @param enabledTools ToolData storing the serial number for each enabled tool.
    */
   void writeCSV(
      String fileName, int numberOfLines, ArrayList<ToolData> enabledTools) throws IOException {
      
      // Print header information to the first line of the output file
      System.out.println ("\nWriting CSV file...");
      PrintWriter csvw =
         new PrintWriter (
            new BufferedWriter (new FileWriter (fileName)));

      csvw.print ("#Tools");

         // Loop to gather tracking data and write to the file
         int linesWritten = 0;
      int previousFrameNumber = 0; // use this variable to avoid printing duplicate data with BX
      while (linesWritten < numberOfLines) {
         // Get new tool data using BX2
         ArrayList<ToolData> newToolData =
            apiSupportsBX2 ?
            capi.getTrackingDataBX2("--6d=tools --3d=tools --sensor=none --1d=buttons") :
            capi.getTrackingDataBX(
               (short)(TrackingReplyOption.TransformData.getMask() |
                       TrackingReplyOption.AllTransforms.getMask()));

         // Update enabledTools array with new data
         for (int t = 0; t < enabledTools.size(); t++) {
            for (int j = 0; j < newToolData.size(); j++) {
               if (enabledTools.get(t).transform.toolHandle == newToolData.get(j).transform.toolHandle) {
                  // Copy the new tool data
                  newToolData.get(j).toolInfo = enabledTools.get(t).toolInfo; // keep the serial number
                  enabledTools.get(t).set (newToolData.get(j)); // use the new data
               }
            }
         }

         if (newToolData.size() == 0) {
            throw new IOException ("Error reading binary data");
         }


         // If we're using BX2 there's extra work to do because BX2 and BX use opposite philosophies.
         // BX will always return data for all enabled tools, but some of the data may be old: #linesWritten == # BX commands
         // BX2 never returns old data, but cannot guarantee new data for all enabled tools with each call: #linesWritten <= # BX2 commands
         // We want a CSV file with data for all enabled tools in each line, but this requires multiple BX2 calls.
         if (apiSupportsBX2) {
            // Count the number of tools that have new data
            int newDataCount = 0;
            for (int t = 0; t < enabledTools.size(); t++) {
               if (enabledTools.get(t).dataIsNew) {
                  newDataCount++;
               }
            }

            // Send another BX2 if some tools still have old data
            if (newDataCount < enabledTools.size())	{
               continue;
            }
         }
         else {
            if (previousFrameNumber == enabledTools.get(0).frameNumber) {
               // If the frame number didn't change, don't print duplicate data to the CSV, send another BX
               continue;
            }
            else {
               // This frame number is different, so we'll print a line to the CSV, but remember it for next time
               previousFrameNumber = enabledTools.get(0).frameNumber;
            }
         }

         // If this is the first line of the CSV, print labels for tool and marker data
         if (linesWritten == 0) {
            for (int t = 0; t < enabledTools.size(); t++) {
               csvw.print (",ToolInfo,Frame#,PortHandle,Face#,TransformStatus,Q0,Qx,Qy,Qz,Tx,Ty,Tz,Error,#Markers");
               for (int m = 0; m < enabledTools.get(t).markers.size(); m++) {
                  csvw.print (",Marker" + m  + ".Status,Tx,Ty,Tz");
               }
            }
            csvw.println ("");
         }

         // Print a line of the CSV file if all enabled tools have new data
         csvw.print (enabledTools.size());
         for (int t = 0; t < enabledTools.size(); t++) {
            csvw.print ("," + enabledTools.get(t).toolInfo + "," + toolDataToCSV(enabledTools.get(t)));
            enabledTools.get(t).dataIsNew = false; // once printed, the data becomes "old"
         }
         csvw.println ("");
         linesWritten++;
      }
      csvw.close();
   }

   /**
    * Prints a ToolData object to stdout
    * @param toolData The data to print
    */
   void printToolData(ToolData toolData) {
      
      if (toolData.systemAlerts.size() > 0){
         System.out.println ("[" + toolData.systemAlerts.size() + " alerts] ");
         for (int a = 0; a < toolData.systemAlerts.size(); a++) {
            System.out.println (toolData.systemAlerts.get(a).toString());
         }
      }

      if (toolData.buttons.size() > 0) {
         System.out.println ("[buttons: ");
         for (int b = 0; b < toolData.buttons.size(); b++) {
            System.out.print (ButtonState.findValue(toolData.buttons.get(b))+" ");
         }
         System.out.println ("] ");
      }
      System.out.println (toolDataToCSV(toolData));
   }

   /**
    * Put the system into tracking mode, and poll a few frames of data.
    */
   void printTrackingData() {
      
      // Start tracking, output a few frames of data, and stop tracking

      for (int i = 0; i < 10; i++) {
         // Demonstrate TX command: ASCII command sent, ASCII reply received
         System.out.println (capi.getTrackingDataTX((short)0));

         // Demonstrate BX or BX2 command
         ArrayList<ToolData> toolData =
            apiSupportsBX2 ? capi.getTrackingDataBX2("") : capi.getTrackingDataBX((short)0);

         // Print to stdout in similar format to CSV
         System.out.println ("[alerts] [buttons] Frame#,ToolHandle,Face#,TransformStatus,Q0,Qx,Qy,Qz,Tx,Ty,Tz,Error,#Markers,State,Tx,Ty,Tz");
         for (ToolData td : toolData) {
            printToolData(td);
         }
      }
   }

   /*
    * @brief Streams a command over a socket different than the one used for command/response
    * @details Note the streaming replies could be interleaved with command/response on the same socket,
    * but more sophisticated threadsafe code is necessary to ensure replies are processed correctly.
    */
   void streamTcp()
   {
      String streamId = "tcpStream1";
      // capi.startStreaming("TX 0801", streamId, Protocol::TCP, "12345");
      capi.startStreaming("TX 0801", streamId, CombinedApi.Protocol.TCP, "12345");
      while (!detectKeyStroke()) {
         System.out.println (capi.readStream(streamId));
         sleepSeconds (0.1);
      }
      capi.stopStreaming(streamId);
   }

   /**
    * Initialize and enable loaded tools. This is the same regardless of tool type.
    */
   void initializeAndEnableTools(ArrayList<ToolData> enabledTools) {
      System.out.println ("Initializing and enabling tools...");

      // Initialize and enable tools
      ArrayList<PortHandleInfo> portHandles =
         capi.portHandleSearchRequest(PortHandleSearchRequestOption.NotInit);
      for (PortHandleInfo ph :  portHandles) {
         onErrorPrintDebugMessage(
            "capi.portHandleInitialize()", capi.portHandleInitialize(ph.getPortHandle()));
         onErrorPrintDebugMessage(
            "capi.portHandleEnable()", capi.portHandleEnable(ph.getPortHandle()));
      }

      // Print all enabled tools
      portHandles = capi.portHandleSearchRequest(PortHandleSearchRequestOption.Enabled);
      for (PortHandleInfo ph :  portHandles) {
         System.out.println (ph);
      }

      // Lookup and store the serial number for each enabled tool
      for (PortHandleInfo ph :  portHandles) {
         ToolData tool = new ToolData();
         enabledTools.add(tool);
         tool.transform.toolHandle = (short)Utils.hexStringToInt(ph.getPortHandle());
         tool.toolInfo = getToolInfo(ph.getPortHandle());
      }
   }

   /**
    * Loads a tool from a tool definition file (.rom)
    */
   void loadTool (String toolDefinitionFilePath) {
      // Request a port handle to load a passive tool into
      int portHandle = capi.portHandleRequest();
      onErrorPrintDebugMessage("capi.portHandleRequest()", portHandle);

      // Load the .rom file using the previously obtained port handle
      capi.loadSromToPort(toolDefinitionFilePath, portHandle);
   }

   /**
    * Demonstrate detecting active tools.
    * Active tools are connected through a System Control Unit (SCU) with physical wires.
    */
   void configureActiveTools (String scuHostname) {
      // Setup the SCU connection for demonstrating active tools
      System.out.println ("Configuring Active Tools - Setup SCU Connection");
      onErrorPrintDebugMessage("capi.setUserParameter()", capi.setUserParameter("Param.Connect.SCU Hostname", scuHostname));
      System.out.println (capi.getUserParameter("Param.Connect.SCU Hostname"));

      // Wait a few seconds for the SCU to detect any wired tools plugged in
      System.out.println ("\nDemo Active Tools - Detecting Tools...");
      sleepSeconds(1);

      // Print all port handles
      ArrayList<PortHandleInfo> portHandles = capi.portHandleSearchRequest(PortHandleSearchRequestOption.NotInit);
      for (int i = 0; i < portHandles.size(); i++) {
         System.out.println (portHandles.get(i));
      }
   }

   /**
    * Demonstrate loading an active wireless tool.
    * Active wireless tools are battery powered and emit IR in response to a chirp from the illuminators.
    */
   void configureActiveWirelessTools() {
      // Load an active wireless tool definitions from a .rom files
      System.out.println ("Configuring an Active Wireless Tool - Loading .rom File...");
      loadTool("sroms/active-wireless.rom");
   }

   /**
    * Demonstrate loading dummy tools of each tool type.
    * Dummy tools are used to report 3Ds in the absence of real tools.
    *          Dummy tools should not be loaded with regular tools of the same type.
    *          TSTART will fail if real and dummy tools are enabled simultaneously.
    */
   void configureDummyTools() {
      System.out.println ("Loading passive, active-wireless, and active dummy tools...");
      onErrorPrintDebugMessage("capi.loadPassiveDummyTool()", capi.loadPassiveDummyTool());
      onErrorPrintDebugMessage("capi.loadActiveWirelessDummyTool()", capi.loadActiveWirelessDummyTool());
      onErrorPrintDebugMessage("capi.loadActiveDummyTool()", capi.loadActiveDummyTool());
   }

   /**
    * Demonstrate getting/setting user parameters.
    */
   void configureUserParameters() {
      System.out.println (capi.getUserParameter("Param.User.String0"));
      onErrorPrintDebugMessage(
         "capi.setUserParameter(Param.User.String0, customString)",
         capi.setUserParameter("Param.User.String0", "customString"));
      System.out.println (capi.getUserParameter("Param.User.String0"));
      onErrorPrintDebugMessage(
         "capi.setUserParameter(Param.User.String0, emptyString)",
         capi.setUserParameter("Param.User.String0", ""));
   }

   /**
    * Sets the user parameter "Param.Simulated Alerts" to test communication of system alerts.
    * This method does nothing if simulatedAlerts is set to 0x00000000.
    */
   void simulateAlerts(int simulatedAlerts) {
      // Simulate alerts if any were requested
      if (simulatedAlerts > 0x0000) {
         System.out.println ("\nSimulating system alerts...");
         onErrorPrintDebugMessage(
            "capi.setUserParameter(Param.Simulated Alerts, alerts)",
            capi.setUserParameter("Param.Simulated Alerts", Integer.toString(simulatedAlerts)));
         System.out.println (capi.getUserParameter("Param.Simulated Alerts"));
      }
   }

   /**
    * Determines whether an NDI device supports the BX2 command by
    * looking at the API revision
    */
   void determineApiSupportForBX2() {
      
      // Lookup the API revision
      String response = capi.getApiRevision();

      // Refer to the API guide for how to interpret the APIREV response
      char deviceFamily = response.charAt(0);
      int majorVersion = Utils.stringToInt(response.substring(2,5));

      // As of early 2017, the only NDI device supporting BX2 is the Vega
      // Vega is a Polaris device with API major version 003
      if ( deviceFamily == 'G' && majorVersion >= 3) {
         apiSupportsBX2 = true;
         apiSupportsStreaming = true;
      }
   }

   class StreamingThread extends Thread {

      public void run() {
         streamTcp();
      }
   }

   void dorun (String[] args) {
      if (args.length < 1 || args.length > 3) {
         System.out.println (
            "CapiSample Ver " + capi.getVersion() +"\n" +
	    "usage: java artisynth.istar.NDI.CapiSample <hostname> [args]\n" +
	    "where:\n" +
            "    <hostname>      (required) The measurement device's hostname, IP address, or serial port.\n" +
            "    [args]          (optional) Any other arguments such as tools to load, or SCU to connect to.\n" +
            "example hostnames:\n" +
            "    Connecting to device by IP address: 169.254.8.50\n" +
            "    Connecting to device by hostname: P9-B0103.local\n" +
            "    Connecting to serial port varies by operating system:\n" +
            "        COM10 (Windows), /dev/ttyUSB0 (Linux), /dev/cu.usbserial-001014FA (Mac)\n" +
            "Optional arguments:\n" +
            "--scu=[scu_hostname] A System Control Unit (SCU) hostname, used to connect active tools.\n" +
            "--stream=UDP Specify the streaming protocol as UDP (default is TCP)\n" +
            "--tools=[file1.rom],[file2.rom]... A comma delimited list of tools to load.");
         return;
      }

      String hostname = args[0];

      // Look for optional arguments
      String scu_hostname = "";
      ArrayList<String> toolDefinitions = new ArrayList<String>();
      boolean streamUDP = false;
      for (int i = 1; i < args.length; i++) {
         String arg = args[i];
         if (arg.startsWith("--scu=")) {
            scu_hostname = arg;
         }
         else if (arg.startsWith("--stream=UDP")) {
            streamUDP = true;
         }
         else if (arg.startsWith("--tools=")) {
            String[] toolList = arg.substring(8).split(",");
            for (String tool : toolList) {
               if (new File(tool).canRead()) {
                  toolDefinitions.add(tool);
               }
               else {
                  System.out.println (
                     "Cannot access file: " + tool);
               }
            }
         }
      }

      // Attempt to connect to the device
      if (capi.connect(hostname) != 0) {
         // Print the error and exit if we can't connect to a device
         System.out.println ("Connection Failed!");
         return;
      }
      System.out.println ("Connected!");

      // Wait a second - needed to support connecting to LEMO Vega
      sleepSeconds(1);

      // Print the firmware version for debugging purposes
      System.out.println (
         capi.getUserParameter("Features.Firmware.Version"));

      // Determine if the connected device supports the BX2 command
      determineApiSupportForBX2();

      // Initialize the system. This clears all previously loaded tools, unsaved settings etc...
      onErrorPrintDebugMessage("capi.initialize()", capi.initialize());

      // Demonstrate error handling by asking for tracking data in the wrong mode
      System.out.println (capi.getTrackingDataTX((short)0));

      // Demonstrate getting/setting user parameters
      configureUserParameters();

      // Load any passive tool definitions from a .rom files
      if (toolDefinitions.size() > 0) {
         System.out.println ("Loading Tool Definitions (.rom files) ...");
         for (int f = 0; f < toolDefinitions.size(); f++) {
            System.out.println ("Loading: " + toolDefinitions.get(f));
            loadTool(toolDefinitions.get(f));
         }
      }

      // Wired tools are connected through a System Control Unit (SCU)
      if (scu_hostname.length() > 0) {
         configureActiveTools(scu_hostname);
      }

      // Dummy tools are used to report 3Ds in the absence of real tools.
      // TSTART will fail if real and dummy tools of the same type are enabled simultaneously.
      // To experiment with dummy tools, comment out the previous tool configurations first.
      // configureDummyTools();

      // Once loaded or detected, tools are initialized and enabled the same way
      // PHSR can be time consuming, so store the tool metadata immediately (eg. port handle, serial number)
      ArrayList<ToolData> enabledTools = new ArrayList<ToolData>();
      initializeAndEnableTools(enabledTools);

      // Print an error if no tools were specified
      if (enabledTools.size() == 0) {
         System.out.println (
            "No tools detected. To load passive tools, specify: --tools=[tool1.rom],[tool2.rom]");
      }
      // Spoofing system faults is handy to see how system alerts are handled, but faults can
      // prevent tracking data from being returned. This sample application has valued code simplicity
      // above robustness, so it does not gracefully handle every error case. You can experiment with
      // system faults, but attempting to do other tasks (eg. writing a .csv of tracking data) may fail
      // depending on what faults have been simulated.
      // simulateAlerts(0x7FFFFFFF);

      // Once the system is put into tracking mode, data is returned for whatever tools are enabled
      System.out.println ("Entering tracking mode...");
      onErrorPrintDebugMessage(
         "capi.startTracking()", capi.startTracking());

      System.out.println ("\nGet tracking data");
      System.out.println (capi.getTrackingDataTX());


      if (true || !apiSupportsStreaming) {
         // Demonstrate polling a few frames of data
         printTrackingData();

         // Write a CSV file
         try {
            writeCSV("example.csv", 50, enabledTools);
         }
         catch (IOException e) {
            System.out.println ("Error writeing example.csv, "+e);
         }
      }
      else {
         // Stream tracking data to the terminal on another thread
         Thread streamingThread = new StreamingThread();
         streamingThread.start();
         try {
            // Command/response is possible while data is streaming.
            // Keep the command channel alive by issuing commands.
            // Param.Connect.Idle Timeout=300s by default will terminate the connection if there is no activity.
            while (!detectKeyStroke()) {
               // Periodically read the gravity vector
               sleepSeconds(1);
               capi.getUserParameter("Info.Status.Gravity Vector");
            }
            
            // Wait for the streaming thread to stop and cleanup
            streamingThread.join();
         }
         catch (InterruptedException e) {
            System.out.println ("Streaming thread intertupted");
         }
      }

      // Stop tracking (back to configuration mode)
      System.out.println (
         "\nLeaving tracking mode and returning to configuration mode...");
      onErrorPrintDebugMessage(
         "capi.stopTracking()", capi.stopTracking());

      System.out.println ("CAPI demonstration complete");
   }

   public static void main (String[] args) {

      CapiSample sample = new CapiSample();
      sample.dorun (args);
   }
}
