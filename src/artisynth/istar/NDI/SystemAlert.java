package artisynth.istar.NDI;

public class SystemAlert {

   public static enum Type {
      Fault,
      Alert,
      Event;

      public Type findValue (int num) {
         if (num < values().length) {
            return values()[num];
         }
         else {
            return null;
         }
      }
   };

   public static enum Fault {
      Ok, // 0x00
      FatalParameter, // 0x01
      SensorParameter, // 0x02
      MainVoltage, // 0x03
      SensorVoltage, // 0x04
      IlluminatorVoltage, // 0x05
      IlluminatorCurrent, // 0x06
      Sensor0Temp, // 0x07
      Sensor1Temp, // 0x08
      MainTemp, // 0x09
      SensorMalfunction; // 0x0a

      public static Fault findValue (int num) {
         if (num < values().length) {
            return values()[num];
         }
         else {
            return null;
         }
      }
   };


   public static enum Alert {
      Ok, // 0x0000
      BatteryLow, // 0x0001
      BumpDetected, // 0x0002
      IncompatibleFirmware, // 0x0003
      NonFatalParameter, // 0x0004
      FlashMemoryFull, // 0x0005
      Reserved06, // 0x0006
      StorageTempExceeded, // 0x0007
      TempHigh, // 0x0008
      TempLow, // 0x0009
      ScuDisconnected, // 0x000a
      Reserved0b, // 0x000b
      Reserved0c, // 0x000c
      Reserved0d, // 0x000d
      PtpClockSynch; // 0x000e

      public static Alert findValue (int num) {
         if (num < values().length) {
            return values()[num];
         }
         else {
            return null;
         }
      }
   };

   public static enum Event {
      Ok, //0x0000
      ToolPluggedIn, // 0x0001
      ToolUnplugged, // 0x0002
      SiuPluggedIn, // 0x0003
      SiuUnplugged; // 0x0004

      public static Event findValue (int num) {
         if (num < values().length) {
            return values()[num];
         }
         else {
            return null;
         }
      }
   };

   byte conditionType;
   int conditionCode;
}
