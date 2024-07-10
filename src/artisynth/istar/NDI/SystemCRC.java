package artisynth.istar.NDI;

public class SystemCRC {

   int[] crcTable = new int[256];

   public SystemCRC() {

      long lCrcTable;
      for(int i = 0; i < 256; i++) {
         lCrcTable = i;
         for(int j = 0; j < 8; j++) {
            lCrcTable = (lCrcTable >>> 1) ^ ((lCrcTable & 1) != 0 ? 0xA001L : 0 );
         }
         crcTable[i] = (int) lCrcTable & 0xFFFF;
      }
   }

   public int calcValue (int crc, int data) {
	crc = crcTable[(crc^data) & 0xFF]^(crc >>> 8);
	return (crc & 0xFFFF);
   }

   public int calculateCRC16 (byte[] reply, int replyLength) {
      int uCrc = 0;
      for (int m = 0; m < replyLength ; m++) {
         uCrc = calcValue(uCrc, reply[m]);
      }
      return uCrc;
   }

   public int calculateCRC16 (byte[] bytes) {
      return calculateCRC16 (bytes, bytes.length);
   }

   public int calculateCRC16 (String str) {
      byte[] bytes = str.getBytes();
      return calculateCRC16 (bytes, bytes.length);
   }
}
   
