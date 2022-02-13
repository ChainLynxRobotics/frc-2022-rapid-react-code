package frc.robot;

import java.util.List;

public class SerialPorts {
    private DatumSerial datumSerial;
    public SerialPorts(){
        datumSerial = new DatumSerial();
        List<String> portNames= datumSerial.getPorts();
        System.out.println("Available Ports:");
        for(int i = 0; 1<portNames.size(); ++i){
            System.out.println(portNames.get(i));
        }
    }
}
