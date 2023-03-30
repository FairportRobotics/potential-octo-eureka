package com.fairportrobotics.frc.poe.controllers.lighting;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.SerialPort;


public class ArduinoLightingController {
    
    final SerialPort bus;
    private final static String NULL_COLOR = "000000000";
    private final static String RAINBOW_VALUE =  NULL_COLOR + "254";
    private final static String SHIFT_VALUE = NULL_COLOR + "251";
    private final static String SHIFT_WRAP_VALUE = NULL_COLOR + "250";

    private Thread lightingRunnerThread;
    private boolean running = true;
    private ArrayList<String> commandQueue = new ArrayList<>();

    public ArduinoLightingController(int baudRate, SerialPort.Port port) {
        this.bus = new SerialPort(baudRate, port);

        this.lightingRunnerThread = new Thread(new Runnable() {
            public void run(){
                while(running){
                    for(String command : commandQueue){
                        bus.writeString(command);
                        try{
                            Thread.sleep(500);
                        }catch(InterruptedException e){
                            e.printStackTrace();
                        }
                    }

                }
            }
        });

        lightingRunnerThread.start();

    }

    public void executeCommand(LightingCommand command) {

        commandQueue.addAll(command.getCommands());

        // if ( validateColor(colorValue) ) {
        //     return bus.writeString( colorValue + command );
        // }
        // return 0;
    }

    public static class LightingCommandBuilder{

        private ArrayList<String> commands;

        public LightingCommandBuilder(){
            commands= new ArrayList<>();
        }

        public LightingCommandBuilder setSpecificLED(String colorValue, String numberLED){
            commands.add(colorValue + numberLED);
            return this;
        }

        public LightingCommandBuilder fillAll(String colorValue){
            commands.add(colorValue + "255");
            return this;
        }

        public LightingCommandBuilder shiftWrap(){
            commands.add(SHIFT_WRAP_VALUE);
            return this;
        }

        public LightingCommandBuilder fillShift(String colorValue){
            commands.add(colorValue + "252");
            return this;
        }

        public LightingCommandBuilder shift(){
            commands.add(SHIFT_VALUE);
            return this;
        }
        
        public LightingCommandBuilder fillRainbow(){
            commands.add(RAINBOW_VALUE);
            return this;
        }

        public LightingCommandBuilder flash(String colorValue){
            commands.add(colorValue + "249");
            return this;
        }

        public LightingCommandBuilder switchGradient(String colorValue){
            commands.add(colorValue + "253");
            return this;
        }

        public LightingCommand build(){
            return new LightingCommand(commands);
        }

    }

    public static class LightingCommand{
        private ArrayList<String> commands;

        public LightingCommand(ArrayList<String> commands){
            this.commands = commands;
        }

        public ArrayList<String> getCommands(){
            return commands;
        }

    }

}
