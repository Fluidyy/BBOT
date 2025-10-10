package frc.robot;

public class Constants {

    public static enum RobotState {
        IDLE, // Robot is not doing anything2
        INTAKING, // Robot is picking up a game piece
        SCORING, // Robot is shooting a game piece
        CLIMBING;
      }
      public static enum coralState {
        intake, // Robot is not doing anything2
        scoring, // Robot is driving
        intakedpeice,
        idle; // Robot is picking up a game piece
      
      }
    public static coralState currentState = coralState.idle;
    public static coralState getcoralState(){
        return currentState;
    }
    public static coralState setcoralState(coralState state){
        currentState = state;
        return currentState;
    }
    
}
