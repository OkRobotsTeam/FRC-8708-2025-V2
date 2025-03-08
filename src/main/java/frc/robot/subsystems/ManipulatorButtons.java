package frc.robot.subsystems;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Objects;

public class ManipulatorButtons extends SubsystemBase {
    private final StringSubscriber keySubscriber;
    private double lastChange = 0;

    public ManipulatorButtons(StringTopic keyTopic) {
        keySubscriber = keyTopic.subscribe("");
    }

    public String getKey() {
        double thisChange = keySubscriber.getLastChange();
        if (thisChange > lastChange) {
            lastChange = thisChange;
            return keySubscriber.get();
        } else {
            return "";
        }
    }

    public void checkButtons(Elevator elevator) {
        String key = getKey();
        if (!Objects.equals(key, "")) {
            if (Objects.equals(key, "a")) {
                System.out.println("Transitioning to state 4");
                elevator.transitionToState(4);
            } else if (Objects.equals(key, "b")) {
                System.out.println("Transitioning to state 3");
                elevator.transitionToState(3);
            } else if (Objects.equals(key, "c")) {
                System.out.println("Transitioning to state 2");
                elevator.transitionToState(2);
            } else if (Objects.equals(key, "d")) {
                System.out.println("Transitioning to state 1");
                elevator.transitionToState(1);
            } else if (Objects.equals(key, "e")) {
                System.out.println("Transitioning to state 0");
                elevator.transitionToState(0);
            }
        }
    }
}
