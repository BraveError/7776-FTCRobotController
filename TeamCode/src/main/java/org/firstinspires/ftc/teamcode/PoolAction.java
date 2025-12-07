package org.firstinspires.ftc.teamcode;

import java.util.Set;
import java.util.Iterator;
import com.acmerobotics.roadrunner.Action;
import static java.lang.System.currentTimeMillis;

public class PoolAction implements Action {
    private Set<Action> ActiveActions;

    public PoolAction() {
        this.ActiveActions = new Set<Action>();
    }

    public void Add(Action action) {
        ActiveActions.add(action);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        Iterator<Action> iterator = ActiveActions.iterator();
        while (iterator.hasNext()) {
            Action action = iterator.next();
            if (!action.run(packet)) {
                iterator.remove();
            }
        }

        return true;
    }
}
