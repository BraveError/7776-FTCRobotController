package org.firstinspires.ftc.teamcode;

import java.util.function.DoubleConsumer;
import com.acmerobotics.roadrunner.Action;
import static java.lang.System.currentTimeMillis;

public class UpdateAction implements Action {
    double LastTime = currentTimeMillis();
    DoubleConsumer Func;

    public UpdateAction(double LastTime, DoubleConsumer Func) {
        this.LastTime = LastTime;
        this.Func = Func;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        long CurrTime = currentTimeMillis() / 1000.0;
        double DeltaTime = CurrTime - LastTime;
        LastTime = CurrTime;

        this.Func.accept(DeltaTime);

        return true;
    }
}
