package frc.robot.util;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation3d;

public class FuelSim {
    private static final double PERIOD = 0.02; // sec
    private static final Translation3d GRAVITY = new Translation3d(0, 0, -9.81); // m/s^2
    private static final double FIELD_COR = Math.sqrt(22 / 51.5); // coefficient of restitution with the field
    private static final double FUEL_COR = Math.sqrt(22 / 51.5); // coefficient of restitution with another fuel
    private static final double FUEL_RADIUS = 0.075;

    private class Fuel {
        private Translation3d pos;
        private Translation3d vel;

        private Fuel(Translation3d pos, Translation3d vel) {
            this.pos = pos;
            this.vel = vel;
        }

        private Fuel(Translation3d pos) {
            this(pos, new Translation3d());
            
        }

        public void update() {
            pos = pos.plus(vel.times(PERIOD));
            vel = vel.plus(GRAVITY);
            handleFieldCollisions();
        }

        private void handleFieldCollisions() {
            if (pos.getZ() < FUEL_RADIUS && vel.getZ() < 0) {
                pos = pos.plus(new Translation3d(0, 0, FUEL_RADIUS - pos.getZ()));
                vel = vel.plus(new Translation3d(0, 0, -(1+FIELD_COR)*vel.getZ()));
            }
        }

        private void addImpulse(Translation3d impulse) {
            vel = vel.plus(impulse);
        }

        private Translation3d getCollisionNormalVector(Fuel other) {
            Translation3d normal = other.pos.minus(this.pos);
            normal = normal.div(normal.getNorm());

            return normal;
        }
    }

    private static void handleFuelCollision(Fuel a, Fuel b) {
        Translation3d normal = b.pos.minus(a.pos);
        double impulse = 0.5*(1+FUEL_COR)*(b.vel.minus(a.vel).dot(normal));
        a.addImpulse(normal.times(-impulse));
        b.addImpulse(normal.times(impulse));
    }

    private static void handleFuelCollisions(ArrayList<Fuel> fuels) {
        for (int i = 0; i < fuels.size()-1; i++) {
            for (int j = i+1; j < fuels.size(); j++) {
                handleFuelCollision(fuels.get(i), fuels.get(j));
            }
        }
    }
}
