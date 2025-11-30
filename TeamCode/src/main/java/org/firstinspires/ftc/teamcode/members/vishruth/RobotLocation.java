package org.firstinspires.ftc.teamcode.members.vishruth;

public class RobotLocation {
    double angle;
    double x;
    double y;

    public void changeX(double changeAmount){
        this.x += changeAmount;
    }

    public void changeY(double changeAmount){
        this.y += changeAmount;
    }

    public RobotLocation(double angle,double x,double y){
        if(angle>180){
            this.angle = angle - 180;
        } else if (angle<180) {
            this.angle = angle + 180;
        } else
            this.angle=angle;
        this.x=x;
        this.y=y;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getAngle() {
        return angle;
    }

    @Override
    public String toString() {
        final StringBuilder sb = new StringBuilder("RobotLocation{");
        sb.append("angle=").append(angle);
        sb.append(", x=").append(x);
        sb.append(", y=").append(y);
        sb.append('}');
        return sb.toString();
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    public void turn(double changeAngle){
        this.angle += changeAngle;
    }
}
