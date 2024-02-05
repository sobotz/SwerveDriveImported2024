package frc.robot.subsystems;
public class Vector {
    double[] array;
    double x;
    double y;
    double magnatude;
    double degree;
    public Vector(double x, double y){
        xyreset(x,y);
        /*array = new double[2];
        this.x = x;
        this.y = y;
        array[0] = x;
        array[1] = y;
        this.magnatude = Math.abs(Math.sqrt(Math.pow(x,2) + Math.pow(y,2)));
        degree = Math.abs(Math.toDegrees(Math.atan(Math.abs(x)/Math.abs(y))));
        
        if (x>0){
            degree = 180 + degree;
            if (y>0){
                degree = + 180 - degree;
            }
        }
        else if(x<0){
            if (y<0){
                degree = 180 - degree;
            }
        }
        else if (y==0){
            if (x>0){
                degree = 270;
            }
            else if (x<0){
                degree = 90;
            }
            else{
                degree = 0;
            }
        }
        else if (x==0){
            if (y>0){
                degree = 0;
            }
            else if (y<0){
                degree = 180;
            }
            else{
                degree = 0;
            }
        }
        if (degree < 0){
            degree += 360;
        }*/
    }
    public void xyreset(double x, double y){
        array = new double[2];
        this.x = x;
        this.y = y;
        array[0] = x;
        array[1] = y;
        this.magnatude = Math.abs(Math.sqrt(Math.pow(x,2) + Math.pow(y,2)));
        degree = Math.abs(Math.toDegrees(Math.atan(Math.abs(x)/Math.abs(y))));
        if (x>0){
            degree = 180 + degree;
            if (y>0){
                degree = + 180 - degree;
            }
        }
        else if(x<0){
            if (y<0){
                degree = 180 - degree;
            }
        }
        else if (y==0){
            if (x>0){
                degree = 270;
            }
            else if (x<0){
                degree = 90;
            }
            else{
                degree = 0;
            }
        }
        else if (x==0){
            if (y>0){
                degree = 0;
            }
            else if (y<0){
                degree = 180;
            }
            else{
                degree = 0;
            }
        }
        if (degree < 0){
            degree += 360;
        }
    }
    public Vector(double magnatude2, double direction,boolean magnatudeVectorCreation){
        this.magnatude = Math.abs(magnatude2);
        this.degree = direction;
        if (direction == 0){
            this.x = 0;
            this.y = magnatude;
        }
        else if (direction < 90){
            this.x = -magnatude * Math.abs(Math.sin(Math.toRadians(direction)));
            this.y = magnatude * Math.abs(Math.cos(Math.toRadians(direction)));
        }
        else if (direction == 90){
            this.x = -magnatude;
            this.y = 0;
        }
        else if (90<direction && direction < 180){
            this.x = -magnatude * Math.abs(Math.sin(Math.toRadians(direction)));
            this.y = -magnatude * Math.abs(Math.cos(Math.toRadians(direction)));
        }
        else if (direction == 180){
            this.x = 0;
            this.y = -magnatude;
        }
        else if (180 < direction && direction < 270){
            this.x = magnatude * Math.abs(Math.sin(Math.toRadians(direction)));
            this.y = -magnatude * Math.abs(Math.cos(Math.toRadians(direction)));
        }
        else if (direction == 270){
            this.x = magnatude;
            this.y = 0;
        }
        else if (270<direction && direction < 360){
            this.x = magnatude * Math.abs(Math.sin(Math.toRadians(direction)));
            this.y = magnatude * Math.abs(Math.cos(Math.toRadians(direction)));
        }
        /*double placeHolder = x;
        x = y;
        y = placeHolder;*/
        

    }
    public double[] getVectorPoints(){
        return array;
    }
    public double getVectorX(){
        return x;
        
    }
    
    public double getVectorY(){
        return y;
    }
    public void setVectorX(double xx){
        xyreset(x + xx,y);
    }
    public void setVectorY(double yy){
        xyreset(x, y+ yy);
    }
    public void addVectorX(double xx){
        xyreset(x + xx, y);
    }
    public void addVectorY(double yy){
        xyreset(x ,y + yy);
    }

    public double getDegree(){
        return degree;
    }
    public double getMagnatude(){
        return magnatude;
    }
    public Vector addVector(Vector v){
        double combinedX = x + v.getVectorX();
        double combinedY = y + v.getVectorY();
        if (-0.0000000001<combinedX && combinedX<0.0000000001){
            combinedX = 0;
        }
        if (-0.0000000001<combinedY && combinedY <0.0000000001){
            combinedY = 0;
        }
        Vector newVector = new Vector(combinedX,combinedY);
        //double[] combinedVector = {combinedX, combinedY};
        return newVector;
    }
}
