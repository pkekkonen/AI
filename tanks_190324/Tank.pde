/** Ida Söderberg, Magnus Palmstierna och Paulina Lagebjer Kekkonen (Grupp 5) **///<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>//

import java.util.Comparator; //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>//
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.List;
import java.util.LinkedList;
import java.util.Collections;
import java.util.Arrays;

class Tank extends Sprite {
  int id;
  //String name; //Sprite
  int team_id;

  PVector acceleration;
  PVector velocity;
  //PVector position; //Sprite

  float rotation;
  float rotation_speed;

  Team team;
  PImage img;
  //float diameter; //Sprite
  //float radius; //Sprite

  float maxrotationspeed;
  float maxspeed;
  float maxforce;

  int health;// 3 är bra, 2 är mindre bra, 1 är immobilized, 0 är destroyed.
  boolean isImmobilized; // Tanken kan snurra på kanonen och skjuta, men inte förflytta sig.
  boolean isDestroyed; // Tanken är död.

  //PVector hitArea;

  PVector startpos;
  PVector positionPrev; //spara temp senaste pos.

  Node startNode; // noden där tanken befinner sig.
  Node currentNode;
  Node target;
  boolean hasTarget; // Används just nu för att kunna köra "manuellt" (ai har normalt target).
  PVector targetPosition; // Används vid förflyttning mot target.
  float targetHeading; // Används vid rotation mot en target.
  PVector sensor_targetPosition;

  PVector[] otherTanks  = new PVector[5];
  PVector distance3_sensor;

  ArrayList listOfActions; // Används ännu inte.

  float heading; // Variable for heading!

  // variabler som anger vad tanken håller på med.
  boolean backward_state;
  boolean forward_state;
  boolean turning_right_state;
  boolean turning_left_state;
  boolean turning_turret_right_state;
  boolean turning_turret_left_state;
  boolean stop_state;
  boolean stop_turning_state;
  boolean stop_turret_turning_state;
  boolean patrolling;
  boolean tankAhead;

  boolean idle_state; // Kan användas när tanken inte har nåt att göra.
  boolean collidedWithTree;
  boolean collidedWithTank;

  boolean isMoving; // Tanken är i rörelse.
  boolean isRotating; // Tanken håller på att rotera.
  boolean isColliding; // Tanken håller på att krocka.
  boolean isAtHomebase;
  boolean userControlled; // Om användaren har tagit över kontrollen.
  boolean isReporting;
  boolean isReportingInHomebase;
  boolean okayToGoNextStepHome;

  boolean hasShot; // Tanken kan bara skjuta om den har laddat kanonen, hasShot=true.
  CannonBall ball;

  float s = 2.0;
  float image_scale;

  boolean isSpinning; // Efter träff snurrar tanken runt, ready=false.
  boolean isReady; // Tanken är redo för action efter att tidigare blivit träffad.
  int remaining_turns;
  float heading_saved; // För att kunna återfå sin tidigare heading efter att ha snurrat.

  Turret turret;

  // Tank sensors
  private HashMap<String, Sensor> mappedSensors = new HashMap<String, Sensor>();
  private ArrayList<Sensor> sensors = new ArrayList<Sensor>();

  protected ArrayList<Sensor> mySensors = new ArrayList<Sensor>();
  float periphery = PI/314;
  float sightDistance = 1100;

  // List of traversed areas
  private ArrayList<Node> homeNodes = new ArrayList<Node>();
  private ArrayList<Node> enemyNodes = new ArrayList<Node>();
  private Node lastVisitedNode;
  private Node lastSeenEnemy;
  PVector vectorTarget; // Used for patrolling
  Direction lastDir; // Used for reporting

  // Used to make sure tank stays still for three seconds after reporting to homebase
  StopWatchTimer timer;

  //Shortest path home 
  private LinkedList<Node> pathHome = new LinkedList<Node>();
  //**************************************************
  Tank(int id, Team team, PVector _startpos, float diameter, CannonBall ball) {
    println("*** NEW TANK(): [" + team.getId()+":"+id+"]");
    this.id = id;
    this.team = team;
    this.team_id = this.team.getId();

    this.name = "tank";

    this.startpos = new PVector(_startpos.x, _startpos.y);
    this.position = new PVector(this.startpos.x, this.startpos.y);
    //********************************************************************** OBSOBSOBS
    if (this.team_id == 0) {
      this.velocity = new PVector(random(-1, 1), random(-1, 1));
    } else {
      velocity = new PVector(0, 0);
    }

    this.acceleration = new PVector(0, 0);
    this.positionPrev = new PVector(this.position.x, this.position.y); //spara temp senaste pos.
    this.targetPosition = new PVector(this.position.x, this.position.y); // Tanks har alltid ett target.

    this.startNode = grid.getNearestNode(this.startpos);
    this.currentNode = startNode;


    if (this.team.getId() == 0) this.heading = radians(0); // "0" radians.
    if (this.team.getId() == 1) this.heading = radians(180); // "3.14" radians.

    this.targetHeading = this.heading; // Tanks har alltid en heading mot ett target.
    this.hasTarget = false;

    this.diameter = diameter;
    this.radius = this.diameter/2; // For hit detection.

    this.backward_state = false;
    this.forward_state = false;
    this.turning_right_state = false;
    this.turning_left_state = false;
    this.turning_turret_right_state = false;
    this.turning_turret_left_state = false;
    this.stop_state = true;
    this.stop_turning_state = true;
    this.stop_turret_turning_state = true;
    // Under test
    this.isMoving = false;
    this.isRotating = false;
    this.isAtHomebase = true;
    this.idle_state = true;
    this.isReporting = false;
    this.isReportingInHomebase = false;
    this.okayToGoNextStepHome = true;

    this.ball = ball;
    this.hasShot = false;
    this.maxspeed = 2; //3;
    this.maxforce = 0.01;
    this.maxrotationspeed = radians(3);
    this.rotation_speed = 0;
    this.image_scale = 0.5;
    this.isColliding = false;
    this.patrolling = false;
    this.tankAhead = false;

    //this.img = loadImage("tankBody2.png");
    this.turret = new Turret(this.diameter/2);

    this.radius = diameter/2;

    this.health = 3;// 3 är bra, 2 är mindre bra, 1 är immobilized, 0 är oskadliggjord.
    this.isReady = true; // Tanken är redo för action.
    this.isImmobilized = false; // Tanken kan snurra på kanonen och skjuta, men inte förflytta sig.
    this.isDestroyed = false; // Tanken är död.

    this.isSpinning = false;
    this.remaining_turns = 0;
    this.heading_saved = this.heading;

    this.ball.setColor(this.team.getColor());
    this.ball.setOwner(this);

    Node[][] nodes = grid.getAllNodes();
    for (int i = 0; i < nodes.length; i++) {
      for (int j = 0; j < nodes[i].length; j++) {
        if (nodes[i][j].x > team.homebase_x && nodes[i][j].x < team.homebase_x+team.homebase_width && nodes[i][j].y > team.homebase_y && nodes[i][j].y < team.homebase_x+team.homebase_height) {
          homeNodes.add(nodes[i][j]);
        } else if (nodes[i][j].x > width-150 && nodes[i][j].x < width && nodes[i][j].y > height-350 && nodes[i][j].y < height) {
          enemyNodes.add(nodes[i][j]);
        }
      }
    }

    initializeSensors();
  }


  //**************************************************
  int getId() {
    return this.id;
  }

  //**************************************************
  //String getName(){
  //  return this.name;
  //}

  //**************************************************
  float getRadius() {
    return this.radius;
  }

  //**************************************************
  float getHeadingInDegrees() {
    return degrees(this.heading);
  }

  //**************************************************
  float getHeading() {
    return this.heading;
  }

  //**************************************************
  // Anropas då användaren tar över kontrollen av en tank.
  void takeControl() {
    println("*** Tank[" + team.getId()+"].takeControl()");
    stopMoving_state();
    this.userControlled = true;
  }

  //**************************************************
  // Anropas då användaren släpper kontrollen av en tank.
  void releaseControl() {
    println("*** Tank[" + team.getId()+"].releaseControl()");
    stopMoving_state();
    idle_state = true;

    this.userControlled = false;
  }

  //**************************************************
  // Används ännu inte.
  PVector getRealPosition() {
    return this.position;
  }

  //************************************************** 
  // Returns the Sensor with the specified ID

  public Sensor getSensor(String ID) {
    return mappedSensors.get(ID);
  }

  //************************************************** 
  // Add your Sensor.

  public void addSensor(Sensor s) {
    mySensors.add(s);
  }

  //**************************************************
  //Register a sensor inside this robot, with the given ID

  protected void registerSensor(Sensor sensor, String ID) {
    mappedSensors.put(ID, sensor);
    sensors.add(sensor);
  }

  //**************************************************
  protected void initializeSensors() {

    SensorDistance ultrasonic_front = new SensorDistance(this, 0f);
    registerSensor(ultrasonic_front, "ULTRASONIC_FRONT");

    //SensorDistance ultrasonic_back = new SensorDistance(this, 180f);
    //registerSensor(ultrasonic_back, "ULTRASONIC_BACK");

    /*
     SensorCompass compass = new SensorCompass(game, this);
     registerSensor(compass, "COMPASS");
     
     SensorDistance ultrasonic_left = new SensorDistance(game, this, 270f);
     registerSensor(ultrasonic_left, "ULTRASONIC_LEFT");
     
     SensorDistance ultrasonic_right = new SensorDistance(game, this, 90f);
     registerSensor(ultrasonic_right, "ULTRASONIC_RIGHT");
     
     SensorDistance ultrasonic_front = new SensorDistance(game, this, 0f);
     registerSensor(ultrasonic_front, "ULTRASONIC_FRONT");
     
     SensorDistance ultrasonic_back = new SensorDistance(game, this, 180f);
     registerSensor(ultrasonic_back, "ULTRASONIC_BACK");
     */
  }

  //**************************************************

  SensorReading readSensor_distance(Sensor sens) {
    //println("*** Tank.readSensorDistance()");

    Sprite df = sens.readValue().obj();

    return sens.readValue();
  }

  //**************************************************
  void readSensors() {
    /*
     println("*** Tank[" + team.getId()+"].readSensors()");
     println("sensors: " + sensors);
     
     for (Sensor s : mySensors) {
     if (s.tank == this) {
     PVector sens = (s.readValue().obj().position);
     
     //println("============");
     //println("("+sens.x + " , "+sens.y+")");
     //ellipse(sens.x, sens.y, 10,10);
     if (sens != null) {
     line(this.position.x,this.position.y, sens.x, sens.y);
     println("Tank" + this.team.getId() + ":"+this.id + " ( " + sens.x + ", "+ sens.y + " )");
     
     }
     }
     */
  }

  //**************************************************
  void spin(int antal_varv) {
    println("*** Tank[" + team.getId()+"].spin(int)");
    if (!this.isSpinning) {
      this.heading_saved = this.heading;
      isSpinning = true; 
      this.remaining_turns = antal_varv;
    }
  }

  //**************************************************
  // After calling this method, the tank can shoot.
  void loadShot() {
    println("*** Tank[" + team.getId()+":"+id+"].loadShot() and ready to shoot.");

    this.hasShot = true;
    this.ball.loaded();
  }

  //**************************************************
  void testCollisionSensor() {
  }

  //**************************************************
  void fire() {
    // Ska bara kunna skjuta när den inte rör sig.
    if (this.stop_state) {
      println("*** Tank[" + this.team.getId()+":"+this.id+"].fire()");

      if (this.hasShot) {
        println("! Tank["+ this.getId() + "] – PANG.");
        this.hasShot = false;

        PVector force = PVector.fromAngle(this.heading + this.turret.heading);
        force.mult(10);
        this.ball.applyForce(force);

        shoot(this.id); // global funktion i huvudfilen

        soundManager.playSound("tank_firing");
        //soundManager.playSound("blast");
      } else {
        println("! Tank["+ this.getId() + "] – You have NO shot loaded and ready.");
      }
    } else {
      println("! Tank["+ this.getId() + "] – The tank must stand STILL to shoot.");
    }
  }

  //**************************************************
  // Anropad från den cannonBall som träffat.
  final boolean takeDamage() {
    println("*** Tank["+ this.getId() + "].takeDamage()");

    if (!this.isDestroyed) {
      this.health -= 1;

      println("! Tank[" + team.getId()+":"+id+"] has been hit, health is now "+ this.health);

      stopMoving_state();
      resetTargetStates();

      if (!this.isImmobilized) {
        if (this.health == 1) {
          this.isImmobilized = true;
        }
      }

      if (this.health <= 0) {
        this.health = 0;
        this.isDestroyed = true;
        this.isSpinning  = false;
        this.isReady = false;

        return true;
      }

      spin(3);
      this.isReady = false; // Efter träff kan inte tanken utföra action, så länge den "snurrar".


      return true;
    }

    return false; // ingen successfulHit omtanken redan är destroyed.
  }

  //**************************************************
  // Anropad från sin egen cannonBall efter träff.
  final void successfulHit() {
    this.team.messageSuccessfulHit();
  }

  //**************************************************
  // Det är denna metod som får tankens kanon att svänga vänster.
  void turnTurretLeft_state() {
    if (this.stop_state) { 
      if (!this.turning_turret_left_state) {
        println("*** Tank[" + getId() + "].turnTurretLeft_state()");
        this.turning_turret_right_state = false;
        this.turning_turret_left_state = true;
        this.stop_turret_turning_state = false;
      }
    } else {
      println("Tanken måste stå still för att kunna rotera kanonen.");
    }
  }

  //**************************************************
  void turnTurretLeft() {
    this.turret.turnLeft();
  }

  //**************************************************
  // Det är denna metod som får tankens kanon att svänga höger.
  void turnTurretRight_state() {
    if (this.stop_state) { 
      if (!this.turning_turret_right_state) {
        println("*** Tank[" + getId() + "].turnTurretRight_state()");
        this.turning_turret_left_state = false;
        this.turning_turret_right_state = true;
        this.stop_turret_turning_state = false;
      }
    } else {
      println("Tanken måste stå still för att kunna rotera kanonen.");
    }
  }

  //**************************************************
  void turnTurretRight() {
    this.turret.turnRight();
  }

  //**************************************************
  // Det är denna metod som får tankens kanon att sluta rotera.
  void stopTurretTurning_state() {
    if (!this.stop_turret_turning_state) {
      println("*** Tank[" + getId() + "].stopTurretTurning_state()");
      this.turning_turret_left_state = false;
      this.turning_turret_right_state = false;
      this.stop_turret_turning_state = true;
    }
  }

  //**************************************************
  // Det är denna metod som får tanken att svänga vänster.
  void turnLeft_state() {
    this.stop_turning_state = false;
    this.turning_right_state = false;

    if (!this.turning_left_state) {
      println("*** Tank[" + getId() + "].turnLeft_state()");
      this.turning_left_state = true;
    }
  }

  //**************************************************
  void turnLeft() {
    //println("*** Tank[" + getId()+"].turnLeft()");

    if (this.hasTarget && abs(this.targetHeading - this.heading) < this.maxrotationspeed) {
      this.rotation_speed -= this.maxforce;
    } else {
      this.rotation_speed += this.maxforce;
    }
    if (this.rotation_speed > this.maxrotationspeed) {
      this.rotation_speed = this.maxrotationspeed;
    }
    this.heading -= this.rotation_speed;
  }

  //**************************************************
  // Det är denna metod som får tanken att svänga höger.
  void turnRight_state() {
    this.stop_turning_state = false;
    this.turning_left_state = false;

    if (!this.turning_right_state) {
      println("*** Tank[" + getId() + "].turnRight_state()");
      this.turning_right_state = true;
    }
  }

  //**************************************************  
  void turnRight() {
    //println("*** Tank[" + getId() + "].turnRight()");

    if (this.hasTarget && abs(this.targetHeading - this.heading) < this.maxrotationspeed) {
      this.rotation_speed -= this.maxforce;
    } else {
      this.rotation_speed += this.maxforce;
    }
    if (this.rotation_speed > this.maxrotationspeed) {
      this.rotation_speed = this.maxrotationspeed;
    }
    this.heading += this.rotation_speed;
  }

  //**************************************************  
  void turnRight(PVector targetPos) {
    println("*** Tank[" + getId() + "].turnRight(PVector)");
    PVector desired = PVector.sub(targetPos, position);  // A vector pointing from the position to the target

    desired.setMag(0);
    PVector steer = PVector.sub(desired, velocity);
    steer.limit(maxforce);  // Limit to maximum steering force
    applyForce(steer);
  }

  //**************************************************  
  void stopTurning() {
    println("*** Tank[" + getId()+"].stopTurning()");
    this.rotation_speed = 0;
    arrivedRotation();
  }

  //**************************************************
  // Det är denna metod som får tanken att sluta svänga.
  void stopTurning_state() {
    if (!this.stop_turning_state) {
      println("*** Tank[" + getId() + "].stopTurning_state()");
      this.turning_left_state = false;
      this.turning_right_state = false;
      this.stop_turning_state = true;

      println("! Tank[" + getId() + "].stopTurning_state() – stop_turning_state=true");
    }
  }

  //**************************************************
  void moveTo(float x, float y) {
    //println("*** Tank["+ this.getId() + "].moveTo(" +x +", "+ y+")");

    moveTo(new PVector(x, y));
  }

  //**************************************************
  void moveTo(PVector coord) {
    //println("*** Tank["+ this.getId() + "].moveTo(PVector)");
    if (!isImmobilized) {
      //println("*** Tank["+ this.getId() + "].moveTo(" +coord.x +", "+ coord.y+")");


      this.idle_state = false;
      this.isMoving = true;
      this.stop_state = false;

      this.targetPosition.set(coord);

      this.hasTarget = true;
    }
  }

  //**************************************************
  void moveBy(float x, float y) {
  //  println("*** Tank["+ this.getId() + "].moveBy(float x, float y)");

    moveBy(new PVector(x, y));
  }

  //**************************************************
  void moveBy(PVector coord) {
//    println("*** Tank["+ this.getId() + "].moveBy(PVector)");

    PVector newCoord = PVector.add(this.position, coord);
    PVector nodevec = grid.getNearestNodePosition(newCoord);

    moveTo(nodevec);
  }

  //**************************************************
  // Det är denna metod som får tanken att gå framåt.
  void moveForward_state() {
    println("*** Tank[" + getId() + "].moveForward_state()");

    if (!this.forward_state) {
      this.acceleration.set(0, 0, 0);
      this.velocity.set(0, 0, 0);

      this.forward_state = true;
      this.backward_state = false;
      this.stop_state = false;
    }
  }

  //**************************************************
  void moveForward() {
    //println("*** Tank[" + getId() + "].moveForward()");

    // Offset the angle since we drew the ship vertically
    float angle = this.heading; // - PI/2;
    // Polar to cartesian for force vector!
    PVector force = new PVector(cos(angle), sin(angle));
    force.mult(0.1);
    applyForce(force);
  }

  //**************************************************
  void moveForward(int numSteps) {
  }

  //**************************************************
  // Det är denna metod som får tanken att gå bakåt.
  void moveBackward_state() {
    println("*** Tank[" + getId() + "].moveBackward_state()");
    this.stop_state = false;
    this.forward_state = false;

    if (!this.backward_state) {
      println("! Tank[" + getId() + "].moveBackward_state() – (!this.backward_state)");
      this.acceleration.set(0, 0, 0);
      this.velocity.set(0, 0, 0);
      this.backward_state = true;
    }
  }

  //**************************************************
  void moveBackward() {
    println("*** Tank[" + getId() + "].moveBackward()");
    // Offset the angle since we drew the ship vertically
    float angle = this.heading - PI; // - PI/2;
    // Polar to cartesian for force vector!
    PVector force = new PVector(cos(angle), sin(angle));
    force.mult(0.1);
    applyForce(force);
  }

  //**************************************************
  void stopMoving() {
    println("*** Tank[" + getId() + "].stopMoving()");

    this.acceleration.set(0, 0, 0);
    this.velocity.set(0, 0, 0);

    this.isMoving = false;  

    resetTargetStates();
  }

  //**************************************************
  // Det är denna metod som får tanken att sluta åka framåt eller bakåt.
  // "this.stop_state" anropas 
  void stopMoving_state() {
    //println("stopMoving_state() ");

    if (!this.stop_state) {
      //println("*** Tank[" + getId() + "].stopMoving_state()");

      resetMovingStates();
      stopMoving();
    }
  }

  //**************************************************
  void resetAllMovingStates() {
    println("*** Tank[" + getId() + "].resetAllMovingStates()");
    this.stop_state = true;
    this.backward_state = false;
    this.forward_state = false;

    this.backward_state = false;
    this.forward_state = false;
    this.turning_right_state = false;
    this.turning_left_state = false;
    this.turning_turret_right_state = false;
    this.turning_turret_left_state = false;
    this.stop_state = true;
    this.stop_turning_state = true;
    this.stop_turret_turning_state = true;

    this.velocity = new PVector(0, 0);
    this.acceleration = new PVector(0, 0);
  }

  //**************************************************
  void resetMovingStates() {
    println("*** Tank[" + getId() + "].resetMovingStates()");
    this.stop_state = true;
    this.backward_state = false;
    this.forward_state = false;
  }

  //**************************************************
  void resetTargetStates() {
    println("*** Tank[" + getId() + "].resetTargetStates()");
    this.targetPosition = new PVector(this.position.x, this.position.y);

    this.targetHeading = this.heading; // Tanks har alltid en heading mot ett target.
    this.hasTarget = false;
  }
  int count = 0;

  //**************************************************
  void updatePosition() {
    this.positionPrev.set(this.position); // spara senaste pos.
    if (this.team_id == 0) {/*
      println("positionen " + this.position + " hos tank " + this.id);
     println("acceleration " + this.acceleration + " hos tank " + this.id);
     println("velocity " + this.velocity + " hos tank " + this.id);*/
    }
    this.velocity.add(this.acceleration);
    this.velocity.limit(this.maxspeed);
    this.position.add(this.velocity);
    this.acceleration.mult(0);
    count++;
    if (count % 10 == 0) {
      //    pause = true;
    }
  }

  //**************************************************
  // Newton's law: F = M * A
  void applyForce(PVector force) {
    this.acceleration.add(force);
  }

  //**************************************************
  public void destroy() {
    println("*** Tank.destroy()");
    //dead = true;
    this.isDestroyed = true;
  }

  //**************************************************

  void rotating() {
    //println("*** Tank["+ this.getId() + "].rotating()");
    if (!isImmobilized) {

      if (this.hasTarget) {
        float diff = this.targetHeading - this.heading;

        if ((abs(diff) <= radians(0.5))) {
          this.isRotating = false;
          this.heading = this.targetHeading;
          this.targetHeading = 0.0;
          this.hasTarget = false;
          stopTurning_state();
          arrivedRotation();
        } else if ((diff) > radians(0.5)) {

          turnRight_state();
        } else if ((diff) < radians(0.5)) {
          turnLeft_state();
        }
      }
    }
  }

  //**************************************************

  void rotateTo(float angle) {
    println("*** Tank["+ this.getId() + "].rotateTo(float): "+angle);

    if (!isImmobilized) {

      this.heading = angle;

      // Hitta koordinaten(PVector) i tankens riktning
      Sensor sens = getSensor("ULTRASONIC_FRONT");
      PVector sens_pos = (sens.readValue().obj().position);
      PVector grid_pos = grid.getNearestNodePosition(sens_pos);
      rotateTo(grid_pos); // call "rotateTo(PVector)"
    }
  }

  //**************************************************

  void rotateTo(PVector coord) {
    //println("*** Tank["+ this.getId() + "].rotateTo(PVector) – ["+(int)coord.x+","+(int)coord.y+"]");

    if (!isImmobilized) {

      this.idle_state = false;
      this.isMoving = false;
      this.isRotating = true;
      this.stop_state = false;
      this.hasTarget = true;


      PVector target = new PVector(coord.x, coord.y);
      PVector me = new PVector(this.position.x, this.position.y);

      // Bestäm headin till target.
      PVector t = PVector.sub(target, me);
      this.targetHeading = t.heading();
    }
  }

  //**************************************************
  // A method that calculates a steering force towards a target
  // STEER = DESIRED MINUS VELOCITY
  void arrive() {

    // rotera tills heading mot target.
    PVector desired = PVector.sub(this.targetPosition, this.position);  // A vector pointing from the position to the target
    float d = desired.mag();
    // If arrived

    // Scale with arbitrary damping within 100 pixels
    if (d < 100) {
      float m = map(d, 0, 100, 0, maxspeed);
      desired.setMag(m);
    } else {
      desired.setMag(maxspeed);
    }

    // Steering = Desired minus Velocity
    PVector steer = PVector.sub(desired, velocity);
    steer.limit(maxforce);  // Limit to maximum steering force
    applyForce(steer);

    if (d < 1) {
      arrived();
    }
  }

  //**************************************************
  // Tanken meddelas om att tanken är redo efter att blivit träffad.
  void readyAfterHit() {
    println("*** Tank["+ this.getId() + "].readyAfterHit()");

    if (!this.isDestroyed) {
      this.isReady = true; // Efter träff kan inte tanken utföra action, så länge den "snurrar".
    }
  }

  //**************************************************
  // Tanken meddelas om kollision med trädet.
  void arrivedRotation() {
    println("*** Tank["+ this.getId() + "].arrivedRotation()");
    stopTurning_state();
    this.isMoving = false;
  }

  //**************************************************
  void arrived() {
    println("*** Tank["+ this.getId() + "].arrived()");
    target.isEmpty = false;
    this.isMoving = false;  
    okayToGoNextStepHome = true;
    stopMoving_state();
  }

  //**************************************************
  // Är tänkt att överskuggas (override) i subklassen.
  void updateLogic() {
  }

  //**************************************************
  // Called from game
  final void update() {
    // Om tanken fortfarande lever.

    if (!this.isDestroyed) {
      // Om tanken har blivit träffad och håller på och snurrar runt.
      int spinning_speed = 5;
      if (this.isSpinning) {
        if (this.remaining_turns > 0) {
          this.heading += rotation_speed * spinning_speed; 

          if (this.heading > (this.heading_saved + (2 * PI))||(this.heading == this.heading_saved)) {

            this.remaining_turns -= 1;
            this.heading = this.heading_saved;
          }
        } else {

          this.heading = this.heading_saved;
          this.remaining_turns = 0;
          this.isSpinning = false;
          this.isReady = true;

          this.idle_state = true;
        }
      } else {

        // Om tanken är redo för handling och kan agera.
        if (!this.isImmobilized && this.isReady) {

          /*
          if (patrolling) {
           keepPatrolling();
           } */

          // Om tanken är i rörelse.
          if (this.isMoving) {

            this.heading = this.velocity.heading();
            arrive();
          }


          // Om tanken ska stanna, men ännu inte gjort det.
          if (this.stop_state && !this.idle_state) {

            resetTargetStates(); // Tank
            resetAllMovingStates(); // Tank 
            this.idle_state = true;

            println("! Tank[" + getId() + "].update() – idle_state = true");
          }

          // Om tanken håller på och rotera.
          if (this.isRotating) {
            rotating();
          }

          // ----------------
          // state-kontroller
          if (this.forward_state) {
            moveForward();
          }
          if (this.backward_state) {
            moveBackward();
          }
          if (this.turning_right_state) {
            turnRight();
          }
          if (this.turning_left_state) {
            turnLeft();
          }

          if (this.stop_state && !this.isMoving && this.hasTarget) {
            println("Tank["+ this.getId() + "], vill stanna!");
            //this.stop_state = false;
            stopMoving();
          }
          if (this.stop_turning_state && !this.isMoving && this.hasTarget) {
            println("Tank["+ this.getId() + "], vill sluta rotera!");
            stopTurning();
          }
        } // end (!this.isImmobilized && this.isReady)



        // Om tanken är immobilized
        // Om tanken har laddat ett skott.
        if (this.hasShot) {
          this.ball.updateLoadedPosition(this.position);
        }

        //---------------
        // state-kontroller ...
        if (this.turning_turret_left_state) {
          turnTurretLeft();
        }
        if (this.turning_turret_right_state) {
          turnTurretRight();
        }

        readSensors();
      }
      updateContent();
      updatePosition();
    }
  }

  //**************************************************
  void updateContent() {
    if (!(this.lastVisitedNode == null)) {
      grid.removeContent(this.lastVisitedNode);
    }
    grid.addContent(this);

    if (this.id == 1) {
      //showContentGrid();
    }
  }

  //**************************************************
  void showContentGrid() {
    Node[][] nodes = grid.nodes;
    println();
    for (int i = 0; i < nodes.length; i++) {
      for (int j= 0; j < nodes.length; j++) {
        if (!(nodes[j][i].content() == null)) {
          print(nodes[j][i].content().getName()+" ");
        } else {
          print(nodes[j][i].content()+" ");
        }
      }
      println();
    }
  }

  //**************************************************
  // Anropas från spelet.
  void checkEnvironment() {
    //checkEnvironment_sensor();

    // Check for collisions with Canvas Boundaries
    float r = this.diameter/2;
    if ((this.position.y+r > height) || (this.position.y-r < 0) ||
      (this.position.x+r > width) || (this.position.x-r < 0)) {
      if (!this.stop_state) {
        this.position.set(this.positionPrev); // Flytta tillbaka.
        //println("***");
        stopMoving_state();
      }
    }

    if (
      position.x + radius > team.homebase_x && 
      position.x + radius < team.homebase_x+team.homebase_width &&
      position.y + radius > team.homebase_y &&
      position.y + radius < team.homebase_y+team.homebase_height) {
      if (!isAtHomebase) {
        isAtHomebase = true;

        // checks if reporting tank has just arrived to the homebase
        if (isReporting) {
          timer = new StopWatchTimer();
          timer.start();
          isReporting = false;
          isReportingInHomebase = true;

          //showGrid();
        }
        message_arrivedAtHomebase();
      }
    } else {
      isAtHomebase = false;
    }
  }

  // Tanken meddelas om att tanken är i hembasen.
  public void message_arrivedAtHomebase() {
    println("! Tank["+ this.getId() + "] – har kommit hem.");
  }

  // Tanken meddelas om kollision med trädet.
  public void message_collision(Tree other) {
    println("*** Tank["+ this.getId() + "].collision(Tree)");
    //println("Tank.COLLISION");
  }

  // Tanken meddelas om kollision med den andra tanken.
  public void message_collision(Tank other) {
    println("*** Tank["+ this.getId() + "].collision(Tank)");
    //println("Tank.COLLISION");
  }

  //**************************************************
  void checkCollision(Tree other) {
    //println("*** Tank.checkCollision(Tree)");
    // Check for collisions with "no Smart Objects", Obstacles (trees, etc.)

    // Get distances between the tree component
    PVector distanceVect = PVector.sub(other.position, this.position);

    // Calculate magnitude of the vector separating the tank and the tree
    float distanceVectMag = distanceVect.mag();

    // Minimum distance before they are touching
    float minDistance = this.radius + other.radius;

    if (distanceVectMag <= minDistance && !this.stop_state) {

      println("! Tank["+ this.getId() + "] – collided with Tree.");

      if (!this.stop_state) {
        this.position.set(this.positionPrev); // Flytta tillbaka.

        // Kontroll om att tanken inte "fastnat" i en annan tank. 
        distanceVect = PVector.sub(other.position, this.position);
        distanceVectMag = distanceVect.mag();
        collidedWithTree = true;
        if (distanceVectMag < minDistance) {
          println("! Tank["+ this.getId() + "] – FAST I ETT TRÄD");
        }

        stopMoving_state();
      }

      if (this.hasShot) {
        this.ball.updateLoadedPosition(this.positionPrev);
      }


      // Meddela tanken om att kollision med trädet gjorts.
      message_collision( other);//collision(Tree);
    }
  }

  //**************************************************
  // Called from environment
  // Keeps an array with vectors to the other tanks, so the tank object can access the other tanks when called for.
  void checkCollision(Tank other) {
    //println("*** Tank.checkCollision(Tank)");
    // Check for collisions with "Smart Objects", other Tanks.

    // Get distances between the tanks components
    PVector distanceVect = PVector.sub(other.position, this.position);

    // Calculate magnitude of the vector separating the tanks
    float distanceVectMag = distanceVect.mag();

    // Minimum distance before they are touching
    float minDistance = this.radius + other.radius;

    if (distanceVectMag <= minDistance) {
      println("! Tank["+ this.getId() + "] – collided with another Tank" + other.team_id + ":"+other.id);
      collidedWithTank = true;
      this.position.set(this.positionPrev); // Flytta tillbaka.
      if (!this.stop_state) {
        //this.position.set(this.positionPrev); // Flytta tillbaka.

        // Kontroll om att tanken inte "fastnat" i en annan tank. 
        distanceVect = PVector.sub(other.position, this.position);
        distanceVectMag = distanceVect.mag();


        if (distanceVectMag <= minDistance) {
          println("! Tank["+ this.getId() + "] – FAST I EN ANNAN TANK");
        }

        this.isMoving = false;  
        stopMoving_state();
      }

      if (this.hasShot) {
        this.ball.updateLoadedPosition(this.positionPrev);
      }


      // Meddela tanken om att kollision med den andra tanken gjorts.
      message_collision(other);
    }
    checkForward();
    /*
    checkTankForward(other);
     checkTankForward();
     */
  }

  void setNode() {
    //setTargetPosition(this.position);
  }

  void displayInfo() {
    fill(230);
    rect(width - 151, 0, 150, 300);
    strokeWeight(1);
    fill(255, 0, 0);
    stroke(255, 0, 0);
    textSize(10);
    text("id: "+this.id+"\n"+
      "health: "+this.health+"\n"+
      "position: ("+(int)this.position.x +","+(int)this.position.y+")"+"\n"+
      "isMoving: "+this.isMoving+"\n"+
      "isSpinning : "+this.isSpinning +"\n"+
      "remaining_turns: "+this.remaining_turns +"\n"+
      "isReady : "+this.isReady +"\n"+
      "hasTarget : "+this.hasTarget +"\n"+
      "stop_state : "+this.stop_state +"\n"+
      "stop_turning_state : "+this.stop_turning_state +"\n"+
      "idle_state : "+this.idle_state +"\n"+
      "isDestroyed : "+this.isDestroyed +"\n"+
      "isImmobilized : "+this.isImmobilized +"\n"+
      "targetHeading : "+this.targetHeading +"\n"+
      "heading : "+this.heading +"\n"+
      "heading_saved: "+this.heading_saved +"\n"
      , width - 145, 35 );
  }

  //**************************************************
  void drawTank(float x, float y) {
    fill(this.team.getColor()); 

    if (this.team.getId() == 0) fill((((255/6) * this.health) *40 ), 50* this.health, 50* this.health, 255 - this.health*60);
    if (this.team.getId() == 1) fill(10*this.health, (255/6) * this.health, (((255/6) * this.health) * 3), 255 - this.health*60);

    if (this.userControlled) {
      strokeWeight(3);
    } else strokeWeight(1);

    ellipse(x, y, 50, 50);
    strokeWeight(1);
    line(x, y, x+25, y);

    fill(this.team.getColor(), 255); 
    this.turret.display();
  }

  //**************************************************
  final void display() {

    imageMode(CENTER);
    pushMatrix();
    translate(this.position.x, this.position.y);

    rotate(this.heading);

    //image(img, 20, 0);
    drawTank(0, 0);

    if (debugOn) {
      noFill();
      strokeWeight(2);
      stroke(255, 0, 0);
      ellipse(0, 0, this.radius * 2, this.radius * 2);

      //for (Sensor s : mySensors) {
      //  if (s.tank == this) {
      //     strokeWeight(2);
      //     stroke(0,0,255); 
      //     PVector sens = s.readValue1();
      //     println("============");
      //     println("("+sens.x + " , "+sens.y+")");
      //     //ellipse(sens.x, sens.y, 10,10);
      //  }
      //}
    }
    //   arc(0, 0, sightDistance*2, sightDistance*2, -periphery, periphery);

    popMatrix();  

    if (pause) {
      PVector mvec = new PVector(mouseX, mouseY);
      PVector distanceVect = PVector.sub(mvec, this.position);
      float distanceVectMag = distanceVect.mag();
      if (distanceVectMag < getRadius()) {
        displayInfo();
      }
    }

    if (debugOn) {

      for (Sensor s : mySensors) {
        if (s.tank == this) {
          // Rita ut vad sensorn ser (target och linje dit.)
          strokeWeight(1);
          stroke(0, 0, 255); 
          PVector sens = (s.readValue().obj().position);

          //println("============");
          //println("("+sens.x + " , "+sens.y+")");
          //ellipse(sens.x, sens.y, 10,10);

          if ((sens != null && !this.isSpinning && !isImmobilized)) {
            line(this.position.x, this.position.y, sens.x, sens.y);
            ellipse(sens.x, sens.y, 10, 10);
            //println("Tank" + this.team.getId() + ":"+this.id + " ( " + sens.x + ", "+ sens.y + " )");
          }
        }
      }

      // Rita ut en linje mot target, och tank-id och tank-hälsa.
      strokeWeight(2);
      fill(255, 0, 0);
      stroke(255, 0, 0);
      textSize(14);
      text(this.id+":"+this.health, this.position.x + this.radius, this.position.y + this.radius);

      if (obst != null) {
        strokeWeight(1);
        line(this.position.x, this.position.y, this.obst.x, obst.y);
      }
    }
  }

  //************************************************************************************



  void showGrid() {
    for (int i = 0; i < grid.nodes.length; i++) {
      for (int j = 0; j < grid.nodes[i].length; j++) {
        if (team.patrolled.containsKey(grid.nodes[j][i])) {
          if (team.patrolled.get(grid.nodes[j][i]) == Integer.MAX_VALUE) {
          } else {
          }
        } else {
        }
      }
    }
  }

  HashMap checkForward() {
    ArrayList<Sprite> allEnemiesAndTrees = fillList();
    HashMap<Sprite, PVector> sighted = new HashMap<Sprite, PVector>();
    float shortest = 100000;
    PVector closest = null;
    Sprite closestSprite = null;

    //***********************************************************************************************************TODO Kolla om inom fiendebas


    //http://rosettacode.org/wiki/Bitmap/Midpoint_circle_algorithm#Java
    // checks radius of every Sprite except yourself using Midpoint circle algorithm to iterate through, 
    // then chooses the closest one which is what the tank sees. 

    for (Sprite s : allEnemiesAndTrees) {   
      float d = (5 - s.radius * 4)/4;
      float x = 0;
      float y = s.radius;

      do {
        if (checkCollision(s.position.x + x, s.position.y + y)) {
          PVector near = new PVector(s.position.x+x, s.position.y + y);
          if (PVector.dist(near, position) < shortest) {
            shortest = PVector.dist(near, position);
            closest = near;
            closestSprite = s;
          }
        }
        if (checkCollision(s.position.x + x, s.position.y - y)) {
          PVector near = new PVector(s.position.x+x, s.position.y - y);
          if (PVector.dist(near, position) < shortest) {
            shortest = PVector.dist(near, position);
            closest = near;
            closestSprite = s;
          }
        }
        if (checkCollision(s.position.x - x, s.position.y + y)) {
          PVector near = new PVector(s.position.x-x, s.position.y + y);
          if (PVector.dist(near, position) < shortest) {
            shortest = PVector.dist(near, position);
            closest = near;
            closestSprite = s;
          }
        }
        if (checkCollision(s.position.x - x, s.position.y - y)) {
          PVector near = new PVector(s.position.x-x, s.position.y - y);
          if (PVector.dist(near, position) < shortest) {
            shortest = PVector.dist(near, position);
            closest = near;
            closestSprite = s;
          }
        }                        
        if (checkCollision(s.position.x + y, s.position.y + x)) {
          PVector near = new PVector(s.position.x+y, s.position.y + x);
          if (PVector.dist(near, position) < shortest) {
            shortest = PVector.dist(near, position);
            closest = near;
            closestSprite = s;
          }
        }                        
        if (checkCollision(s.position.x + y, s.position.y - x)) {
          PVector near = new PVector(s.position.x+y, s.position.y + x);
          if (PVector.dist(near, position) < shortest) {
            shortest = PVector.dist(near, position);
            closest = near;
            closestSprite = s;
          }
        }                        
        if (checkCollision(s.position.x - y, s.position.y + x)) {
          PVector near = new PVector(s.position.x-y, s.position.y + x);
          if (PVector.dist(near, position) < shortest) {
            shortest = PVector.dist(near, position);
            closest = near;
            closestSprite = s;
          }
        }                        
        if (checkCollision(s.position.x - y, s.position.y - x)) {
          PVector near = new PVector(s.position.x-y, s.position.y -x);
          if (PVector.dist(near, position) < shortest) {
            shortest = PVector.dist(near, position);
            closest = near;
            closestSprite = s;
          }
        }                        
        if (d < 0) {
          d += 2 * x + 1;
        } else {
          d += 2 * (x - y) + 1;
          y--;
        }
        x++;
      } while (x <= y);
    }
    if (closest != null)
      sighted.put(closestSprite, closest);
    return sighted;
  }

//checks if a sprite is within cone of sight. 
  Boolean checkCollision(float x, float y) {
    PVector comp = new PVector(x, y);
    PVector comparision = PVector.sub(comp, this.position);
    float d = PVector.dist(this.position, comp);
    float diff = PVector.angleBetween(comparision, this.velocity);
    if (diff < periphery && d > 0 && d < sightDistance) {
      return true;
    }
    return false;
  }

// fill lista med sprites med alla sprites utom tanken själv
// och heller inte om någon är camouflaged
  ArrayList fillList() {
    ArrayList<Sprite> list = new ArrayList<Sprite>(8);
    for (Tank t : allTanks) {
      if (t != this) {
        if(t.team_id != this.team_id && t.isAtHomebase && !inEnemyBase(this, t)){
        }
        list.add(t);
      }
    }
    for (Tree tree : allTrees) {
      list.add(tree);
    }
    return list;
  }
  
  //used SOLELY for fillList to check if the tank being looked at is in fact, not at all being looked at because it is in it's camp and camouflaged
   Boolean inEnemyBase(Tank me, Tank other){   
    if (
      me.position.x > other.team.homebase_x && 
      me.position.x < other.team.homebase_x+other.team.homebase_width &&
      me.position.y > other.team.homebase_y &&
      me.position.y < other.team.homebase_y+other.team.homebase_height) {
        return true;
    } else {
      return false;
    }
   }

  Direction getDirection() {
    float heading = this.getHeadingInDegrees();

    if ((heading > 22.5) && (heading < 67.6)) {
      return Direction.SOUTHEAST;
    } else if ((heading > 67.5) && (heading < 112.6)) {
      return Direction.SOUTH;
    } else if ((heading > 112.5) && (heading < 157.6)) {
      return Direction.SOUTHWEST;
    } else if ((heading > 157.5) && (heading < 202.6)) {
      return Direction.WEST;
    } else if ((heading > 202.5) && (heading < 247.6)) {
      return Direction.NORTHWEST;
    } else if ((heading > 247.5) && (heading < 292.6)) {
      return Direction.NORTH;
    } else if ((heading > 292.5) && (heading < 337.6)) {
      return Direction.NORTHEAST;
    } else {
      return Direction.EAST;
    }
  }


  // Check if the Node current is within homebase
  boolean isNodeInHomeBase(Node current) {
    if (
      current.x > team.homebase_x && 
      current.x <= team.homebase_x+team.homebase_width &&
      current.y > team.homebase_y &&
      current.y <= team.homebase_y+team.homebase_height) {
      return true;
    } else {
      return false;
    }
  }

  PVector obst;

  // We accumulate a new acceleration each time based on five rules
  void flock(Node target_from) {
    ArrayList<Tank> flock = new ArrayList(Arrays.asList(team.tanks));
    PVector sep = separate(flock);   // Separation
    PVector ali = align(flock);      // Alignment
    PVector coh = cohesion(flock);   // Cohesion
    PVector targ = seek_heur_target(target_from);
    //println("TARGET in tank :    "+target);
    PVector avoidObstacles = avoidObstacles();
    this.target = target_from;
    // Arbitrarily weight these forces
    sep.mult(2.8);
    ali.mult(1.0);
    coh.mult(1.0);
    targ.mult(1.5);
    avoidObstacles.mult(60);
    // Add the force vectors to acceleration
    applyForce(sep);
    applyForce(ali);
    applyForce(coh);
    applyForce(targ);
    applyForce(avoidObstacles);
    checkRotating();
    //println("positionen " + this.position + " hos tank " + this.id);
  }

// checkRotating changes the rotation to where the tank is planned to go
  Boolean checkRotating() {
    //PVector targ = PVector.add(this.position, this.velocity);
    //println("VELOCITY "+id+ " :  " +this.velocity);
    //println("POSITION "+id+ " :  " +this.position);
    //println(targ);
    this.hasTarget = true;
    //this.targetPosition.set(targ);
    rotateTo(PVector.add(this.position, this.velocity));
    return true;
  }

  // A method that calculates and applies a steering force towards a target
  // STEER = DESIRED MINUS VELOCITY
  PVector seek(PVector target) {
    PVector desired = PVector.sub(target, position);  // A vector pointing from the position to the target;esired);
    // Normalize desired and scale to maximum speed
    desired.normalize();
    desired.mult(maxspeed);
    // Steering = Desired minus Velocity
    PVector steer = PVector.sub(desired, velocity);
    steer.limit(maxforce);  // Limit to maximum steering force
    return steer;
  }

  // A method that calculates and applies a steering force towards a target
  // STEER = DESIRED MINUS VELOCITY
  PVector seek_heur_target(Node target_from) {
    target = target_from;
    PVector desired = PVector.sub(target_from.position, position);  // A vector pointing from the position to the target
    //println("desired:  "+desired);
    // Normalize desired and scale to maximum speed
    desired.normalize();
    desired.mult(maxspeed);
    // Steering = Desired minus Velocity
    PVector steer = PVector.sub(desired, velocity);
    steer.limit(maxforce);  // Limit to maximum steering force
    return steer;
  }

  // Separation
  // Method checks for nearby friendly tanks and steers away
  PVector separate (ArrayList<Tank> tanks) {
    float desiredseparation = this.radius*3;
    PVector steer = new PVector(0, 0, 0);
    int count = 0;
    // For every tank in the system, check if it's too close
    for (Tank other : tanks) {
      float d = PVector.dist(position, other.position);
      // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
      if ((d > 0) && (d < desiredseparation)) {
        // Calculate vector pointing away from neighbor
        PVector diff = PVector.sub(position, other.position);
        //println("diff " + diff);
        diff.normalize();
        diff.div(d);        // Weight by distance
        steer.add(diff);
        count++;            // Keep track of how many tanks
      }
    }
    // Average -- divide by how many
    if (count > 0) {
      steer.div((float)count);
    }

    // As long as the vector is greater than 0 //<>//
    if (steer.mag() > 0) {
      // Implement Reynolds: Steering = Desired - Velocity
      steer.normalize();
      steer.mult(maxspeed);
      steer.sub(velocity);
      steer.limit(maxforce);
    }
    //println(steer + " separate");
    return steer;
  }

  // Alignment
  // For every nearby tank in the system, calculate the average velocity
  PVector align (ArrayList<Tank> tanks) {
    float neighbordist = 300;
    PVector sum = new PVector(0, 0);
    int count = 0; //<>//
    for (Tank other : tanks) {
      float d = PVector.dist(position, other.position);
      if ((d > 0) && (d < neighbordist)) {
        sum.add(other.velocity);
        count++;
      }
    }
    if (count > 0) {
      sum.div((float)count);
      sum.normalize();
      sum.mult(maxspeed);
      PVector steer = PVector.sub(sum, velocity);
      steer.limit(maxforce);
      return steer; //<>// //<>//
    } else {
      return new PVector(0, 0);
    }
  }

  // Cohesion
  // For the average position (i.e. center) of all nearby tanks, calculate steering vector towards that position
  PVector cohesion (ArrayList<Tank> tanks) {
    float neighbordist = 300; 
    PVector sum = new PVector(0, 0);   // Start with empty vector to accumulate all positions
    int count = 0;
    for (Tank other : tanks) {
      float d = PVector.dist(position, other.position);
      if ((d > 0) && (d < neighbordist)) {
        //println("HÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄÄR");
        sum.add(other.position); // Add position
        count++; //<>//
      }
    }
    if (count > 0) {
      sum.div(count);
      return seek(sum);  // Steer towards the position
    } else {
      return new PVector(0, 0);
    }
  }

  PVector avoidObstacles () {
    PVector steer = new PVector(0, 0, 0);
    HashMap view = this.checkForward();

    if (!view.isEmpty()) {
      PVector obstacle_pos = (PVector) view.values().stream().findFirst().get();
      obst =obstacle_pos;
      Sprite obstacle = (Sprite) view.keySet().stream().findFirst().get();
      float desiredseparation = this.diameter;
      if (obstacle.getName() == "tree")
        desiredseparation += obstacle.radius+10; //add atleast the radius of the tree to the tanks diameter
      else {
        Tank other = (Tank) obstacle;
        if (other.team_id != this.team_id) {
          desiredseparation +=this.diameter; // if it's a enemytank, ad atleast the diameter of the tank besides the tanks diameter          TODO: OBS också FULKODAt :) cause that's how I roll
        }
      }
      float d = PVector.dist(position, obstacle_pos);
      // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
      if ((d > 0) && (d < desiredseparation)) {
        // Calculate vector pointing away from neighbor
        PVector diff = PVector.sub(position, obstacle_pos);
        //println("diff " + diff);
        diff.normalize();
        diff.div(d);        // Weight by distance
        steer.add(diff);
        println("Tank "+ id + "sees: "+ obstacle.getName()+ " at " + obstacle_pos + " -------------------------------------------------------------------");
      }
    } else {
      obst = null;
    }
    // As long as the vector is greater than 0
    if (steer.mag() > 0) {
      // Implement Reynolds: Steering = Desired - Velocity
      steer.normalize();
      steer.mult(maxspeed);
      steer.sub(velocity);
      steer.limit(maxforce);
    }
    //println(steer + " separate");
    return steer;
  }
  

}


private enum Direction {
  SOUTH, 
    NORTH, 
    EAST, 
    WEST, 
    NORTHEAST, 
    SOUTHEAST, 
    NORTHWEST, 
    SOUTHWEST
}
